"""Serial transport helper for the wearable."""

from __future__ import annotations

import logging
import time
from collections import deque
from typing import Any, Deque, Dict, Iterable, Optional

from .constants import DEBUG_AUDIO_LEVEL, SERIAL_PORT_DEFAULT, SERIAL_TIMEOUT_SEC, USB_BAUD_RATE

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - hardware-specific import
    serial = None

logger = logging.getLogger(__name__)

# Keep the device ~450 ms ahead of real time. Ordering constraint:
# prebuffer (0.4 s) < lead (0.45 s) < ring (0.768 s) − expected jitter.
AUDIO_STREAM_LEAD_SEC = 0.45


class DeviceLink:
    """Thin wrapper around a simple line-based serial protocol."""

    def __init__(self, port: str = SERIAL_PORT_DEFAULT, baudrate: int = USB_BAUD_RATE) -> None:
        self._port_name = port
        self._baudrate = baudrate
        self._serial: Optional[Any] = None
        self._audio_stream_active = False
        self._audio_bytes_per_second: Optional[float] = None
        self._audio_stream_t0: float = 0.0
        self._audio_bytes_sent: int = 0
        self._audio_label: str = ""
        self._audio_max_deficit: float = 0.0
        self._audio_last_deficit_log: float = 0.0
        self._audio_lead_established: bool = False
        # Non-audio device messages read while waiting for LOG audio_end are parked here
        # so read_messages() can still deliver them (H-2).
        self._pending_messages: Deque[Dict[str, Any]] = deque()

    def connect(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed; cannot talk to device.")
        if self._serial and self._serial.is_open:
            return
        self._serial = serial.Serial(
            self._port_name,
            baudrate=self._baudrate,
            timeout=SERIAL_TIMEOUT_SEC,
            write_timeout=3.0,
        )
        logger.info("Opened device link on %s @ %d baud", self._port_name, self._baudrate)

    def close(self) -> None:
        if self._serial:
            self._serial.close()
            logger.info("Closed device link on %s", self._port_name)
        self._serial = None

    @property
    def is_connected(self) -> bool:
        return bool(self._serial and self._serial.is_open)

    def _write_line(self, line: str) -> None:
        if not self.is_connected:
            self.connect()
        assert self._serial is not None
        self._serial.write((line + "\n").encode("ascii"))

    def _write_raw(self, payload: bytes) -> None:
        if not payload:
            return
        if not self.is_connected:
            self.connect()
        assert self._serial is not None
        written = self._serial.write(payload)
        if written != len(payload):
            raise RuntimeError(
                f"Short write while streaming audio ({written}/{len(payload)} bytes)"
            )
        # No flush() here: pacing is done by our own sleep math and the kernel + usbipd
        # pipeline the writes. A per-chunk tcdrain was a USB-IP round trip that pinned
        # throughput near real-time (H-1).

    def send_session_start(self, planet: str, audio_enabled: bool, cadence_sec: float) -> None:
        line = f"START planet={planet} audio={int(audio_enabled)} cadence={cadence_sec:.2f}"
        self._write_line(line)

    def send_session_end(self) -> None:
        self._write_line("END")

    def send_target_vector(self, vector: Iterable[float]) -> None:
        x, y, z = vector
        self._write_line(f"TARGET {x:.6f} {y:.6f} {z:.6f}")

    def send_idle(self) -> None:
        self._write_line("IDLE")

    def request_bucket_ping(self) -> None:
        self._write_line("PING")

    def send_tare(self) -> None:
        self._write_line("TARE")

    def stream_audio_chunk(self, label: str, payload: bytes, final: bool) -> None:
        import base64

        encoded = base64.b64encode(payload).decode("ascii")
        self._write_line(f"AUDIO {label} {int(final)} {encoded}")

    # ------------------------------------------------------------------
    # New binary audio streaming helpers

    def start_audio_stream(self, sample_rate: int, frame_count: int, label: str) -> None:
        if sample_rate <= 0:
            raise ValueError("sample_rate must be positive")
        if frame_count <= 0:
            raise ValueError("frame_count must be positive")
        safe_label = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in label) or "clip"
        self._write_line(f"AUDIO START {sample_rate} {frame_count} {safe_label}")
        self._audio_stream_active = True
        bytes_per_sample = 2  # 16-bit mono expected
        self._audio_bytes_per_second = sample_rate * bytes_per_sample
        self._audio_stream_t0 = time.perf_counter()
        self._audio_bytes_sent = 0
        self._audio_label = safe_label
        self._audio_max_deficit = 0.0
        self._audio_last_deficit_log = 0.0
        self._audio_lead_established = False

    def stream_audio_payload(self, payload: bytes) -> None:
        if not self._audio_stream_active:
            raise RuntimeError("Audio stream not started.")
        if not payload:
            return
        self._write_raw(payload)
        self._audio_bytes_sent += len(payload)
        if self._audio_bytes_per_second:
            # Pace against a fixed stream-start anchor with a constant lead. Never
            # re-anchor: after a host stall the next chunks go out back-to-back until
            # we are ahead again, so supply automatically catches up (proposal §4.1).
            target = (
                self._audio_stream_t0
                + self._audio_bytes_sent / self._audio_bytes_per_second
                - AUDIO_STREAM_LEAD_SEC
            )
            sleep_time = target - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
                if not self._audio_lead_established:
                    self._audio_lead_established = True
                    logger.log(DEBUG_AUDIO_LEVEL, "Audio lead established for %s", self._audio_label)
            else:
                # Still building (or rebuilding) the lead: the burst phase is expected to
                # run "late" by design, so don't log per chunk — track the worst deficit
                # and emit at most one line per second (H-5).
                deficit = -sleep_time
                if deficit > self._audio_max_deficit:
                    self._audio_max_deficit = deficit
                if deficit > 0.1:
                    now = time.perf_counter()
                    if now - self._audio_last_deficit_log >= 1.0:
                        self._audio_last_deficit_log = now
                        logger.log(DEBUG_AUDIO_LEVEL, "Audio lead deficit %d ms", int(deficit * 1000))

    def finish_audio_stream(self, wait: bool = True) -> None:
        if not self._audio_stream_active:
            return
        self._write_line("AUDIO END")
        # Per-clip throughput telemetry (H-4): ratio = audio-seconds / wall-seconds over
        # the send phase (computed before any device drain-wait).
        if self._audio_bytes_per_second and self._audio_bytes_sent:
            audio_sec = self._audio_bytes_sent / self._audio_bytes_per_second
            wall_sec = time.perf_counter() - self._audio_stream_t0
            ratio = audio_sec / wall_sec if wall_sec > 0 else 0.0
            logger.log(
                DEBUG_AUDIO_LEVEL,
                "audio stream %s: %.2fs audio, wall %.2fs, ratio %.2fx, max_deficit %dms",
                self._audio_label,
                audio_sec,
                wall_sec,
                ratio,
                int(self._audio_max_deficit * 1000),
            )
        # Device-authoritative drain (H-2): wait for the device's own LOG audio_end, which
        # fires only when the ring AND the DMA queue are empty. Replaces the old
        # sleep-to-real-time-end estimate, which under-waited by prebuffer + deficit and
        # truncated clip tails (F4).
        if wait and self._audio_bytes_sent:
            self._wait_for_audio_end()
        self._audio_stream_active = False
        self._audio_bytes_per_second = None
        self._audio_stream_t0 = 0.0
        self._audio_bytes_sent = 0
        self._audio_label = ""
        self._audio_max_deficit = 0.0

    def _wait_for_audio_end(self) -> None:
        if not self.is_connected:
            return
        assert self._serial is not None
        timeout = max(3.0, AUDIO_STREAM_LEAD_SEC + 1.5)
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            raw = self._serial.readline()
            if not raw:
                continue
            raw_str = raw.decode("utf-8", errors="ignore").strip()
            if not raw_str:
                continue
            if raw_str.startswith("LOG audio_end"):
                logger.log(DEBUG_AUDIO_LEVEL, "Device: %s", raw_str)
                return
            # Don't drop unrelated device traffic that arrives during the wait; park it so
            # read_messages() still delivers it in order.
            parsed = self._parse_line(raw_str)
            if parsed is not None:
                self._pending_messages.append(parsed)
        logger.warning(
            "Timed out after %.1fs waiting for device audio_end; continuing.", timeout
        )

    def _parse_line(self, raw_str: str) -> Optional[Dict[str, Any]]:
        parts = raw_str.split()
        if not parts:
            return None
        verb = parts[0].lower()
        if verb == "state" and len(parts) >= 2:
            return {"state": parts[1]}
        if verb == "bucket" and len(parts) >= 2:
            return {"bucket": parts[1]}
        if verb == "orient" and len(parts) >= 4:
            try:
                if len(parts) >= 7:
                    yaw, pitch, roll, east, north, up = map(float, parts[1:7])
                    return {
                        "orientation": {"east": east, "north": north, "up": up},
                        "euler": {"yaw": yaw, "pitch": pitch, "roll": roll},
                    }
                east, north, up = map(float, parts[1:4])
                return {"orientation": {"east": east, "north": north, "up": up}}
            except ValueError:
                logger.warning("Malformed ORIENT message: %s", raw_str)
                return None
        if verb == "event" and len(parts) >= 2:
            return {"event": parts[1]}
        # Device audio logs live at DEBUG-AUDIO so normal DEBUG runs stay free of
        # audio spam; run with --log-level DEBUG-AUDIO to see them.
        if raw_str.startswith("LOG audio_"):
            logger.log(DEBUG_AUDIO_LEVEL, "Device: %s", raw_str)
        else:
            logger.debug("Device message passthrough: %s", raw_str)
        return None

    def read_messages(self) -> Iterable[Dict[str, Any]]:
        if not self.is_connected:
            self.connect()
        assert self._serial is not None
        # Drain anything parked during a drain-wait first, preserving arrival order (H-2).
        while self._pending_messages:
            yield self._pending_messages.popleft()
        while True:
            raw = self._serial.readline()
            if not raw:
                break
            raw_str = raw.decode("utf-8", errors="ignore").strip()
            if not raw_str:
                continue
            parsed = self._parse_line(raw_str)
            if parsed is not None:
                yield parsed

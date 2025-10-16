"""Serial transport helper for the wearable."""

from __future__ import annotations

import logging
import time
from typing import Any, Dict, Iterable, Optional

from .constants import SERIAL_PORT_DEFAULT, SERIAL_TIMEOUT_SEC, USB_BAUD_RATE

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - hardware-specific import
    serial = None

logger = logging.getLogger(__name__)

class DeviceLink:
    """Thin wrapper around a simple line-based serial protocol."""

    def __init__(self, port: str = SERIAL_PORT_DEFAULT, baudrate: int = USB_BAUD_RATE) -> None:
        self._port_name = port
        self._baudrate = baudrate
        self._serial: Optional[Any] = None
        self._audio_stream_active = False
        self._audio_bytes_per_second: Optional[float] = None
        self._audio_next_deadline: float = 0.0

    def connect(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed; cannot talk to device.")
        if self._serial and self._serial.is_open:
            return
        self._serial = serial.Serial(
            self._port_name,
            baudrate=self._baudrate,
            timeout=SERIAL_TIMEOUT_SEC,
            write_timeout=1.0,
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
        self._serial.flush()

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
        self._audio_next_deadline = time.perf_counter()

    def stream_audio_payload(self, payload: bytes) -> None:
        if not self._audio_stream_active:
            raise RuntimeError("Audio stream not started.")
        if not payload:
            return
        self._write_raw(payload)
        if self._audio_bytes_per_second:
            self._audio_next_deadline += len(payload) / self._audio_bytes_per_second
            sleep_time = self._audio_next_deadline - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                self._audio_next_deadline = time.perf_counter()

    def finish_audio_stream(self) -> None:
        if not self._audio_stream_active:
            return
        self._write_line("AUDIO END")
        self._audio_stream_active = False
        self._audio_bytes_per_second = None
        self._audio_next_deadline = 0.0

    def read_messages(self) -> Iterable[Dict[str, Any]]:
        if not self.is_connected:
            self.connect()
        assert self._serial is not None
        while True:
            raw = self._serial.readline()
            if not raw:
                break
            raw_str = raw.decode("utf-8", errors="ignore").strip()
            if not raw_str:
                continue
            parts = raw_str.split()
            verb = parts[0].lower()
            if verb == "state" and len(parts) >= 2:
                yield {"state": parts[1]}
            elif verb == "bucket" and len(parts) >= 2:
                yield {"bucket": parts[1]}
            elif verb == "orient" and len(parts) >= 4:
                try:
                    if len(parts) >= 7:
                        yaw, pitch, roll, east, north, up = map(float, parts[1:7])
                        yield {
                            "orientation": {"east": east, "north": north, "up": up},
                            "euler": {"yaw": yaw, "pitch": pitch, "roll": roll},
                        }
                    else:
                        east, north, up = map(float, parts[1:4])
                        yield {"orientation": {"east": east, "north": north, "up": up}}
                except ValueError:
                    logger.warning("Malformed ORIENT message: %s", raw_str)
                    continue
            elif verb == "event" and len(parts) >= 2:
                yield {"event": parts[1]}
            else:
                logger.debug("Device message passthrough: %s", raw_str)

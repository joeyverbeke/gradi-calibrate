"""Session controller for the desktop companion app."""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional, Tuple

from . import buckets
from .audio import AudioPlayer
from .constants import ALLOW_DIAGONAL_BUCKETS, GUIDANCE_INTERVAL_SEC, TARGET_BROADCAST_INTERVAL_SEC
from .link import DeviceLink
from .planets import ObserverLocation, PlanetCoefficients, select_planet, target_unit_vector

logger = logging.getLogger(__name__)

_DIAGONAL_LABELS = {"up_right", "up_left", "down_right", "down_left"}
_DIAGONAL_FALLBACK = {
    "up_right": "up",
    "up_left": "up",
    "down_right": "down",
    "down_left": "down",
}


def _clamp(value: float, minimum: float = -1.0, maximum: float = 1.0) -> float:
    return max(minimum, min(maximum, value))


def _vector_to_az_el(vec: Tuple[float, float, float]) -> Tuple[float, float]:
    east, north, up = vec
    azimuth = (math.degrees(math.atan2(east, north)) + 360.0) % 360.0
    elevation = math.degrees(math.asin(_clamp(up)))
    return azimuth, elevation


def _wrap_degrees(delta: float) -> float:
    return ((delta + 180.0) % 360.0) - 180.0


def _collapse_decision_to_cardinal(decision: buckets.BucketDecision) -> str:
    abs_yaw = abs(decision.yaw_error_deg)
    abs_pitch = abs(decision.pitch_error_deg)
    if abs_yaw >= abs_pitch:
        return "right" if decision.yaw_error_deg >= 0.0 else "left"
    return "up" if decision.pitch_error_deg >= 0.0 else "down"


@dataclass
class SessionState:
    active: bool = False
    current_planet: Optional[PlanetCoefficients] = None
    target_vector: Optional[tuple[float, float, float]] = None
    last_target_broadcast: float = 0.0
    last_guidance_prompt: float = 0.0
    cadence_sec: float = GUIDANCE_INTERVAL_SEC


class SessionController:
    def __init__(
        self,
        link: DeviceLink,
        audio: AudioPlayer,
        location: ObserverLocation,
        cadence_sec: float = GUIDANCE_INTERVAL_SEC,
        auto_tare: bool = True,
    ) -> None:
        self._link = link
        self._audio = audio
        self._location = location
        self._state = SessionState(cadence_sec=cadence_sec)
        self._last_orientation: Optional[dict] = None
        self._last_bucket_eval: Optional[buckets.BucketDecision] = None
        self._last_bucket_output: Optional[str] = None
        self._auto_tare = auto_tare
        self._device_state: Optional[str] = None
        self._pending_tare: bool = auto_tare
        self._last_tare_attempt: float = 0.0
        self._tare_retry_interval_sec: float = 1.0

    def run_forever(self) -> None:
        logger.info("Starting session loop; press Ctrl+C to exit.")
        try:
            self._send_idle_safe()
            while True:
                self._poll_device()
                if self._auto_tare:
                    self._process_tare(time.monotonic())
                self._tick()
                time.sleep(0.05)
        except KeyboardInterrupt:
            logger.info("Session loop interrupted by user.")
            self._shutdown(abort=True)
        finally:
            self._link.close()

    def _send_idle_safe(self) -> None:
        try:
            self._link.send_idle()
        except Exception as exc:
            logger.debug("Failed to send IDLE command: %s", exc)

    def _start_session(self) -> None:
        dt = datetime.now(timezone.utc)
        planet = select_planet(dt)
        target_vec = target_unit_vector(dt, self._location, planet)
        self._state.active = True
        self._state.current_planet = planet
        self._state.target_vector = target_vec
        self._state.last_target_broadcast = 0.0
        self._last_orientation = None
        self._last_bucket_eval = None
        self._last_bucket_output = None
        logger.info("Session start: target planet %s", planet.name.capitalize())
        self._link.send_session_start(planet.name, self._audio.should_stream, self._state.cadence_sec)
        self._audio.play_intro(planet.name)

    def _end_session(self) -> None:
        if not self._state.active:
            return
        logger.info("Session ended; returning to idle.")
        self._audio.stop_streaming()
        self._link.send_session_end()
        self._audio.play_outro()
        self._state = SessionState(cadence_sec=self._state.cadence_sec)
        self._last_orientation = None
        self._last_bucket_eval = None
        self._last_bucket_output = None
        self._pending_tare = self._auto_tare

    def _tick(self) -> None:
        if not self._state.active or self._state.target_vector is None:
            return
        now = time.monotonic()
        if now - self._state.last_target_broadcast >= TARGET_BROADCAST_INTERVAL_SEC:
            self._link.send_target_vector(self._state.target_vector)
            self._state.last_target_broadcast = now

    def _process_tare(self, now: float) -> None:
        if not self._pending_tare:
            return
        if self._device_state != "idle":
            return
        if now - self._last_tare_attempt < self._tare_retry_interval_sec:
            return
        logger.info("Requesting dock tare.")
        self._link.send_tare()
        self._last_tare_attempt = now

    def _compute_output_label_from_decision(self, decision: buckets.BucketDecision) -> str:
        label = decision.label
        if not ALLOW_DIAGONAL_BUCKETS and label in _DIAGONAL_LABELS:
            label = _collapse_decision_to_cardinal(decision)
        return label

    def _resolve_bucket_label(self, raw_label: Optional[str]) -> Optional[str]:
        if self._last_bucket_output:
            return self._last_bucket_output
        if raw_label is None:
            return None
        label = raw_label
        if not ALLOW_DIAGONAL_BUCKETS and label in _DIAGONAL_LABELS:
            label = _DIAGONAL_FALLBACK.get(label, label)
        return label

    def _poll_device(self) -> None:
        for message in self._link.read_messages():
            logger.debug("Device -> PC: %s", message)
            state = message.get("state")
            bucket_label_raw = message.get("bucket")
            event = message.get("event")

            if state == "motion_start":
                self._device_state = "motion_start"
                if not self._state.active:
                    self._start_session()
                continue
            if state == "idle":
                self._device_state = "idle"
                if self._auto_tare:
                    self._pending_tare = True
                if self._state.active:
                    self._end_session()
                continue
            if state == "guiding" and not self._state.active:
                # Device already active; align our state.
                self._start_session()
            if state == "guiding":
                self._device_state = "guiding"

            if event:
                if event == "tared":
                    logger.info("Wearable tare acknowledged.")
                    self._pending_tare = False
                elif event == "tare_wait":
                    logger.info("Wearable needs orientation before tare completes.")
                    # keep pending true; allow retry after cooldown
                    continue

            orientation = message.get("orientation")
            euler = message.get("euler")
            if orientation:
                self._last_orientation = {
                    "vector": (
                        orientation.get("east", 0.0),
                        orientation.get("north", 0.0),
                        orientation.get("up", 0.0),
                    ),
                    "euler": euler,
                }
                if self._state.target_vector:
                    aim_vec = (
                        orientation.get("east", 0.0),
                        orientation.get("north", 0.0),
                        orientation.get("up", 0.0),
                    )
                    decision = buckets.bucketize_direction(aim_vec, self._state.target_vector)
                    self._last_bucket_eval = decision
                    self._last_bucket_output = self._compute_output_label_from_decision(decision)

            bucket_label = None
            if bucket_label_raw:
                bucket_label = self._resolve_bucket_label(bucket_label_raw)
                print(self._format_bucket_debug(bucket_label))
                self._audio.play_bucket(bucket_label)
                decision = self._last_bucket_eval
                expected = self._last_bucket_output
                if not expected and decision:
                    computed_label = self._compute_output_label_from_decision(decision)
                    expected = computed_label
                if decision and expected and bucket_label != expected:
                    logger.debug(
                        "Bucket mismatch (device=%s local=%s | yaw=%.2f pitch=%.2f)",
                        bucket_label_raw,
                        expected,
                        decision.yaw_error_deg if decision else 0.0,
                        decision.pitch_error_deg if decision else 0.0,
                    )
                self._last_bucket_output = None

    def _abort_session(self) -> None:
        if not self._state.active:
            return
        logger.info("Aborting active session.")
        self._audio.stop_streaming()
        try:
            self._link.send_session_end()
        except Exception as exc:
            logger.debug("Failed to send session end during abort: %s", exc)
        self._state = SessionState(cadence_sec=self._state.cadence_sec)
        self._last_orientation = None
        self._last_bucket_eval = None
        self._last_bucket_output = None
        self._pending_tare = self._auto_tare

    def _shutdown(self, abort: bool = False) -> None:
        if abort:
            self._abort_session()
        else:
            self._end_session()
        self._send_idle_safe()

    def _format_bucket_debug(self, bucket_label: str) -> str:
        target_vec = self._state.target_vector
        orientation_data = self._last_orientation
        if not target_vec or not orientation_data:
            return f"[Bucket] {bucket_label}"

        aim_vec = orientation_data.get("vector")
        if aim_vec is None:
            return f"[Bucket] {bucket_label}"

        aim_az, aim_el = _vector_to_az_el(aim_vec)
        target_az, target_el = _vector_to_az_el(target_vec)
        delta_az = _wrap_degrees(target_az - aim_az)
        delta_el = target_el - aim_el

        yaw_pitch_hint = ""
        if self._last_bucket_eval:
            yaw_pitch_hint = (
                f" | yaw_err={self._last_bucket_eval.yaw_error_deg:+.1f} "
                f"pitch_err={self._last_bucket_eval.pitch_error_deg:+.1f}"
            )

        return (
            f"[Bucket] {bucket_label} | "
            f"az={aim_az:.1f} el={aim_el:.1f} | "
            f"target az={target_az:.1f} el={target_el:.1f} | "
            f"dAz={delta_az:+.1f} dEl={delta_el:+.1f}"
            f"{yaw_pitch_hint}"
        )

"""Body-centric bucket selection for the spoken guidance prompts.

The host is authoritative for what the participant hears. The firmware still runs
its own ``selectBucket`` and emits a ``BUCKET`` line each tick, but session.py
prefers the label computed here and treats the device label only as a timing tick
(and as a fallback if an ORIENT line was dropped). This file and the firmware's
``selectBucket`` are therefore no longer mirrors -- see TUNABLES.md.

Two algorithms live here:
  * ``BODY_CENTRIC_GUIDANCE = True`` (default): the phase machine below. Errors are
    decomposed the way a human moves -- azimuth error => turn the body (left/right),
    elevation error => raise/lower the arm (up/down) -- with a TURN phase for
    far-off-azimuth targets, a rear latch, and a zenith guard.
  * ``BODY_CENTRIC_GUIDANCE = False``: the legacy yaw/pitch algorithm, kept intact
    for host-only rollback (restart the CLI, no reflash).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

from .constants import (
    ALLOW_DIAGONAL_BUCKETS,
    AZ_TURN_ENTER_DEG,
    AZ_TURN_EXIT_DEG,
    AZ_WEIGHT_FADE_START_DEG,
    BODY_CENTRIC_GUIDANCE,
    BUCKET_LABELS,
    MIN_AZ_WEIGHT,
    NEAR_ENTER_DEG,
    REAR_LATCH_DEG,
    TARGET_BROADCAST_INTERVAL_SEC,
    WANDER_AMPLITUDE_DEG,
    WANDER_ENGAGE_DEG,
    WANDER_MIN_DEG,
    WANDER_PERIOD_SEC,
)

Vector = Tuple[float, float, float]

MIN_DIAGONAL_DEG = 12.0
MICRO_ADJUST_DEG = 1.0
_WORLD_UP: Vector = (0.0, 0.0, 1.0)
_FALLBACK_AXIS: Vector = (1.0, 0.0, 0.0)

# Legacy micro-cycle state (only reachable when BODY_CENTRIC_GUIDANCE is False).
_MICRO_PROMPTS: Tuple[str, ...] = ("up", "right", "down", "left")
_micro_cycle_index: int = 0

PHASE_POINT = "point"
PHASE_TURN = "turn"


@dataclass
class BucketDecision:
    label: str
    # yaw/pitch kept for backward compatibility with session helpers and the legacy
    # path. In the body-centric path yaw_error_deg == d_az_deg and pitch_error_deg
    # == d_el_deg (both signed, + = right / + = up).
    yaw_error_deg: float
    pitch_error_deg: float
    phase: str = PHASE_POINT
    d_az_deg: float = 0.0
    d_el_deg: float = 0.0
    az_weight: float = 1.0


class GuidanceState:
    """Phase-machine + rear-latch state for body-centric guidance.

    One instance per session; ``reset()`` is called from session.py on
    session start / end / abort.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.phase: str = PHASE_POINT
        self.last_turn_sign: int = 0


# Module-level default used when a caller does not supply its own state (keeps the
# old ``bucketize_direction(forward, target)`` call signature working).
_default_state = GuidanceState()


def _dot(a: Vector, b: Vector) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vector, b: Vector) -> Vector:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(vec: Vector) -> float:
    return math.sqrt(_dot(vec, vec))


def _normalize(vec: Vector) -> Optional[Vector]:
    mag = _norm(vec)
    if mag == 0.0:
        return None
    return (vec[0] / mag, vec[1] / mag, vec[2] / mag)


def _clamp(value: float, minimum: float = -1.0, maximum: float = 1.0) -> float:
    return max(minimum, min(maximum, value))


def _az_of(vec: Vector) -> float:
    """World-frame azimuth in degrees, atan2(east, north)."""
    east, north, _up = vec
    return math.degrees(math.atan2(east, north))


def _el_of(vec: Vector) -> float:
    """World-frame elevation in degrees, asin(up)."""
    return math.degrees(math.asin(_clamp(vec[2])))


def _wrap180(delta: float) -> float:
    return ((delta + 180.0) % 360.0) - 180.0


def _az_weight(target_el_deg: float) -> float:
    """Weight applied to |dAz|.

    Facing the target matters fully until it climbs into the zenith band, where a
    tiny physical move swings the azimuth wildly and "which way you face" stops
    mattering; there the weight fades linearly to MIN_AZ_WEIGHT at 90 deg.
    """
    el = abs(target_el_deg)
    if el <= AZ_WEIGHT_FADE_START_DEG:
        return 1.0
    span = 90.0 - AZ_WEIGHT_FADE_START_DEG
    if span <= 0.0:
        return MIN_AZ_WEIGHT
    frac = _clamp((90.0 - el) / span, 0.0, 1.0)
    return max(MIN_AZ_WEIGHT + frac * (1.0 - MIN_AZ_WEIGHT), MIN_AZ_WEIGHT)


def bucketize_direction(
    forward_vec: Vector,
    target_vec: Vector,
    state: Optional[GuidanceState] = None,
) -> BucketDecision:
    """Return the body-relative bucket for the given pointing error."""
    if not BODY_CENTRIC_GUIDANCE:
        return _bucketize_legacy(forward_vec, target_vec)
    if state is None:
        state = _default_state
    return _bucketize_body_centric(forward_vec, target_vec, state)


def _bucketize_body_centric(
    forward_vec: Vector, target_vec: Vector, state: GuidanceState
) -> BucketDecision:
    forward = _normalize(forward_vec)
    target = _normalize(target_vec)
    if forward is None or target is None:
        return BucketDecision(label="up", yaw_error_deg=0.0, pitch_error_deg=0.0)

    tgt_el = _el_of(target)
    d_az = _wrap180(_az_of(target) - _az_of(forward))  # + = clockwise = "right"
    d_el = tgt_el - _el_of(forward)                     # + = "up"

    az_weight = _az_weight(tgt_el)
    d_az_eff = abs(d_az) * az_weight

    # Rear-hemisphere latch: near +-180 the sign of dAz is noise; keep the last turn.
    if abs(d_az) >= REAR_LATCH_DEG and state.last_turn_sign != 0:
        turn_sign = state.last_turn_sign
    else:
        turn_sign = 1 if d_az >= 0.0 else -1

    # Phase machine with hysteresis.
    if state.phase == PHASE_POINT and d_az_eff > AZ_TURN_ENTER_DEG:
        state.phase = PHASE_TURN
    elif state.phase == PHASE_TURN and d_az_eff < AZ_TURN_EXIT_DEG:
        state.phase = PHASE_POINT

    def decision(label: str) -> BucketDecision:
        return BucketDecision(
            label=label,
            yaw_error_deg=d_az,
            pitch_error_deg=d_el,
            phase=state.phase,
            d_az_deg=d_az,
            d_el_deg=d_el,
            az_weight=az_weight,
        )

    if state.phase == PHASE_TURN:
        state.last_turn_sign = turn_sign
        return decision("right" if turn_sign > 0 else "left")

    # POINT phase: weighted dominant axis (diagonals optional, as today).
    if ALLOW_DIAGONAL_BUCKETS and d_az_eff >= MIN_DIAGONAL_DEG and abs(d_el) >= MIN_DIAGONAL_DEG:
        state.last_turn_sign = turn_sign
        vertical = "up" if d_el >= 0.0 else "down"
        horizontal = "right" if turn_sign > 0 else "left"
        return decision(f"{vertical}_{horizontal}")
    if d_az_eff >= abs(d_el):
        state.last_turn_sign = turn_sign
        return decision("right" if turn_sign > 0 else "left")
    return decision("up" if d_el > 0.0 else "down")


def _bucketize_legacy(forward_vec: Vector, target_vec: Vector) -> BucketDecision:
    """Original yaw/pitch algorithm, preserved for host-only rollback."""
    global _micro_cycle_index

    forward = _normalize(forward_vec)
    target = _normalize(target_vec)
    if forward is None or target is None:
        return BucketDecision(label="up", yaw_error_deg=0.0, pitch_error_deg=0.0)

    right_axis = _cross(forward, _WORLD_UP)
    if _norm(right_axis) < 1e-6:
        right_axis = _cross(forward, (0.0, 1.0, 0.0))
        if _norm(right_axis) < 1e-6:
            right_axis = _FALLBACK_AXIS
    right = _normalize(right_axis) or _FALLBACK_AXIS
    up = _normalize(_cross(right, forward)) or _WORLD_UP

    forward_component = _dot(target, forward)
    yaw_error_deg = math.degrees(math.atan2(_dot(target, right), forward_component))
    pitch_error_deg = math.degrees(math.atan2(_dot(target, up), forward_component))

    abs_yaw = abs(yaw_error_deg)
    abs_pitch = abs(pitch_error_deg)

    if abs_yaw < MICRO_ADJUST_DEG and abs_pitch < MICRO_ADJUST_DEG:
        label = _MICRO_PROMPTS[_micro_cycle_index % len(_MICRO_PROMPTS)]
        _micro_cycle_index = (_micro_cycle_index + 1) % len(_MICRO_PROMPTS)
        return BucketDecision(label=label, yaw_error_deg=yaw_error_deg, pitch_error_deg=pitch_error_deg)

    if ALLOW_DIAGONAL_BUCKETS and abs_yaw >= MIN_DIAGONAL_DEG and abs_pitch >= MIN_DIAGONAL_DEG:
        if yaw_error_deg >= 0.0:
            label = "up_right" if pitch_error_deg >= 0.0 else "down_right"
        else:
            label = "up_left" if pitch_error_deg >= 0.0 else "down_left"
        return BucketDecision(label=label, yaw_error_deg=yaw_error_deg, pitch_error_deg=pitch_error_deg)

    if abs_yaw >= abs_pitch:
        label = "right" if yaw_error_deg >= 0.0 else "left"
    else:
        label = "up" if pitch_error_deg >= 0.0 else "down"
    return BucketDecision(label=label, yaw_error_deg=yaw_error_deg, pitch_error_deg=pitch_error_deg)


def decision_error_deg(decision: Optional[BucketDecision]) -> float:
    """Scalar pointing error (deg) used to drive wander engagement and cadence.

    Hypot of the weighted azimuth error and the elevation error, matching the way
    the phase machine ranks the two axes.
    """
    if decision is None:
        return float("inf")
    return math.hypot(abs(decision.d_az_deg) * decision.az_weight, decision.d_el_deg)


# ---------------------------------------------------------------------------
# B1 -- phantom target wander
# ---------------------------------------------------------------------------


def _vec_to_az_el(vec: Vector) -> Tuple[float, float]:
    return _az_of(vec), _el_of(vec)


def _az_el_to_vec(az_deg: float, el_deg: float) -> Vector:
    az = math.radians(az_deg)
    el = math.radians(el_deg)
    cos_el = math.cos(el)
    return (cos_el * math.sin(az), cos_el * math.cos(az), math.sin(el))


def wander_engagement_scale(err_deg: float) -> float:
    """Smoothstep blend: 1.0 below NEAR_ENTER_DEG, 0.0 above WANDER_ENGAGE_DEG."""
    lo, hi = NEAR_ENTER_DEG, WANDER_ENGAGE_DEG
    if err_deg <= lo:
        return 1.0
    if err_deg >= hi:
        return 0.0
    t = (err_deg - lo) / (hi - lo)
    # smoothstep, inverted so err=lo -> 1, err=hi -> 0.
    return 1.0 - (t * t * (3.0 - 2.0 * t))


class TargetWander:
    """Slow orbital drift applied to the broadcast/guidance target.

    The offset is a point on a slowly-orbiting ring: the angle advances at roughly
    one revolution per WANDER_PERIOD_SEC (with light random jitter so it never reads
    as clockwork) while the radius mean-reverts around the middle of the band and is
    clamped into [WANDER_MIN_DEG, WANDER_AMPLITUDE_DEG]. This is the plan's "mean-
    reverting toward a ring": because the offset *rotates* smoothly rather than
    jittering, consecutive honest prompts stay coherent (you are guided around the
    ring -- "right, up, left..." -- instead of contradicting up/down flips), and the
    radius floor means the perturbed target can never coincide with the true target,
    so the piece never resolves. Engaged only once the wearer has gotten close, so
    far-field guidance is exact. See Workstream B1.
    """

    def __init__(self, rng: Optional[random.Random] = None) -> None:
        self._rng = rng if rng is not None else random.Random()
        self.reset()

    def reset(self) -> None:
        self._theta = self._rng.uniform(0.0, 2.0 * math.pi)
        self._radius = 0.5 * (WANDER_MIN_DEG + WANDER_AMPLITUDE_DEG)
        # Random orbit direction so successive sessions differ.
        self._spin = 1.0 if self._rng.random() < 0.5 else -1.0
        self._last_time: Optional[float] = None
        self._engaged = False

    def _advance(self, dt: float) -> None:
        omega = 2.0 * math.pi / WANDER_PERIOD_SEC
        # Angle advances steadily (slow orbit) with mild multiplicative jitter.
        jitter = 1.0 + 0.25 * self._rng.gauss(0.0, 1.0) * math.sqrt(min(dt / WANDER_PERIOD_SEC, 1.0))
        self._theta += self._spin * omega * dt * jitter

        # Radius mean-reverts toward the middle of the band with a little noise,
        # then clamps -- it never reaches zero (WANDER_MIN_DEG floor).
        mid = 0.5 * (WANDER_MIN_DEG + WANDER_AMPLITUDE_DEG)
        k = min(dt / WANDER_PERIOD_SEC, 1.0)
        span = 0.5 * (WANDER_AMPLITUDE_DEG - WANDER_MIN_DEG)
        self._radius += -k * (self._radius - mid) + 0.5 * span * math.sqrt(k) * self._rng.gauss(0.0, 1.0)
        self._radius = _clamp(self._radius, WANDER_MIN_DEG, WANDER_AMPLITUDE_DEG)

    @property
    def _az_off(self) -> float:
        return self._radius * math.cos(self._theta)

    @property
    def _el_off(self) -> float:
        return self._radius * math.sin(self._theta)

    def step(
        self, true_vec: Vector, err_deg: float, now: float
    ) -> Tuple[Vector, Tuple[float, float], float]:
        """Advance the orbit and return (perturbed_vec, (az_off, el_off), scale)."""
        if self._last_time is None:
            dt = TARGET_BROADCAST_INTERVAL_SEC
        else:
            dt = max(0.0, now - self._last_time)
        self._last_time = now
        if dt > 0.0:
            self._advance(dt)

        if err_deg <= NEAR_ENTER_DEG:
            self._engaged = True
        scale = wander_engagement_scale(err_deg) if self._engaged else 0.0

        az, el = _vec_to_az_el(true_vec)
        p_az = az + scale * self._az_off
        p_el = _clamp(el + scale * self._el_off, -89.9, 89.9)
        return _az_el_to_vec(p_az, p_el), (self._az_off, self._el_off), scale


def describe_buckets() -> Iterable[str]:
    yield f"Cardinal prompts: {', '.join(label for label in BUCKET_LABELS if '_' not in label)}."
    if BODY_CENTRIC_GUIDANCE:
        yield (
            f"Body-centric guidance: TURN phase streams left/right when the weighted "
            f"azimuth error exceeds {AZ_TURN_ENTER_DEG:.0f} deg (releasing below "
            f"{AZ_TURN_EXIT_DEG:.0f} deg); POINT phase picks the dominant axis."
        )
    else:
        yield "Legacy yaw/pitch guidance is active (BODY_CENTRIC_GUIDANCE = False)."
    if ALLOW_DIAGONAL_BUCKETS:
        yield f"Diagonal prompts engage when both weighted axes exceed {MIN_DIAGONAL_DEG:.1f} deg."
    else:
        yield "Diagonal prompts are collapsed to the dominant axis for output."

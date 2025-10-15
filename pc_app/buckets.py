"""Body-relative bucket selection heuristics shared with the firmware."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

from .constants import ALLOW_DIAGONAL_BUCKETS, BUCKET_LABELS

Vector = Tuple[float, float, float]

MIN_DIAGONAL_DEG = 12.0
MICRO_ADJUST_DEG = 1.0
_WORLD_UP: Vector = (0.0, 0.0, 1.0)
_FALLBACK_AXIS: Vector = (1.0, 0.0, 0.0)
_MICRO_PROMPTS: Tuple[str, ...] = ("up", "right", "down", "left")
_micro_cycle_index: int = 0


@dataclass
class BucketDecision:
    label: str
    yaw_error_deg: float
    pitch_error_deg: float


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


def bucketize_direction(forward_vec: Vector, target_vec: Vector) -> BucketDecision:
    """Return the body-relative bucket and yaw/pitch error estimates."""
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


def describe_buckets() -> Iterable[str]:
    yield f"Cardinal prompts: {', '.join(label for label in BUCKET_LABELS if '_' not in label)}."
    if ALLOW_DIAGONAL_BUCKETS:
        yield f"Diagonal prompts engage when |yaw| and |pitch| exceed {MIN_DIAGONAL_DEG:.1f}° simultaneously."
    else:
        yield "Diagonal prompts are collapsed to the dominant axis for output."
    yield (
        f"Micro prompts cycle through {', '.join(_MICRO_PROMPTS)} when |yaw| and |pitch| < {MICRO_ADJUST_DEG:.1f}°."
    )

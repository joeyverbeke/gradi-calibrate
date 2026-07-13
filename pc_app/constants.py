"""Shared constants and configuration flags for the desktop app."""

import logging
from pathlib import Path

# Custom log level for audio streaming diagnostics, below DEBUG so normal DEBUG runs
# stay free of the per-clip audio telemetry. Select with --log-level DEBUG-AUDIO.
DEBUG_AUDIO_LEVEL: int = 5
logging.addLevelName(DEBUG_AUDIO_LEVEL, "DEBUG-AUDIO")

# Prompt cadence in seconds while guidance is active.
GUIDANCE_INTERVAL_SEC: float = 1.5

# ---------------------------------------------------------------------------
# Body-centric guidance (Workstream A). The host is now authoritative for the
# spoken prompt; the firmware BUCKET line is only a tick/fallback. See buckets.py.
# ---------------------------------------------------------------------------
# Master switch. False = fall back to the legacy yaw/pitch algorithm in buckets.py
# (kept intact for host-only rollback, no reflash).
BODY_CENTRIC_GUIDANCE: bool = True
# Weighted azimuth error (deg) above which guidance enters the TURN phase
# (stream of turn prompts until the wearer has spun around to face the target).
AZ_TURN_ENTER_DEG: float = 35.0
# Weighted azimuth error (deg) below which TURN releases back to POINT.
# Must be < AZ_TURN_ENTER_DEG (hysteresis, no chatter at the boundary).
AZ_TURN_EXIT_DEG: float = 20.0
# Beyond this |dAz| the sign of the turn is noise; keep the previously issued
# turn direction so left/right does not flip-flop behind the wearer.
REAR_LATCH_DEG: float = 150.0
# Floor on the azimuth weight so azimuth never fully vanishes below the zenith guard.
MIN_AZ_WEIGHT: float = 0.10
# Target elevation (deg) at which the azimuth weight begins fading from 1.0 toward
# MIN_AZ_WEIGHT at 90 deg. Below this elevation azimuth counts fully (facing the
# target still matters); only near the zenith does azimuth stop mattering.
# NOTE (implementer): the plan specified azWeight = cos(el_target), but cos(70)=0.34
# makes the az=-60/el=70 acceptance case return "up" instead of "left" while test 4
# pins the TURN band at 20-35. A zenith-band fade satisfies every behavioral test and
# the "near-overhead -> up" design goal; documented deviation from the literal cos.
AZ_WEIGHT_FADE_START_DEG: float = 70.0

# ---------------------------------------------------------------------------
# Forever-calibration end-state (Workstream B). All host-side; no volume changes.
# ---------------------------------------------------------------------------
# B1 -- phantom target wander (the "never resolves" mechanism).
WANDER_ENABLED: bool = True
# Max offset magnitude (deg) of the slow random walk applied to the broadcast target.
WANDER_AMPLITUDE_DEG: float = 1.8
# Min offset magnitude (deg) once engaged -- the guaranteed unreachable floor.
WANDER_MIN_DEG: float = 0.7
# Drift time constant (sec): slow enough to chase, fast enough to never pin down.
WANDER_PERIOD_SEC: float = 12.0
# Error (deg) below which the wander blends in; above it far-field guidance is exact.
WANDER_ENGAGE_DEG: float = 8.0

# B2 -- cadence stretch (the "feel" of nearness, host-gated speech).
# Cadence sent to firmware in START (device tick / telemetry rate, not speech rate).
DEVICE_TICK_SEC: float = 0.5
# Baseline seconds between spoken prompts (far from target). CLI --cadence maps here.
PROMPT_INTERVAL_SEC: float = 1.5
# Interval multiplier as error -> 0 (voice becomes sparse and calm near the target).
NEAR_CADENCE_SCALE_MAX: float = 2.5
# Near-mode hysteresis band (deg): enter near-mode below ENTER, exit above EXIT.
NEAR_ENTER_DEG: float = 4.0
NEAR_EXIT_DEG: float = 7.0

# Minimum time between idle checks to avoid bouncing in and out of Idle.
IDLE_CHECK_INTERVAL_SEC: float = 0.2

# Directory that contains the intro/outro and bucket audio assets.
ASSETS_BASE_PATH: Path = Path("assets")

# Spoken language defaults for audio assets.
AUDIO_LANGUAGE_DEFAULT: str = "en"
AUDIO_LANGUAGE_CHOICES: tuple[str, ...] = ("en", "kr")
AUDIO_LANGUAGE_DIR_MAP: dict[str, str] = {
    "en": "english",
    "kr": "korean",
}
AUDIO_LANGUAGE_SUFFIX_MAP: dict[str, str] = {
    "en": "_EN",
    "kr": "_KR",
}

# Ship with terminal-only mode enabled; flip to True once audio is validated.
AUDIO_ENABLED_DEFAULT: bool = False

# Bucket labels must match firmware expectations and asset folder names.
BUCKET_LABELS = [
    "up",
    "down",
    "left",
    "right",
    "up_right",
    "up_left",
    "down_right",
    "down_left",
]

# Toggle whether diagonals should surface in host prompts (must match firmware setting).
ALLOW_DIAGONAL_BUCKETS: bool = False

# Serial transport configuration.
USB_BAUD_RATE: int = 921600
SERIAL_PORT_DEFAULT = "/dev/gradi-rp-calibrate"
SERIAL_TIMEOUT_SEC: float = 0.1

# How frequently to resend target vectors to defend against dropped packets.
TARGET_BROADCAST_INTERVAL_SEC: float = 2.0

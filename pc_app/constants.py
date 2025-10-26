"""Shared constants and configuration flags for the desktop app."""

from pathlib import Path

# Prompt cadence in seconds while guidance is active.
GUIDANCE_INTERVAL_SEC: float = 1.5

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
SERIAL_PORT_DEFAULT = "/dev/ttyACM0"
SERIAL_TIMEOUT_SEC: float = 0.1

# How frequently to resend target vectors to defend against dropped packets.
TARGET_BROADCAST_INTERVAL_SEC: float = 2.0

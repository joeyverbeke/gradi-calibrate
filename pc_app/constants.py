"""Shared constants and configuration flags for the desktop app."""

from pathlib import Path

# Prompt cadence in seconds while guidance is active.
GUIDANCE_INTERVAL_SEC: float = 0.5

# Minimum time between idle checks to avoid bouncing in and out of Idle.
IDLE_CHECK_INTERVAL_SEC: float = 0.2

# Directory that contains the intro/outro and bucket audio assets.
ASSETS_BASE_PATH: Path = Path("assets")

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
USB_BAUD_RATE: int = 115200
SERIAL_PORT_DEFAULT = "/dev/ttyACM0"
SERIAL_TIMEOUT_SEC: float = 0.1

# How frequently to resend target vectors to defend against dropped packets.
TARGET_BROADCAST_INTERVAL_SEC: float = 2.0

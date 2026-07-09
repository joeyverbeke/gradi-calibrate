"""Command-line entry point for the desktop companion app."""

from __future__ import annotations

import argparse
import logging

from .audio import AudioPlayer
from .constants import (
    AUDIO_LANGUAGE_CHOICES,
    AUDIO_LANGUAGE_DEFAULT,
    DEBUG_AUDIO_LEVEL,
    GUIDANCE_INTERVAL_SEC,
)
from .link import DeviceLink
from .planets import ObserverLocation
from .session import SessionController


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Gradi Calibration desktop driver")
    parser.add_argument("--port", default=None, help="Serial port connected to the wearable (default: auto)")
    parser.add_argument("--lat", type=float, default=37.7749, help="Observer latitude in degrees")
    parser.add_argument("--lon", type=float, default=-122.4194, help="Observer longitude in degrees (east positive)")
    parser.add_argument(
        "--audio",
        action="store_true",
        help="Enable prompt playback; combine with --local-audio to loop audio locally instead of streaming.",
    )
    parser.add_argument("--cadence", type=float, default=GUIDANCE_INTERVAL_SEC, help="Guidance cadence in seconds")
    parser.add_argument(
        "--log-level",
        default="INFO",
        help="Python logging level (DEBUG-AUDIO, DEBUG, INFO, ...); "
        "DEBUG-AUDIO adds the audio streaming diagnostics that plain DEBUG omits.",
    )
    parser.add_argument(
        "--no-auto-tare",
        action="store_true",
        help="Disable automatic dock tare requests while the wearable is idle.",
    )
    parser.add_argument(
        "--local-audio",
        action="store_true",
        help="Play bucket audio on the desktop (requires --audio); nothing is streamed to the wearable.",
    )
    parser.add_argument(
        "--language",
        default=AUDIO_LANGUAGE_DEFAULT,
        choices=AUDIO_LANGUAGE_CHOICES,
        help="Spoken language for prompts (choices: %(choices)s).",
    )
    parser.add_argument(
        "--audio-gain",
        type=float,
        default=1.0,
        help="Scaling factor applied to streamed audio samples (default 1.0); "
        "values >1.0 clip the normalized assets.",
    )
    return parser


def run_app() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    if args.local_audio and not args.audio:
        parser.error("--local-audio requires --audio")

    level_name = args.log_level.upper().replace("_", "-")
    if level_name == "DEBUG-AUDIO":
        log_level = DEBUG_AUDIO_LEVEL
    else:
        log_level = getattr(logging, level_name, logging.INFO)
    logging.basicConfig(
        level=log_level,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    port = args.port
    link = DeviceLink(port=port) if port else DeviceLink()
    location = ObserverLocation(latitude_deg=args.lat, longitude_deg=args.lon)
    audio = AudioPlayer(
        link=link,
        enabled=args.audio,
        local_only=args.local_audio,
        language=args.language,
        gain=args.audio_gain,
    )
    controller = SessionController(
        link=link,
        audio=audio,
        location=location,
        cadence_sec=args.cadence,
        auto_tare=not args.no_auto_tare,
    )
    controller.run_forever()


if __name__ == "__main__":
    run_app()

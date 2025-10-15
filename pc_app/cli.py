"""Command-line entry point for the desktop companion app."""

from __future__ import annotations

import argparse
import logging

from .audio import AudioPlayer
from .constants import GUIDANCE_INTERVAL_SEC
from .link import DeviceLink
from .planets import ObserverLocation
from .session import SessionController


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Gradi Calibration desktop driver")
    parser.add_argument("--port", default=None, help="Serial port connected to the wearable (default: auto)")
    parser.add_argument("--lat", type=float, default=37.7749, help="Observer latitude in degrees")
    parser.add_argument("--lon", type=float, default=-122.4194, help="Observer longitude in degrees (east positive)")
    parser.add_argument("--audio", action="store_true", help="Enable asset streaming to the wearable")
    parser.add_argument("--cadence", type=float, default=GUIDANCE_INTERVAL_SEC, help="Guidance cadence in seconds")
    parser.add_argument("--log-level", default="INFO", help="Python logging level (DEBUG, INFO, ...)")
    parser.add_argument(
        "--no-auto-tare",
        action="store_true",
        help="Disable automatic tare request on startup (useful if the device cannot be docked).",
    )
    return parser


def run_app() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    port = args.port
    link = DeviceLink(port=port) if port else DeviceLink()
    location = ObserverLocation(latitude_deg=args.lat, longitude_deg=args.lon)
    audio = AudioPlayer(link=link, enabled=args.audio)
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

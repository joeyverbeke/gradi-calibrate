"""Audio asset management and streaming."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

from .constants import ASSETS_BASE_PATH
from .link import DeviceLink

logger = logging.getLogger(__name__)

CHUNK_SIZE = 512


def _find_first_audio_file(directory: Path) -> Optional[Path]:
    if not directory.exists():
        return None
    for ext in ("*.wav", "*.mp3", "*.ogg", "*.flac"):
        candidates = sorted(directory.glob(ext))
        if candidates:
            return candidates[0]
    return None


class AudioPlayer:
    """Loads TTS assets from disk and streams them to the wearable."""

    def __init__(self, link: DeviceLink, assets_root: Path = ASSETS_BASE_PATH, enabled: bool = False) -> None:
        self._link = link
        self._assets_root = assets_root
        self._enabled = enabled
        if enabled:
            logger.info("Audio playback enabled via assets in %s", assets_root)
        else:
            logger.info("Audio playback disabled; running in terminal-only mode.")

    @property
    def enabled(self) -> bool:
        return self._enabled

    def play_intro(self, planet: str) -> None:
        if not self._enabled:
            logger.info("Intro (planet=%s) suppressed in terminal-only mode.", planet)
            return
        directory = self._assets_root / "intro"
        path = _find_first_audio_file(directory)
        if not path:
            logger.warning("No intro audio found in %s", directory)
            return
        self._stream_file(path, label=f"intro_{planet}")

    def play_outro(self) -> None:
        if not self._enabled:
            logger.info("Outro suppressed in terminal-only mode.")
            return
        directory = self._assets_root / "outro"
        path = _find_first_audio_file(directory)
        if not path:
            logger.warning("No outro audio found in %s", directory)
            return
        self._stream_file(path, label="outro")

    def play_bucket(self, bucket: str) -> None:
        if not self._enabled:
            logger.debug("Bucket %s prompt suppressed in terminal-only mode.", bucket)
            return
        directory = self._assets_root / bucket
        path = _find_first_audio_file(directory)
        if not path:
            logger.warning("Missing audio for bucket %s (looked in %s)", bucket, directory)
            return
        self._stream_file(path, label=bucket)

    def _stream_file(self, path: Path, label: str) -> None:
        logger.info("Streaming %s (%s)", label, path.name)
        data = path.read_bytes()
        offset = 0
        if not data:
            logger.warning("Audio file %s is empty.", path)
            return
        while offset < len(data):
            chunk = data[offset : offset + CHUNK_SIZE]
            offset += len(chunk)
            final = offset >= len(data)
            self._link.stream_audio_chunk(label, chunk, final)

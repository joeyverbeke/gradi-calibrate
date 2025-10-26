"""Audio asset management and playback."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

from .constants import (
    ASSETS_BASE_PATH,
    AUDIO_LANGUAGE_DEFAULT,
    AUDIO_LANGUAGE_DIR_MAP,
    AUDIO_LANGUAGE_SUFFIX_MAP,
)
from .link import DeviceLink

logger = logging.getLogger(__name__)

STREAM_FRAMES_PER_CHUNK = 256
_AUDIO_EXTENSIONS = (".wav", ".mp3", ".ogg", ".flac")


def _find_audio_file(directory: Path, base_name: Optional[str] = None) -> Optional[Path]:
    if not directory.exists():
        return None
    if base_name:
        for ext in _AUDIO_EXTENSIONS:
            candidate = directory / f"{base_name}{ext}"
            if candidate.exists():
                return candidate
        return None
    for ext in _AUDIO_EXTENSIONS:
        candidates = sorted(directory.glob(f"*{ext}"))
        if candidates:
            return candidates[0]
    return None


class AudioPlayer:
    """Loads TTS assets from disk and either streams them to the wearable or plays them locally."""

    def __init__(
        self,
        link: DeviceLink,
        assets_root: Path = ASSETS_BASE_PATH,
        enabled: bool = False,
        local_only: bool = False,
        language: str = AUDIO_LANGUAGE_DEFAULT,
    ) -> None:
        self._link = link
        self._assets_root = assets_root
        self._enabled = enabled
        self._local_only = local_only and enabled
        normalized_language = language.lower()
        if normalized_language not in AUDIO_LANGUAGE_DIR_MAP:
            raise ValueError(f"Unsupported language: {language}")
        self._language = normalized_language
        self._language_chain: list[str] = []
        for code in (self._language, AUDIO_LANGUAGE_DEFAULT):
            if code not in self._language_chain:
                self._language_chain.append(code)
        if enabled:
            if self._local_only:
                logger.info(
                    "Audio playback enabled locally via %s assets in %s",
                    self._language,
                    assets_root,
                )
            else:
                logger.info(
                    "Audio playback enabled via %s assets in %s (streaming to device)",
                    self._language,
                    assets_root,
                )
        else:
            logger.info("Audio playback disabled; running in terminal-only mode.")

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def local_only(self) -> bool:
        return self._local_only

    @property
    def should_stream(self) -> bool:
        return self._enabled and not self._local_only

    @property
    def language(self) -> str:
        return self._language

    def play_intro(self, planet: str) -> None:
        if not self._enabled:
            logger.info("Intro (planet=%s) suppressed in terminal-only mode.", planet)
            return
        normalized = (planet or "").strip().lower()
        base_name = f"Intro-{normalized.capitalize()}" if normalized else None
        path, lang_used = self._select_audio_path("intro", base_name=base_name)
        if not path:
            logger.warning("No intro audio found for %s (language=%s).", planet, self._language)
            return
        self._log_language_fallback(lang_used, f"intro:{planet}")
        self._play_or_stream(path, label=f"intro_{planet}")

    def play_outro(self) -> None:
        if not self._enabled:
            logger.info("Outro suppressed in terminal-only mode.")
            return
        path, lang_used = self._select_audio_path("outro", base_name="Outro")
        if not path:
            logger.warning("No outro audio found for language=%s.", self._language)
            return
        self._log_language_fallback(lang_used, "outro")
        self._play_or_stream(path, label="outro")

    def play_bucket(self, bucket: str) -> None:
        if not self._enabled:
            logger.debug("Bucket %s prompt suppressed in terminal-only mode.", bucket)
            return
        path, lang_used = self._select_audio_path(bucket, base_name=None)
        if not path:
            logger.warning("Missing audio for bucket %s (language=%s)", bucket, self._language)
            return
        self._log_language_fallback(lang_used, f"bucket:{bucket}")
        self._play_or_stream(path, label=bucket)

    def _play_or_stream(self, path: Path, label: str) -> None:
        if self._local_only:
            self._play_locally(path, label)
        else:
            self._stream_file(path, label)

    def _stream_file(self, path: Path, label: str) -> None:
        import wave

        stream_started = False
        try:
            with wave.open(str(path), "rb") as wav_handle:
                num_channels = wav_handle.getnchannels()
                sample_width = wav_handle.getsampwidth()
                sample_rate = wav_handle.getframerate()
                frame_count = wav_handle.getnframes()

                if frame_count <= 0:
                    logger.warning("Audio file %s has no frames.", path.name)
                    return
                if num_channels != 1:
                    logger.error(
                        "Only mono WAV files are supported for streaming (%s has %d channels).",
                        path.name,
                        num_channels,
                    )
                    return
                if sample_width != 2:
                    logger.error(
                        "Audio file %s must be 16-bit PCM (sample width=%d bytes).",
                        path.name,
                        sample_width,
                    )
                    return

                logger.info(
                    "Streaming %s (%s @ %d Hz, %d frames)",
                    label,
                    path.name,
                    sample_rate,
                    frame_count,
                )
                self._link.start_audio_stream(sample_rate, frame_count, label)
                stream_started = True
                frames_per_chunk = STREAM_FRAMES_PER_CHUNK
                bytes_per_frame = sample_width * num_channels
                while True:
                    chunk = wav_handle.readframes(frames_per_chunk)
                    if not chunk:
                        break
                    if len(chunk) % bytes_per_frame != 0:
                        # Guard against partial frames introduced by the decoder.
                        valid_len = len(chunk) - (len(chunk) % bytes_per_frame)
                        if valid_len == 0:
                            continue
                        chunk = chunk[:valid_len]
                    self._link.stream_audio_payload(chunk)
        except FileNotFoundError:
            logger.error("Audio file %s not found.", path.name)
        except wave.Error as exc:
            logger.error("Failed to read WAV metadata from %s: %s", path.name, exc)
        except Exception as exc:  # pragma: no cover - serial/hardware dependent
            logger.error("Failed to stream %s: %s", path.name, exc)
        finally:
            if stream_started:
                try:
                    self._link.finish_audio_stream()
                except Exception as exc:  # pragma: no cover - serial/hardware dependent
                    logger.error("Failed to finalize audio stream for %s: %s", label, exc)

    def _play_locally(self, path: Path, label: str) -> None:
        try:
            import simpleaudio  # type: ignore
            import wave
        except ImportError:
            logger.warning("Local audio playback requested but simpleaudio is not installed.")
            return

        if path.suffix.lower() != ".wav":
            logger.warning("Local playback currently supports WAV files only (%s skipped).", path.name)
            return

        try:
            with wave.open(str(path), "rb") as wav_handle:
                num_channels = wav_handle.getnchannels()
                sample_width = wav_handle.getsampwidth()
                sample_rate = wav_handle.getframerate()
                audio_data = wav_handle.readframes(wav_handle.getnframes())
            logger.info("Playing %s locally (%s)", label, path.name)
            play_obj = simpleaudio.play_buffer(
                audio_data,
                num_channels=num_channels,
                bytes_per_sample=sample_width,
                sample_rate=sample_rate,
            )
            play_obj.wait_done()
        except Exception as exc:  # pragma: no cover - hardware/audio dependent
            logger.error("Failed to play %s locally: %s", path, exc)

    def _select_audio_path(self, category: str, base_name: Optional[str]) -> tuple[Optional[Path], Optional[str]]:
        for code in self._language_chain:
            directory = self._assets_root / category / AUDIO_LANGUAGE_DIR_MAP[code]
            suffix = AUDIO_LANGUAGE_SUFFIX_MAP[code]
            candidate_base = f"{base_name}{suffix}" if base_name else None
            path = _find_audio_file(directory, base_name=candidate_base)
            if not path and base_name:
                path = _find_audio_file(directory, base_name=base_name)
            if not path:
                path = _find_audio_file(directory)
            if path:
                return path, code
        return None, None

    def _log_language_fallback(self, lang_used: Optional[str], context: str) -> None:
        if not lang_used or lang_used == self._language:
            return
        logger.info(
            "Falling back to %s assets for %s (requested language=%s).",
            lang_used,
            context,
            self._language,
        )

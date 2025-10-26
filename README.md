# Gradi Calibration

Arm-worn, never-ending pointing guidance that keeps the participant aligning to a randomly selected planet. The desktop Python app computes a target vector, drives session control, and (optionally) streams TTS audio. The RP2040 firmware maintains the motion state machine, quantises pointing error into eight directional buckets, and reports its status back to the host.

## Repository Layout

- `pc_app/`: Desktop companion package (`cli.py`, `session.py`, `planets.py`, `buckets.py`, `audio.py`).
- `gradi_calibration/gradi_calibration.ino`: Wearable firmware for the RP2040 + BNO08x.
- `assets/`: Bucket-specific audio placeholders (`up`, `down`, `left`, `right`, `up_right`, `up_left`, `down_right`, `down_left`, plus `intro`, `outro`) with `english/` and `korean/` subfolders.
- `TUNABLES.md`: Summary of the adjustable constants shared between host and firmware.

## Desktop App

### Setup (uv)

Requires Python 3.10+ available to `uv`.

1. [Install `uv`](https://github.com/astral-sh/uv) if it is not already available.
2. Create and activate a virtual environment:

   ```bash
   uv venv .venv
   source .venv/bin/activate
   ```

3. Install dependencies:

   ```bash
   uv pip install -r requirements.txt
   ```

   (Runtime dependencies include `pyserial` for USB comms and `simpleaudio` for optional local playback; add additional packages to `requirements.txt` as the project grows.)

### Running

```bash
python -m pc_app.cli --audio --port /dev/ttyACM0 --lat 35.1458 --lon 126.9231 --log-level INFO
```

- Omit `--audio` for the terminal-only pass (default).
- Combine `--audio` with `--local-audio` to play prompts on the host (WAV files only; installs `simpleaudio` via `requirements.txt`).
- Leave auto-tare enabled (default) when the device starts docked; pass `--no-auto-tare` if you need to handle the tare manually.
- Adjust `--cadence` to change the prompt interval (seconds).
- Select the spoken language with `--language en` or `--language kr` (defaults to English). If a localized prompt is missing the app falls back to English automatically.
- If you want a different default serial port or cadence, edit `pc_app/constants.py` or use environment-specific CLI overrides.

While running, the CLI prints bucket names reported by the device and streams matching assets when audio mode is enabled. The loop is intended to run indefinitely; press `Ctrl+C` to stop.

## Firmware

Source: `gradi_calibration/gradi_calibration.ino`

1. Open the sketch in the Arduino IDE (or use `arduino-cli`) targeting the Seeed XIAO RP2040.
2. Ensure the Adafruit `BNO08x_RVC` library is installed.
3. Flash the board; the firmware talks to the BNO08x via `Serial1` at 115200 baud and communicates with the host over USB CDC at 921600 baud to sustain PCM streaming.

The firmware removes the legacy LED/haptic game and implements:

- Motion detection (Idle ➜ motion_start ➜ guiding ➜ Idle).
- ENU forward-vector smoothing (`ORIENTATION_SMOOTH_ALPHA`) with a dock-side tare to compensate mounting bias.
- Body-relative guidance that never settles in a “center” state; diagonals can be disabled via `ALLOW_DIAGONAL_BUCKETS` to force pure left/right/up/down prompts.
- Idle transitions check for the dock pose (`DOCK_FORWARD_WORLD` within `DOCK_ALIGNMENT_THRESHOLD_DEG`) and ignore “micro adjustment” stillness when the wearer is already close to the target.
- Serial protocol: `STATE`, `BUCKET`, `ORIENT`, plus `EVENT tared`/`tare_wait` notifications (and `LOG audio_*` helpers while streaming clips).
- Host commands handled today: `TARE`, `START`, `TARGET`, `END`, and `AUDIO START <sample_rate_hz> <frame_count> <label>` / `AUDIO END` for 16-bit mono PCM streaming. After `START` the RP2040 configures I2S (`BCLK=D10`, `LRCLK=D9`, `DIN=D8`, `SD=D6`) and feeds the MAX98357A directly.
- Calibrate for your install:
  - Set `MAG_DECLINATION_DEG` to the local magnetic declination (east-positive; e.g. Seoul ≈ `-8.28f` in Oct 2025).
  - Adjust `DOCK_FORWARD_WORLD` so the docked finger vector matches the real-world heading during tare (`{0,1,0}` = north, `{1,0,0}` = east, etc.).

Adjust the thresholds and cadence defaults in the sketch to tune responsiveness (see `TUNABLES.md`).

## MAX98357A Wiring

| MAX98357A | XIAO RP2040 | Notes |
| --- | --- | --- |
| VIN | 3V3 or 5V | Module tolerates 3–5 V supply (share board power). |
| GND | GND | Common ground with the wearable. |
| BCLK | D10 / P3 | I2S bit clock (firmware constant `PIN_I2S_BCLK`). |
| LRC | D9 / P4 | I2S word select (firmware constant `PIN_I2S_LRCLK`). |
| DIN | D8 / P2 | I2S serial data (firmware constant `PIN_I2S_DATA`). |
| SD | D6 / P0 or 3V3 | Tie high to enable the amp (firmware drives D6 high). |
| OUT+/OUT- | Speaker | Observes the breakout’s differential outputs. |

As long as the asset WAV files are 16-bit mono the host will stream them directly; no additional conversion is required.

## Audio Assets

Every prompt lives under `assets/<category>/<language>/` where `<language>` is `english` (`_EN` suffix) or `korean` (`_KR` suffix). Example:

```
assets/
  intro/
    english/Intro-Mars_EN.wav
    korean/Intro-Mars_KR.wav
  up/
    english/Up-1_EN.wav
    korean/Up-1_KR.wav
  outro/
    english/Outro_EN.wav
    korean/Outro_KR.wav
```

- The `pc_app` streamer requires mono, 16-bit PCM WAV files. Use `tools/mp3_to_wav.py assets/` to batch-convert any MP3 placeholders; pass `--remove-source` if you no longer want to keep the MP3 copies.
- Keep the `_EN` / `_KR` suffixes when adding new clips so the host can resolve the right language variant.
- Local playback (`--local-audio`) shares the same files, so once converted to WAV you can stream to the wearable or play them on the host with no further changes.

## Configuration & Tuning

- Host-side constants live in `pc_app/constants.py`.
- Firmware defaults and thresholds live in `gradi_calibration/gradi_calibration.ino`.
- `TUNABLES.md` provides a single-page reference for the most important knobs (cadence, motion thresholds, smoothing, bucket tolerances).

## Serial Protocol Quick Reference

Host ➜ Device (ASCII lines):

- `START planet=<name> audio=<0|1> cadence=<seconds>`
- `TARGET <east> <north> <up>`
- `END`
- `TARE`
- `AUDIO START <sample_rate_hz> <frame_count> <label>` (followed by raw 16-bit little-endian PCM bytes over USB) and `AUDIO END`

Device ➜ Host:

- `STATE <idle|motion_start|guiding>`
- `BUCKET <label>`
- `ORIENT <east> <north> <up>`
- `EVENT <tared|tare_wait>`
- `LOG audio_*` (diagnostics during playback)

## Next Steps

1. Characterise real-world pointing behaviour and iterate on the micro-adjustment cadence/thresholds.
2. Verify end-to-end audio under load (long-form clips, repeated sessions) and tune buffer sizes if glitches appear.
3. Layer logging/visualisation tooling on the host for session analysis (optional).

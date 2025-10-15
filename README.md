# Gradi Calibration

Arm-worn, never-ending pointing guidance that keeps the participant aligning to a randomly selected planet. The desktop Python app computes a target vector, drives session control, and (optionally) streams TTS audio. The RP2040 firmware maintains the motion state machine, quantises pointing error into eight directional buckets, and reports its status back to the host.

## Repository Layout

- `pc_app/`: Desktop companion package (`cli.py`, `session.py`, `planets.py`, `buckets.py`, `audio.py`).
- `gradi_calibration/gradi_calibration.ino`: Wearable firmware for the RP2040 + BNO08x.
- `assets/`: Bucket-specific audio placeholders (`up`, `down`, `left`, `right`, `up_right`, `up_left`, `down_right`, `down_left`, plus `intro`, `outro`).
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

   (For now the only runtime dependency is `pyserial`; add additional packages to `requirements.txt` as the project grows.)

### Running

```bash
python -m pc_app.cli --port /dev/ttyACM0 --lat 35.1458 --lon 126.9231 --log-level INFO
```

- Omit `--audio` for the terminal-only pass (default). Add `--audio` once the firmware can render streamed audio.
- Leave auto-tare enabled (default) when the device starts docked; pass `--no-auto-tare` if you need to handle the tare manually.
- Adjust `--cadence` to change the prompt interval (seconds).
- If you want a different default serial port or cadence, edit `pc_app/constants.py` or use environment-specific CLI overrides.

While running, the CLI prints bucket names reported by the device and streams matching assets when audio mode is enabled. The loop is intended to run indefinitely; press `Ctrl+C` to stop.

## Firmware

Source: `gradi_calibration/gradi_calibration.ino`

1. Open the sketch in the Arduino IDE (or use `arduino-cli`) targeting the Seeed XIAO RP2040.
2. Ensure the Adafruit `BNO08x_RVC` library is installed.
3. Flash the board; the firmware talks to the BNO08x via `Serial1` at 115200 baud and communicates with the host over USB CDC.

The firmware removes the legacy LED/haptic game and implements:

- Motion detection (Idle ➜ motion_start ➜ guiding ➜ Idle).
- ENU forward-vector smoothing (`ORIENTATION_SMOOTH_ALPHA`) with a dock-side tare to compensate mounting bias.
- Body-relative guidance that never settles in a “center” state; diagonals can be disabled via `ALLOW_DIAGONAL_BUCKETS` to force pure left/right/up/down prompts.
- Idle transitions check for the dock pose (`DOCK_FORWARD_WORLD` within `DOCK_ALIGNMENT_THRESHOLD_DEG`) and ignore “micro adjustment” stillness when the wearer is already close to the target.
- Serial protocol: `STATE`, `BUCKET`, `ORIENT`, plus `EVENT tared`/`tare_wait` notifications.
- Host commands handled today: `TARE`, `START`, `TARGET`, `END`, and stubbed `AUDIO` (audio frames ignored until MAX98357A playback is wired up).
- Calibrate for your install:
  - Set `MAG_DECLINATION_DEG` to the local magnetic declination (east-positive; e.g. Seoul ≈ `-8.28f` in Oct 2025).
  - Adjust `DOCK_FORWARD_WORLD` so the docked finger vector matches the real-world heading during tare (`{0,1,0}` = north, `{1,0,0}` = east, etc.).

Adjust the thresholds and cadence defaults in the sketch to tune responsiveness (see `TUNABLES.md`).

## Audio Assets

Drop pre-generated TTS clips into the directories under `assets/`. The host streams the first audio file it finds per folder (WAV/MP3/OGG/FLAC).

```
assets/
  intro/
  outro/
  up/
  down/
  ...
```

Keep filenames consistent across prompts to maintain predictable playback ordering.

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
- `AUDIO <label> <final_flag> <base64_chunk>` (future use)

Device ➜ Host:

- `STATE <idle|motion_start|guiding>`
- `BUCKET <label>`
- `ORIENT <east> <north> <up>`
- `EVENT <tared|tare_wait>`

## Next Steps

1. Implement MAX98357A audio playback on the wearable to consume the streamed `AUDIO` chunks.
2. Characterise real-world pointing behaviour and iterate on the micro-adjustment cadence/thresholds.
3. Layer logging/visualisation tooling on the host for session analysis (optional).

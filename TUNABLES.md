# Gradi Calibration Tunable Parameters

### Desktop (`pc_app/constants.py`)
- `GUIDANCE_INTERVAL_SEC`: Default prompt cadence used when the firmware does not request a different value.
- `TARGET_BROADCAST_INTERVAL_SEC`: How often the target vector is resent to guard against dropped serial packets.
- `BUCKET_LABELS`: Canonical ordering of feedback buckets; matches asset folder names and firmware expectations.
- `ALLOW_DIAGONAL_BUCKETS`: When false, incoming diagonal prompts are collapsed to the dominant cardinal direction.
- `AUDIO_ENABLED_DEFAULT`: Convenience flag to launch the desktop app in terminal-only or audio-streaming mode.
- `ASSETS_BASE_PATH`: Location of the on-disk audio assets.
- `SERIAL_PORT_DEFAULT`, `USB_BAUD_RATE`, `SERIAL_TIMEOUT_SEC`: Serial transport configuration.

### Firmware (`gradi_calibration/gradi_calibration.ino`)
- `GUIDANCE_INTERVAL_DEFAULT_MS`: Default bucket cadence (milliseconds) used until the host overrides it.
- `ORIENTATION_SMOOTH_ALPHA`: Low-pass filter coefficient for the forward vector.
- `MOTION_START_THRESHOLD_DEG`: Minimum instantaneous angular change that triggers motion-start detection.
- `MOTION_START_DELAY_MS`: Delay (milliseconds) inserted between motion detection and the motion start notification to the host.
- `MOTION_CONTINUE_THRESHOLD_DEG`: Lower threshold that keeps the device in the active state once motion is detected.
- `MOTION_IDLE_TIMEOUT_MS`: Duration of continuous stillness (below the continue threshold) before returning to Idle.
- `MAG_DECLINATION_DEG`: Site-specific declination applied to the BNO085 yaw reading to reference true north.
- `V_ARM_SENS`: Board-axis vector that represents the finger/pointing direction in the sensor frame.
- `DOCK_FORWARD_WORLD`: World-frame vector that the pointing direction should match in the dock pose (used for tare calibration).
- `MIN_DIAGONAL_DEG`: Minimum simultaneous yaw and pitch error (degrees) before diagonal prompts are selected.
- `MICRO_ADJUST_DEG`: Error magnitude threshold that triggers the micro-adjustment bucket cycle instead of declaring the user “centered.”
- `ALLOW_DIAGONAL_BUCKETS`: Mirror of the host flag; when false the firmware emits only cardinal prompts.
- `DOCK_ALIGNMENT_THRESHOLD_DEG`: Angular tolerance (degrees) used to recognise the dock pose when deciding to return to Idle; paired with a dwell to avoid spurious endings.
- `DOCK_ALIGNMENT_DWELL_MS`: How long the device must remain within the dock cone before transitioning to Idle.
- `DOCK_EXIT_THRESHOLD_DEG`: Angular separation from the dock pose required before the device is treated as “removed from the dock” for the next end-of-session detection.
- `DOCK_EXIT_MIN_MS`: Minimum time outside the dock cone before marking that the device has left the dock.
- `NEAR_TARGET_ANGLE_DEG`: Angular error (degrees) treated as “micro adjustments,” preventing an Idle transition while the wearer hovers near the target.

#### Accel-based dock detection (Change 2 — replaces the yaw-referenced dock cone as the active return-to-holder path)
- `USE_ACCEL_DOCK_DETECT`: Master switch. When true the accel-based detector below decides return-to-Idle; when false the firmware falls back to the legacy yaw-referenced dock-cone path (the `DOCK_ALIGNMENT_*` route) for instant reflash rollback.
- `USE_TRANSITION_ARMING`: When true, Idle only arms after a recent placement event (gross motion / put-down); suppresses drift-into-idle during long motionless holds. When false, arming is always satisfied.
- `DOCK_RIGID_SIGMA`: **BENCH-PROVISIONAL (0.04) — NOT the final in-situ value.** Accel-magnitude sigma ceiling (m/s²) below which the device is considered mechanically rigid (docked). Calibrate on real hardware with the installation audio playing: record `accsig` from the `LOG dock` line for (a) racked in holder and (b) worn/held-as-still-as-possible; set the value at the geometric mean of (a) and (b) only if they separate by ≥3×. If they overlap, the software-only path cannot resolve the pose collision and a hall sensor is required.
  - **Home bench measurements (no show audio, device on floor as dock proxy — 2026-07-04):** (a) resting σ ≈ 0.0195 m/s² (range 0.018–0.021); (b) held-still σ ≈ 0.058 (quietest) to ~0.082 (typical), max ~0.21. Separation ≈ 3.0× at the quietest held moment, ~4.1× typical → method validated. Current 0.04 = geometric mean of (a) and (b), for bench transition testing only.
  - **TODO in situ:** re-measure (a) racked-in-real-holder σ with the subwoofer playing (audio raises the racked floor), redo (a)/(b)/(c), and set the final value here.
- `DOCK_RIGID_WINDOW_MS`: Rolling window length over which the accel-magnitude sigma is computed (paired with `DOCK_RVC_HZ` to size the sample buffer). Sigma is only trusted once a full window has accumulated.
- `DOCK_RVC_HZ`: Nominal BNO08x RVC sample rate, used only to size the rolling accel window.
- `DOCK_POSE_TOL_DEG`: Pitch/roll tolerance (degrees) for the gravity-referenced pose match against the holder pose captured at tare. Tune to the holder's insertion repeatability (open question #3).
- `DOCK_CONFIRM_MS`: How long all dock terms (rigid, pose, armed, has-left-dock) must hold continuously before transitioning to Idle.
- `DOCK_ARM_WINDOW_MS`: Look-back window; Idle only arms if a placement event occurred within this many ms. Must exceed the window-fill + confirm time so genuine docking still fires.
- `DOCK_ARM_ACCEL_DELTA`: Per-sample accel-magnitude excursion (m/s²) that counts as a placement/gross-motion event and refreshes the arming timer.
- `DOCK_ARM_ANGLE_DELTA_DEG`: Per-sample forward-vector change (degrees) that also counts as a placement event.
- `DOCK_PITCH_FALLBACK_DEG`, `DOCK_ROLL_FALLBACK_DEG`: **UNVERIFIED placeholders** for the holder pitch/roll, used only before the first tare. The runtime `dockPitch`/`dockRoll` auto-capture the live pitch/roll at tare (the device is docked then). Replace with the measured holder pose (open question #3).

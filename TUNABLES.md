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
- `USE_ACCEL_DOCK_DETECT`: Master switch. When true the accel-based detector below decides return-to-Idle; when false the firmware falls back to the legacy yaw-referenced dock-cone path (the `DOCK_ALIGNMENT_*` route) for instant reflash rollback. The active decision is `hasLeftDock && poseMatch && rigidStill`, sustained for `DOCK_CONFIRM_MS`.
  - **Note — placement-transition arming removed:** an earlier version also required a recent per-sample accel/orientation "jolt" to arm the idle. It was removed because a gentle, deliberate set-down (e.g. lowering the device into the hanging holder arms) produces no jolt, so the detector missed real placements and blocked legitimate docking; it also gave no protection against the held-still collision (rigid stillness carries that). The collision defense now rests entirely on the sigma separation.
- `DOCK_RIGID_SIGMA`: **BENCH-PROVISIONAL (0.04) — NOT the final in-situ value.** Accel-magnitude sigma ceiling (m/s²) below which the device is considered mechanically rigid (docked). This is the sole discriminator between "docked/at rest on a support" and "held in the air," so its in-situ calibration is critical. It also gates **motion-start (donning)**: guidance only starts when the device is NOT rigid, so speaker/subwoofer vibration (which jitters orientation but not accel magnitude) can't false-start a session while the device sits docked. So a mis-set value affects both ending and starting a session. Calibrate on real hardware with the installation audio playing: record `accsig` from the `LOG dock` line for (a) racked in the real hanging arms and (b) worn/held-as-still-as-possible; set the value at the geometric mean of (a) and (b) only if they separate by ≥3×. If they overlap, the software-only path cannot resolve the pose collision and a hall sensor is required.
  - **Home bench measurements (no show audio, device on floor as dock proxy — 2026-07-04):** (a) resting σ ≈ 0.0195 m/s² (range 0.018–0.021); (b) held-still σ ≈ 0.058 (quietest) to ~0.082 (typical), max ~0.21. Separation ≈ 3.0× at the quietest held moment, ~4.1× typical → method validated. Current 0.04 = geometric mean of (a) and (b), for bench transition testing only.
  - **TODO in situ:** re-measure (a) racked-in-real-holder σ with the subwoofer playing (audio raises the racked floor), redo (a)/(b)/(c), and set the final value here.
- `DOCK_RIGID_WINDOW_MS`: Rolling window length over which the accel-magnitude sigma is computed (paired with `DOCK_RVC_HZ` to size the sample buffer). Sigma is only trusted once a full window has accumulated.
- `DOCK_RVC_HZ`: Nominal BNO08x RVC sample rate, used only to size the rolling accel window.
- `DOCK_POSE_TOL_DEG`: Pitch/roll tolerance (degrees) for the gravity-referenced pose match against the holder pose captured at tare. Tune to the holder's insertion repeatability (open question #3).
- `DOCK_CONFIRM_MS`: How long all dock terms (`hasLeftDock`, `poseMatch`, `rigidStill`) must hold continuously before transitioning to Idle. Combined with `DOCK_RIGID_SIGMA`, this is what absorbs a brief sigma dip and prevents a momentary false "rigid" from ending a session.
- `DOCK_ACCEL_MIN_MS2`, `DOCK_ACCEL_MAX_MS2`: Plausibility band (m/s²) for a raw accel-magnitude sample. Gravity is ~9.8; gentle handling stays well inside. A sample outside the band is treated as a corrupt RVC/UART frame and dropped before it enters the sigma window, so a single garbage reading (a ~22 g spike and a dropout-to-zero were both seen in testing) cannot inflate sigma and spuriously break `rigid` for a full window.
- `DOCK_PITCH_FALLBACK_DEG`, `DOCK_ROLL_FALLBACK_DEG`: **UNVERIFIED placeholders** for the holder pitch/roll, used only before the first tare. The runtime `dockPitch`/`dockRoll` auto-capture the live pitch/roll at tare (the device is docked then). Replace with the measured holder pose (open question #3).

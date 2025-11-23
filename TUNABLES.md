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

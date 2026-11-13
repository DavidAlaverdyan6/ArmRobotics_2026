# Current Post Integral navigation (main.py)

The single route entry point is `python3 main.py`. Sensor calibration and
junction pulse setup use `python3 calibrate.py`.

- Sensor-to-axle offset: 70 mm; crossing width: 30 mm; ideal pulse after
  trailing edge: 55 mm. The user tests a fixed time and PWM, then enters
  those values in calibrate.py.
- `junction_drive.json` is stored locally and used by `turn_alignment.py`.
  It is intentionally not committed: calibrations are robot-specific.
- Every 90-degree turn in the route and task returns first uses
  `align_to_crossbar_center()`, which observes the branch and its trailing
  edge on fresh ADC frames, then drives the saved pulse.
- Missing configuration, stale ADC, or an unconfirmed crossing edge stops the
  maneuver; no arbitrary 0.5-second substitute.
- Ordinary line-following uses `forward_line_pid.py`, at 255 PWM base with
  KP=100, KI=0, KD=5, and no acceleration ramp.
- Reverse control, gyro pivots, and special 90-degree cube sensor stop remain
  separate.

**Physical validation still required.** The sensor-bar confirmation, stop
coasting, restart behavior, and final stop mean that 55 mm from rest does not
guarantee that the axle ends at the exact junction center. Test several
left/right pivots and both task returns under real battery and surface
conditions. No firmware motor watchdog is provided here.

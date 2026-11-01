# ArmRobotics navigation: calibration and test plan

`main.py` is the route entry point. `python3 main.py --diagnostic` opens the STM32 and MPU-6050, waits for eight coherent fresh ADC frames, loads line calibration, and **never arms the drive**. Run it first with the chassis lifted and motor power isolated. `python3 -m unittest discover -s tests -v` runs offline with synthetic ADC data.

## Geometry and limits

- Sensor bar to drive axle: 70 mm. S1–S8 span: 60 mm. Main line and crossbar width: 30 mm.
- Wheel diameter: 168 mm; nominal circumference: 527.79 mm. No encoders are installed, so circumference does not measure travel.
- The reported 226 mm is an inner-edge wheel gap. Measure wheel width or centre-to-centre distance before using track width in any turn calculation. The gyro turn needs no assumed track width.
- Minimum observed pivot PWM: approximately 70. The code uses PWM 90–130 for the main turn and one bounded PWM 70 correction.
- Observed 90-degree command results: 95.6425 degrees left and 96.125 degrees right. These are preliminary coast leads, measured under a different PWM profile; verify both directions again.

The sensor sees a 30 mm crossbar before the axle. At the **trailing edge** the sensor is 15 mm beyond the crossbar centre and the axle is still 55 mm behind it. `AXLE_ALIGNMENT` therefore needs a measured time to advance 55 mm, including the commanded stop at the trailing edge, restart acceleration, and final braking. It refuses to turn when that profile is missing. Bar-edge location from 40 ms ADC frames also adds uncertainty; measure the resulting axle placement on the actual track.

## Measure the motion profile

With the robot on a clear straight track and a physical stop switch available, measure PWM 80, 100, 120, and 140 separately where safe. For each PWM, repeat several runs and record distance against commanded run time. Include startup acceleration, the coasting after the first stop, the restart, and coasting after the final stop. Fit the steady part to `distance_mm = forward_mm_s * time_s - launch_deficit_mm + stop_mm`. Here `stop_mm` is the **total additional travel from both stops** in the stop/restart/stop sequence. Use conservative estimates and verify with a 55 mm trial. Write the measured numbers to `motion_calibration.json` next to `main.py`:

```json
{
  "forward_mm_s": {"80": 0, "100": 0, "120": 0, "140": 0},
  "stop_mm": {"80": 0, "100": 0, "120": 0, "140": 0},
  "launch_deficit_mm": {"80": 0, "100": 0, "120": 0, "140": 0}
}
```

The zeros are placeholders, **not measured values**; do not use this example as a calibration file. Until a profile is installed, `main.py` uses one bounded cautious PWM 80 alignment pulse of 0.50 s and reports that its distance is unverified. Install measured values before relying on precise axle placement. Measure turn coast separately for each direction at the new PWM 90–130 profile, then adjust the preliminary leads only after measuring final angle with an external protractor. The gyro angle is a controller estimate, not an independent accuracy measurement.

## Physical trials, in order

1. Lift the drive wheels or isolate motor power. Run `python3 main.py --diagnostic` several times from centred, offset, no-line, and partial-sensor positions. Confirm it never arms motors and exits with a clear failure on missing ADC or gyro.
2. With supervision and a physical power cutoff, test one low-speed initial centring on a wide straight section. Verify steering direction and stop on line loss.
3. Measure the motion profile above. Start with a single marked crossbar and confirm a single route event, bar exit, and axle placement before any turn. Test left, right, T, cross, staggered S1/S8, and reverse crossing separately.
4. Use `python3 turn_90_test.py.py L --execute` or `R --execute` only on an isolated safe area. Record requested angle, gyro result, and independently measured final angle for repeated trials. Abort if the first result differs substantially. Tune left and right leads separately.
5. Test branch lock and bounded recovery at reduced speed; verify failures end in `SAFE_STOP`. Then test B, F, and C individually, including the corrected 220 mm B target, arm coordinates and wrist orientation, flame sensor timeout, and 5-second C stop.
6. Only after those checks run the complete route. Log each junction type, route index, angle and stop reason. The target of 20 consecutive successful runs has **not** been demonstrated.

## Safety limitation

Python checks stop on stale ADC, exceptions, and timeouts while its loop is running. They cannot stop motors if Python itself hangs after the last PWM write. No independent motor-command expiry was found in the local `Telemetrix4Arduino-master` source. A firmware-side STM32 motor watchdog should reset all four motor PWM outputs to zero unless a fresh host heartbeat arrives before a short deadline. That requires separate firmware design, review, and permission; firmware was not changed here.

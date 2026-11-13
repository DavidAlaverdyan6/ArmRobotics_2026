# Tuning the standalone eight-sensor PID line follower

The standalone test is `python differential2.py`. The competition
`main.py` now uses the same `simple_pid` mathematics through
`forward_line_pid.py` at base PWM 255. Both modules have their own gain constants.

## Install

Activate the project's existing Python virtual environment, then:

```bash
python -m pip install "simple-pid>=2,<3"
python -m py_compile differential2.py
python -m unittest discover -s tests
```

To run the standalone test after setting the robot on an unobstructed track:

```bash
python differential2.py
```

Use Ctrl+C to stop. Raise wheels first when testing orientation/polarity.

## Verified architecture

- `robot_pin_config.py` defines eight ADC lines PA0–PA7;
  `line_tracker.py` reverses their order to match the physical array.
- `line_tracker.calculate_position` uses normalized blackness and weights
  `[-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]`.
- **Zero is the center**, negative is left, positive is right.
- The motor driver accepts signed PWM in `[-255,255]`; this standalone
  controller intentionally clamps each wheel to forward/nonnegative PWM.
- The standalone loop runs approximately every 40 ms, while Telemetrix ADC
  callbacks update asynchronously. PID is only updated on a fresh sensor
  snapshot. The library uses actual elapsed time (`sample_time=None`).
- On missing line, the motors stop and the PID accumulator resets.
- The wheel center-to-center distance is **not** established by the
  documented 226 mm inner-wheel gap; don't substitute it as track width.

## Gains and correction

```
error = line_position - 0.0
steering_pwm = -PID(line_position)  # simple-pid uses setpoint minus measurement
left_pwm  = clamp(base_pwm + steering_pwm, 0, max_pwm)
right_pwm = clamp(base_pwm - steering_pwm, 0, max_pwm)
```

Start with the **experimental defaults** in `differential2.py`:
`BASE_SPEED=100`, `KP=30`, `KI=0.5`, `KD=0.4`,
`MAX_CORRECTION=60`. These values are **not calibrated gains**.

### Step-by-step tuning

1. **Verify sensor signs.** With the wheels raised, shift the black line
   toward the robot's right. `pos` should become positive and the left
   motor should be commanded faster than the right. If this fails, investigate
   wiring/order; don't compensate by blindly changing PID gains.
2. **Check calibration and ADC freshness.** Run `python calibrate.py` if
   necessary. Eight channels must report before motors can move.
3. **Tune P only at low speed.** Set `KI=0`, `KD=0`, and use
   `BASE_SPEED=80` or `100` in a safe test lane. Increase `KP` from
   around 10 in small steps until it corrects reliably. When persistent
   left-right oscillation appears, back `KP` down by roughly 20–30%.
   These are starting procedures, not guarantees.
4. **Add D.** Keeping `KI=0`, gradually increase `KD` (for example,
   increments of 0.1). Seek reduced overshoot without visible chatter. If
   PWM flips rapidly on noisy ADC readings, lower D and inspect sampling.
   `KD` is in PWM·seconds/position-unit, not the same scale as older
   frame-to-frame derivative calculations.
5. **Add I last.** Only for a repeatable constant-side bias, start around
   `KI=0.1`, then increase cautiously. If the robot recovers too slowly
   after a long curve, or starts to weave, reduce I. The library's limited
   output mitigates integral windup and stopping resets integral memory.
6. **Raise speed slowly.** Try increments of 10–20 PWM while repeating
   curves and junction approaches. Tune at the actual operating speed;
   don't assume settings at PWM 100 remain stable at 180–200.
7. **Check debug terms.** `P/I/D` in terminal output are the library's
   un-inverted terms; the printed `correction` is what is applied to the
   motors (opposite sign). Watch for sustained correction near ±60,
   which signals saturation.

## Troubleshooting

| Symptom | First adjustment/check |
| --- | --- |
| Slow to re-center | Increase P modestly |
| Oscillates around line | Reduce P or increase D cautiously |
| Twitchy commands | Reduce D; check ADC frame age |
| Consistent offset | Check alignment first, then consider small I |
| No line seen | Verify calibration, contrast, and sensor wiring |
| Sharp curves require reverse wheel | The standalone version intentionally forbids reverse PWM; reduce base speed or review this separately |

**Important:** Standalone `differential2.py` is a controller test, not the
competition route runner. It does not execute junction routes, gyro pivots,
arm/button tasks, or reverse line tracking. No hardware test has been
performed as part of these commits.

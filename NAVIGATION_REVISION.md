# Navigation calibration and revised tasks

Wheel diameter is 67 mm (circumference approximately 210.49 mm).
55 mm is approximately 0.2613 revolutions. No encoders measure that rotation.

## Measure the 55 mm pulse

Run from the repository directory. These commands actually move the chassis;
mark the axle position and measure final travel after coasting stops.
Repeat each measurement at the same battery charge and surface conditions.

```bash
./venv/bin/python measure_55mm.py --pwm 80 --run 0.4 --execute
./venv/bin/python measure_55mm.py --pwm 80 --run 0.8 --execute
```

Use the two measured distances, in mm, instead of the placeholders below:

```text
./venv/bin/python measure_55mm.py --pwm 80 --first 0.4 DISTANCE1 --second 0.8 DISTANCE2 --save
```

The script fits distance = speed × commanded time + start/stop offset.
It prints the computed time and saves motion_calibration.json. Verify a run
at that computed time measures 55 mm before running the route. Short pulses
are sensitive to battery condition and motor starting friction; repeat trials.
The approach to the junction adds braking travel before the pulse. Measure
the complete axle placement relative to the actual crossbar too; the pulse
fit alone cannot independently establish alignment accuracy at the junction.

Without a valid profile the new L90/R90 refuses the alignment manoeuvre.
There is no unmeasured 0.5-second fallback in the new common turn.

## Behaviour

Route L90/R90 confirms the selected side branch has ended, then runs the
measured straight PWM 80 pulse. After 60 degrees it watches S1 for L90 and
S8 for R90, slows further after that side sees black, and stops when four
centre sensors confirm the new line in three fresh frames. At 90 degrees
without confirmation it returns through the measured angle and invokes the
ordinary L/R turn. These are gyro estimates, not externally measured angles.
Return tasks enter at the already detected junction edge.

BUTTON calls L90, follows for three seconds without laser distance, reverses
until the junction, then calls R90. Return time and line-loss limits are
disabled for this task. ADC and gyro hardware checks still apply.

CUBE1 calls R90, opens the gripper and poses at (250,0,50) with a free wrist,
then (180,0,-180) at -90 degrees. Cube detection lights solid red, arrival at
the next junction lights green, and the subsequent drop ends transport.
Loss before that junction enters fault mode: slow red blink, then fast red
blink past the junction. After clearing that crossbar, three fresh all-black
frames trigger return. Missing cube detection uses the same pit fallback.

FIRE approach and junction-return elapsed-time limits are removed. A missing
line after its initial turn invokes reverse line search. Charging LED errors
and COLOR camera/LED/arm errors allow route continuation. Camera recovery has
a ten-second retry budget; a driver call that blocks internally cannot be
preempted by this Python deadline.

Line following uses Kp=80, Kd=2 and initial Ki=0.5. The integral is bounded,
reset on stop/line loss/junction, and held when wheel output saturates.
Ki still needs physical tuning; the initial integral correction is at most
one PWM unit. Reverse geometry controller retains its separate P/D gains.

Startup calibration checks, stale ADC checks, gyro validity/no-motion checks,
and final motor shutdown remain. No firmware watchdog is added here.

Offline revision checks:

```bash
PYTHONPATH=.:tests ./venv/bin/python -m unittest tests.test_navigation_revision tests.test_line_following tests.test_navigation_core tests.test_reverse_line_controller -q
```

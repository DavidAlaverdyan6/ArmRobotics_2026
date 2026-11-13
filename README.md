# ArmRobotics 2026

# Setup guide:

  1. Clone the repo
  2. Create a virtual environment: `python -m venv venv`
  3. Activate it: `source venv/bin/activate`

## Main robot program

Robot navigation and tasks run from one program:

    source venv/bin/activate
    python main.py

At startup the program hardware-resets the STM32 before opening Telemetrix.
Connect Raspberry Pi BCM23 (physical pin 16) through a 1 kOhm series resistor
to the STM32 pad marked `R` or `NRST`, and connect a Raspberry Pi ground pin to
STM32 ground. The reset code only pulls NRST low and then releases it; it never
drives the reset line high.

Edit `ROUTE` near the top of `main.py` before launch. Movement commands
remain `L`, `R`, `S`, `L90`, and `R90`. Named tasks are `BUTTON`, `CHARGING`,
`FIREFIGHTER`, `CUBE1`, and `COLOR`.

## Calibrate the line and 90-degree positioning

Run `python3 calibrate.py`. After WHITE, GRAY and BLACK sampling, enter
your measured forward **time in seconds** and **motor PWM** for the pulse that
starts only after the trailing edge of a junction. They are stored locally in
`junction_drive.json`, separate from the line sensor calibration.

The route starts with `python3 main.py`. The L90/R90 routines, including
their use inside BUTTON/CUBE1/FIREFIGHTER/COLOR, read this saved pulse.
If the configuration is absent or invalid, alignment stops instead of guessing.
A separate 55 mm test remains available as `python3 measure_55mm.py`.
Before competition, verify axle positioning including coasting at the edge.

## Color marker

Add `"COLOR"` to `ROUTE` to stop at a junction and run the camera detector.
It uses the channel thresholds from Daniel's archived detector, draws a box
around each red, yellow, green, or blue object, and selects the largest marker
after three stable frames. The matching color is shown through `led.py` for
two seconds. Detection is implemented separately in
`color_marker_detector.py`; `main.py` launches the integrated route program.

## CUBE1

`CUBE1` does not use a camera. It moves the open gripper tip to
`X=360, Y=0, Z=-150` mm with a fixed final pitch of `-20°`, then follows the
line slowly while reading the same PB0 optical sensor used by `FIREFIGHTER`.
Three active readings confirm the cube. The robot stops for 0.5 second and
then resumes slow line following.

Transport ends on the first of two confirmed events: three inactive cube
sensor readings, or the next junction is detected. At a junction, the robot
continues forward for 0.5 second using the same approach as an `R90` command,
then folds the arm into the standard home pose `(90, 170, 180, 0, 170)` and
performs a final `R90`. If cube loss ends transport first, the original 80°
right pivot is retained. The PB0 threshold and polarity remain shared with
`FIREFIGHTER` in `firefighter.py`.

During `FIREFIGHTER`, losing the line before a flame is confirmed immediately
stops the approach and cancels spraying. The robot reverses slowly until the
line is confirmed again, folds the arm, and then uses the same line-following
return and final right turn as the button task. Recovery is bounded by a
three-second timeout; failure leaves the chassis stopped.

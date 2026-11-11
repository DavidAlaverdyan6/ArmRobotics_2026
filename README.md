# ArmRobotics 2026

# Setup guide:

  1. Clone the repo
  2. Create a virtual environment: `python -m venv venv`
  3. Activate it: `source venv/bin/activate`

## Main robot program

Robot navigation and tasks run from one program:

    source venv/bin/activate
    python main_test.py

At startup the program hardware-resets the STM32 before opening Telemetrix.
Connect Raspberry Pi BCM23 (physical pin 16) through a 1 kOhm series resistor
to the STM32 pad marked `R` or `NRST`, and connect a Raspberry Pi ground pin to
STM32 ground. The reset code only pulls NRST low and then releases it; it never
drives the reset line high.

Edit `ROUTE` near the top of `main_test.py` before launch. Movement commands
remain `L`, `R`, `S`, `L90`, and `R90`. Named tasks are `BUTTON`, `CHARGING`,
`FIREFIGHTER`, `CUBE1`, and `COLOR`.

## Color marker

Add `"COLOR"` to `ROUTE` to stop at a junction and run the camera detector.
It uses the channel thresholds from Daniel's archived detector, draws a box
around each red, yellow, green, or blue object, and selects the largest marker
after three stable frames. The matching color is shown through `led.py` for
two seconds. Detection is implemented separately in
`color_marker_detector.py`; `main_test.py` launches the integrated route program.

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

# ArmRobotics 2026

# Setup guide:

  1. Clone the repo
  2. Create a virtual environment: `python -m venv venv`
  3. Activate it: `source venv/bin/activate`

## Main robot program

Robot navigation and tasks run from one program:

    source venv/bin/activate
    python main_test.py

Edit `ROUTE` near the top of `main_test.py` before launch. Movement commands
remain `L`, `R`, `S`, `L90`, and `R90`. Named tasks are `BUTTON`, `CHARGING`,
`FIREFIGHTER`, and `CUBE1`.

## CUBE1

`CUBE1` does not use a camera. It moves the gripper tip to
`X=350, Y=0, Z=-120` mm with a fixed final pitch of `0°`, then fully opens the
gripper. The chassis performs the same controlled `R90` sequence as the route
command, waits one second after positioning the arm, then continues straight
through the next junction. The first line loss after that junction is the
course marker for the cube drop point. `CUBE1` does not initialize or read the
fire sensor and never touches the pump output.

After the loss is confirmed, the chassis reverses along the line with gyro
heading correction to the junction. Before reversing, the arm is smoothly
folded into the standard home pose `(90, 170, 180, 0, 180)`, matching the
working `BUTTON` and `FIREFIGHTER` return conditions. The chassis then moves
forward briefly, performs the same 80° right pivot used by `BUTTON`, reacquires
the line, and returns control to the route. Contact/drop phases have bounded
timeouts so a missing sensor transition cannot make the robot drive forever.

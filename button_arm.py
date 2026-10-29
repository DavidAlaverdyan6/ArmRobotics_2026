#!/usr/bin/env python3
"""Arm prototype: move to terminal-entered XYZ and return to arm_web home.

Uses the robot's EXISTING Telemetrix board, never opens /dev/serial0.
XYZ is the gripper tip in MILLIMETRES relative to the arm's model origin:
X forward, Y right, Z up (as in arm_web.py).
"""

import math
import time

BUTTON_HOLD_SECONDS = 0.3


def _execute_plan(arm, path, delay):
    for angles in path:
        arm.write(**angles)
        time.sleep(delay)


def _return_home(arm, home, max_step_deg, delay):
    """Smoothly interpolate servos back to arm_web.START_ANGLES."""
    joints = ("base", "shoulder", "elbow", "wrist")
    current = {
        name: float(getattr(arm, name).angle)
        for name in joints
    }

    steps = max(
        1,
        math.ceil(
            max(abs(float(home[name]) - current[name]) for name in joints)
            / max_step_deg
        ),
    )

    for index in range(1, steps + 1):
        t = index / steps
        smooth = t * t * (3 - 2 * t)
        angles = {
            name: round(
                current[name] + (float(home[name]) - current[name]) * smooth,
                2,
            )
            for name in joints
        }
        arm.write(**angles)
        time.sleep(delay)

    # The gripper stays in its initial closed position throughout the move.
    # Set its original angle again on return to home.
    arm.write(gripper=home["gripper"])


def move_to_xyz_and_home(board, x, y, z):
    """Move the gripper tip to user-entered XYZ, hold, then return home.

    The controller has no joint or button feedback. 'Complete' means servo
    commands were sent; it cannot verify actual contact or physical position.
    """
    x, y, z = (float(x), float(y), float(z))
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError("XYZ coordinates must be finite numbers.")

    # Import the tested IK planner without starting arm_web's server or
    # opening a second serial connection (its main() is guarded).
    import arm_web
    from roboarm import RoboArm

    arm = RoboArm(board)
    home = dict(arm_web.START_ANGLES)

    # arm_web.plan_auto_path reads the active arm's current servo angles.
    arm_web.arm = arm
    arm.write(**home)
    time.sleep(0.6)

    target = {"x": x, "y": y, "z": z}

    # The full plan is solved before moving away from home.
    # If the point cannot be reached, no approach motion is performed.
    path, pitch = arm_web.plan_auto_path(target)
    print(
        f"ARM: planned XYZ=({x:g}, {y:g}, {z:g}) mm, "
        f"tool pitch={pitch:.1f} degrees"
    )

    try:
        _execute_plan(arm, path, arm_web.STEP_DELAY)
        print("ARM: target commanded. Holding.")
        time.sleep(BUTTON_HOLD_SECONDS)

        print("ARM: returning to home...")
        _return_home(
            arm,
            home,
            arm_web.AUTO_MAX_STEP_DEG,
            arm_web.STEP_DELAY,
        )
        print("ARM: home commanded.")
        return True

    except Exception:
        # Do NOT reverse the mobile base if the arm may be left extended.
        raise

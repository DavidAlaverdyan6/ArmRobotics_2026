#!/usr/bin/env python3
"""Optional button-press maneuver using the existing arm_web IK planner.

This module NEVER opens a second Telemetrix connection. The robot's existing
board is passed to press_button(). Configure and physically verify the
button tip XYZ and arm home pose before enabling motion.
"""

import math
import time

# Millimetres from the shoulder/base model origin, not from VL53L0X.
# X forward, Y right, Z up. Set to a tested (x, y, z) after measuring it.
BUTTON_TARGET_XYZ_MM = None

# The button face is approached from 20 mm behind, along X.
BUTTON_APPROACH_OFFSET_MM = 20.0
BUTTON_HOLD_SECONDS = 0.3

# Must be verified against the ACTUAL installed servo neutral pose first.
HOME_ANGLES_CONFIRMED = False


def _execute_plan(arm, path, delay):
    for angles in path:
        arm.write(**angles)
        time.sleep(delay)


def _return_home(arm, home, max_step_deg, delay):
    """Move back to the selected servo home pose with small smooth angle steps."""
    current = {
        name: float(getattr(arm, name).angle)
        for name in ("base", "shoulder", "elbow", "wrist")
    }
    joints = ("base", "shoulder", "elbow", "wrist")
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


def press_button(board):
    """Move to the measured button tip position, press, retract and home.

    Expects the robot to be fully stopped. Returns True only if the sequence
    completes. NO contact sensor is available, so physical press success
    cannot be independently verified.
    """
    if BUTTON_TARGET_XYZ_MM is None:
        raise ValueError("Button XYZ is not configured.")
    if not HOME_ANGLES_CONFIRMED:
        raise ValueError("Arm home pose has not been verified.")

    xyz = tuple(float(x) for x in BUTTON_TARGET_XYZ_MM)
    if len(xyz) != 3 or not all(math.isfinite(x) for x in xyz):
        raise ValueError("Button XYZ must contain three finite millimetre values.")

    x, y, z = xyz
    if x <= BUTTON_APPROACH_OFFSET_MM:
        raise ValueError("Button X must exceed the approach offset.")

    # Import only when arm actuation is requested, not at robot startup.
    # Importing arm_web doesn't run its web server (guarded by __main__).
    import arm_web
    from roboarm import RoboArm

    arm = RoboArm(board)
    home = dict(arm_web.START_ANGLES)

    # The planner assumes arm_web.arm points to the actual arm object.
    # No second board/serial session is created.
    arm_web.arm = arm
    arm.write(**home)
    time.sleep(0.6)

    approach = {"x": x - BUTTON_APPROACH_OFFSET_MM, "y": y, "z": z}
    contact = {"x": x, "y": y, "z": z}

    try:
        path, pitch = arm_web.plan_auto_path(approach)
        print(f"ARM: approach {approach}, pitch {pitch:.1f} degrees")
        _execute_plan(arm, path, arm_web.STEP_DELAY)

        path, pitch = arm_web.plan_auto_path(contact)
        print(f"ARM: press {contact}, pitch {pitch:.1f} degrees")
        _execute_plan(arm, path, arm_web.STEP_DELAY)
        time.sleep(BUTTON_HOLD_SECONDS)

        path, pitch = arm_web.plan_auto_path(approach)
        print(f"ARM: retract {approach}")
        _execute_plan(arm, path, arm_web.STEP_DELAY)

        _return_home(
            arm, home, arm_web.AUTO_MAX_STEP_DEG, arm_web.STEP_DELAY
        )
        print("ARM: returned to home position.")
        return True

    except Exception:
        # Do not automatically attempt unknown recovery through an obstacle.
        # The robot should stay stopped for manual inspection.
        raise

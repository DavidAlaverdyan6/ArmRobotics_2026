#!/usr/bin/env python3
"""Two-stage ArmRobotics button movement using the existing arm_web IK.

The robot owns the Telemetrix board. Do not open another serial connection.
XYZ is the gripper tip (millimetres): X forward, Y right, Z up.
"""

import math
import time

BUTTON_HOLD_SECONDS = 0.3

# Home is about (134, 0, 113) mm in arm_ik's current model. At X=134,
# Z=-40 is not reachable in that model. X=160 permits lowering to Z=-40
# with only a small initial X displacement before the long X extension.
LOWERING_X_MM = 160.0


def _execute_plan(arm, path, delay):
    for angles in path:
        arm.write(**angles)
        time.sleep(delay)


def _return_home(arm, home, max_step_deg, delay):
    """Smoothly return servos to the arm_web-defined home configuration."""
    joints = ("base", "shoulder", "elbow", "wrist")
    current = {
        name: float(getattr(arm, name).angle) for name in joints
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
        smooth = t*t*(3 - 2*t)
        angles = {
            name: round(
                current[name] + (float(home[name])-current[name])*smooth, 2
            )
            for name in joints
        }
        arm.write(**angles)
        time.sleep(delay)

    arm.write(gripper=home["gripper"])


def move_to_xyz_and_home(board, x, y, z):
    """Lower to Z first, then advance along X at constant Z and return home.

    Movement phase 1: lower toward (160, y, z).
    Movement phase 2: move the tip along X to (x, y, z), holding Z fixed.
    No servo or contact feedback is available; the completed state means
    all servo commands were issued, not that contact was physically measured.
    """
    x, y, z = map(float, (x, y, z))
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError("XYZ coordinates must be finite.")

    if x <= LOWERING_X_MM:
        raise ValueError(
            f"Target X must exceed first-stage X={LOWERING_X_MM:g} mm."
        )

    import arm_web
    from roboarm import RoboArm

    arm = RoboArm(board)
    home = dict(arm_web.START_ANGLES)
    arm_web.arm = arm
    arm.write(**home)
    time.sleep(0.6)

    first_target = {"x": LOWERING_X_MM, "y": y, "z": z}
    final_target = {"x": x, "y": y, "z": z}

    # Preflight both segments before moving the arm away from its home pose.
    descent_path, descent_pitch = arm_web.plan_auto_path(first_target)

    # The Cartesian planner uses arm_web.arm's recorded current angles.
    # Temporarily preview phase 1's end angles without sending servo commands.
    joints = ("base", "shoulder", "elbow", "wrist")
    saved_angles = {
        name: getattr(arm, name).angle for name in joints
    }
    try:
        for name, angle in descent_path[-1].items():
            getattr(arm, name).angle = float(angle)

        # Keeping tool pitch constant makes this a genuine X-only motion
        # in the IK model, with Y and Z fixed at the lowered coordinates.
        extension_path = arm_web.plan_cartesian_path({
            **final_target,
            "tool_pitch": float(descent_pitch),
        })
    finally:
        for name, angle in saved_angles.items():
            getattr(arm, name).angle = angle

    print(
        f"ARM phase 1: lower to X={LOWERING_X_MM:g}, Y={y:g}, "
        f"Z={z:g} mm (pitch {descent_pitch:.1f} degrees)."
    )
    print(
        f"ARM phase 2: extend X from {LOWERING_X_MM:g} to {x:g} mm "
        f"while keeping Y={y:g}, Z={z:g} mm."
    )

    try:
        _execute_plan(arm, descent_path, arm_web.STEP_DELAY)
        _execute_plan(arm, extension_path, arm_web.STEP_DELAY)
        print("ARM: button target commanded. Holding...")
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
        # Stop the mobile base rather than reversing with an extended arm.
        raise

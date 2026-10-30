#!/usr/bin/env python3
"""Two-stage ArmRobotics button movement using the existing arm_web IK.

The robot owns the Telemetrix board. Do not open another serial connection.
XYZ is the gripper tip (millimetres): X forward, Y right, Z up.
"""

import math
import time

BUTTON_HOLD_SECONDS = 0.3

BUTTON_START_X_MM = 250.0
BUTTON_START_Z_MM = 100.0
BUTTON_TOOL_PITCH = 0.0


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


def plan_button_paths(arm_web, x, y, z):
    """Preflight every segment with horizontal tool pitch before moving.

    Extend at Z=100 before descending: a direct diagonal from (250,0,100)
    to (350,0,-40) exceeds the elbow servo limit in the current model.
    """
    targets = [
        (BUTTON_START_X_MM, 0.0, BUTTON_START_Z_MM),
        (x, y, BUTTON_START_Z_MM),
        (x, y, z),
    ]
    arm = arm_web.arm
    saved = {n: getattr(arm, n).angle for n in arm_web.IK_JOINTS}
    paths = []
    try:
        for tx, ty, tz in targets:
            path = arm_web.plan_locked_pitch_path({
                "x": tx, "y": ty, "z": tz, "tool_pitch": BUTTON_TOOL_PITCH,
            })
            paths.append(path)
            for name, angle in path[-1].items():
                getattr(arm, name).angle = float(angle)
        return paths
    finally:
        for name, angle in saved.items():
            getattr(arm, name).angle = angle


def move_to_xyz_and_home(board, x, y, z):
    """Horizontal tool: staging point, forward extension, descent, then home."""
    x, y, z = map(float, (x, y, z))
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError("XYZ coordinates must be finite.")

    import arm_web
    from roboarm import RoboArm

    arm = RoboArm(board)
    home = dict(arm_web.START_ANGLES)
    arm_web.arm = arm
    arm.write(**home)
    time.sleep(0.6)
    paths = plan_button_paths(arm_web, x, y, z)

    print("ARM: horizontal tool, via (250, 0, 100) and forward at Z=100.")
    for path in paths:
        _execute_plan(arm, path, arm_web.STEP_DELAY)
    print(f"ARM: button target ({x:g}, {y:g}, {z:g}) commanded. Holding...")
    time.sleep(BUTTON_HOLD_SECONDS)

    # Retrace the horizontal-tool descent/extension before folding home.
    # Include preceding endpoints so short segments are fully retraced.
    for index in (2, 1):
        _execute_plan(arm, list(reversed(paths[index])) + [paths[index-1][-1]],
                      arm_web.STEP_DELAY)
    _return_home(arm, home, arm_web.AUTO_MAX_STEP_DEG, arm_web.STEP_DELAY)
    print("ARM: home commanded.")
    return True

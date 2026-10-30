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
# On the button press, lower Z early instead of driving X forward first.
# Wrist compensation keeps the tool horizontal throughout the joint-space path.
BUTTON_SHOULDER_PROGRESS_POWER = 0.65
BUTTON_ELBOW_PROGRESS_POWER = 1.8


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



def _plan_early_z_press_path(arm_web, target):
    """Lower the tip earlier on the direct staging-to-button transition.

    Use unequal smooth progress for shoulder and elbow. This makes Z descend
    sooner while X advances more gradually, without introducing a separate
    stop/waypoint. Compensate at the wrist to keep the existing tool pitch.
    All servo poses and the maximum angular step are checked before moving.
    """
    from arm_ik import WRIST_DEGREES_PER_SERVO

    current = arm_web.get_current_angles()
    # Let the tested arm_web IK choose a feasible final pose, as before.
    final = arm_web.plan_target_pitch_path(target)[-1]
    joints = arm_web.IK_JOINTS
    max_step = arm_web.AUTO_MAX_STEP_DEG
    steps = max(
        arm_web.MOVE_STEPS,
        math.ceil(max(abs(final[n] - current[n]) for n in joints) / max_step),
    )

    while steps <= 320:
        path = []
        previous = current
        within_step_limit = True
        for index in range(1, steps + 1):
            t = index / steps
            smooth = t * t * (3.0 - 2.0 * t)
            shoulder_t = smooth ** BUTTON_SHOULDER_PROGRESS_POWER
            elbow_t = smooth ** BUTTON_ELBOW_PROGRESS_POWER

            shoulder = current["shoulder"] + (
                final["shoulder"] - current["shoulder"]
            ) * shoulder_t
            elbow = current["elbow"] + (
                final["elbow"] - current["elbow"]
            ) * elbow_t
            # tool_pitch = shoulder - elbow + calibrated wrist contribution.
            wrist = current["wrist"] + (
                (elbow - current["elbow"]) -
                (shoulder - current["shoulder"])
            ) / WRIST_DEGREES_PER_SERVO
            pose = {
                "base": current["base"] + (
                    final["base"] - current["base"]
                ) * smooth,
                "shoulder": shoulder,
                "elbow": elbow,
                "wrist": wrist,
            }
            if index == steps:
                pose = dict(final)

            if any(not 0.0 <= pose[n] <= 180.0 for n in joints):
                raise arm_web.IKError(
                    "Button press would exceed a servo limit; arm not moved."
                )
            if max(abs(pose[n] - previous[n]) for n in joints) > max_step:
                within_step_limit = False
                break
            path.append({n: round(pose[n], 2) for n in joints})
            previous = pose

        if within_step_limit:
            return path
        steps *= 2

    raise arm_web.IKError(
        "Cannot plan the early-Z button press within safe servo step limits."
    )


def plan_button_paths(arm_web, x, y, z):
    """Preflight staging and direct early-Z button press at horizontal pitch.

    The second stage is continuous joint-space motion (not a straight XYZ
    line). The wrist is compensated to keep the tip horizontal.
    """
    targets = [
        (BUTTON_START_X_MM, 0.0, BUTTON_START_Z_MM),
        (x, y, z),
    ]
    arm = arm_web.arm
    saved = {n: getattr(arm, n).angle for n in arm_web.IK_JOINTS}
    paths = []
    try:
        for index, (tx, ty, tz) in enumerate(targets):
            target = {
                "x": tx, "y": ty, "z": tz, "tool_pitch": BUTTON_TOOL_PITCH,
            }
            path = (arm_web.plan_locked_pitch_path(target) if index == 0
                    else _plan_early_z_press_path(arm_web, target))
            paths.append(path)
            for name, angle in path[-1].items():
                getattr(arm, name).angle = float(angle)
        return paths
    finally:
        for name, angle in saved.items():
            getattr(arm, name).angle = angle


def move_to_xyz_and_home(board, x, y, z):
    """Horizontal tool: staging, early-Z continuous press, then home."""
    x, y, z = map(float, (x, y, z))
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError("XYZ coordinates must be finite.")

    import arm_web
    from roboarm import get_shared_arm

    arm, newly_attached = get_shared_arm(board)
    home = dict(arm_web.START_ANGLES)
    home["gripper"] = 180
    arm_web.arm = arm
    if newly_attached:
        arm.write(**home)
    else:
        _return_home(arm, home, arm_web.AUTO_MAX_STEP_DEG,
                     arm_web.STEP_DELAY)
    time.sleep(0.6)
    paths = plan_button_paths(arm_web, x, y, z)

    print("ARM: horizontal tool; from (250, 0, 100) Z descends early while X advances.")
    for path in paths:
        _execute_plan(arm, path, arm_web.STEP_DELAY)
    print(f"ARM: button target ({x:g}, {y:g}, {z:g}) commanded. Holding...")
    time.sleep(BUTTON_HOLD_SECONDS)

    # Retrace the same simultaneous transition before folding home.
    # Include preceding endpoints so short segments are fully retraced.
    for index in (1,):
        _execute_plan(arm, list(reversed(paths[index])) + [paths[index-1][-1]],
                      arm_web.STEP_DELAY)
    _return_home(arm, home, arm_web.AUTO_MAX_STEP_DEG, arm_web.STEP_DELAY)
    print("ARM: home commanded.")
    return True

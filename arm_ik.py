
"""Cartesian inverse and forward kinematics for ArmRobotics 2026."""

from dataclasses import dataclass
from math import acos, atan2, cos, degrees, hypot, radians, sin

LINK1 = 120.0
LINK2 = 120.0

HOME_BASE = 95.0
HOME_SHOULDER = 180.0
HOME_ELBOW = 180.0
HOME_WRIST = 20.0
HOME_GRIPPER = 180.0

HOME_REACH = LINK1 + LINK2

SERVO_MIN = 0.0
SERVO_MAX = 180.0


class IKError(ValueError):
    pass


@dataclass(frozen=True)
class JointSolution:
    base: float
    shoulder: float
    elbow: float
    wrist: float
    x: float
    y: float
    z: float


def checked_angle(name, angle):
    if angle < SERVO_MIN - 1e-6 or angle > SERVO_MAX + 1e-6:
        raise IKError(
            f"{name} requires {angle:.1f} degrees, "
            "outside the 0-180 degree servo range."
        )
    return round(max(SERVO_MIN, min(SERVO_MAX, angle)), 1)


def forward_kinematics(base, shoulder, elbow):
    """Return the gripper endpoint in mm relative to the home endpoint."""

    base_delta = radians(float(base) - HOME_BASE)

    q1 = radians(90.0 + float(shoulder) - HOME_SHOULDER)
    q2 = radians(float(elbow) - HOME_ELBOW)

    radius = LINK1 * cos(q1) + LINK2 * cos(q1 + q2)
    absolute_z = LINK1 * sin(q1) + LINK2 * sin(q1 + q2)

    x = radius * cos(base_delta)
    y = radius * sin(base_delta)
    z = absolute_z - HOME_REACH

    return x, y, z


def inverse_kinematics(
    x,
    y,
    z,
    tool_pitch=0.0,
    elbow_up=False,
):
    """Solve a Cartesian XYZ target relative to the configured home endpoint."""

    x = float(x)
    y = float(y)
    z = float(z)
    tool_pitch = float(tool_pitch)

    absolute_z = HOME_REACH + z
    radius = hypot(x, y)
    distance = hypot(radius, absolute_z)

    if distance > LINK1 + LINK2 + 1e-6:
        raise IKError(
            "Target is beyond maximum reach. "
            "Try moving Z downward or reducing X/Y."
        )

    if distance < abs(LINK1 - LINK2) - 1e-6:
        raise IKError("Target is inside the arm's minimum reach.")

    c2 = (
        radius * radius
        + absolute_z * absolute_z
        - LINK1 * LINK1
        - LINK2 * LINK2
    ) / (2.0 * LINK1 * LINK2)

    c2 = max(-1.0, min(1.0, c2))

    q2 = acos(c2)

    if not elbow_up:
        q2 = -q2

    q1 = atan2(absolute_z, radius) - atan2(
        LINK2 * sin(q2),
        LINK1 + LINK2 * cos(q2),
    )

    q1_deg = degrees(q1)
    q2_deg = degrees(q2)

    if radius < 1e-8:
        base = HOME_BASE
    else:
        base = HOME_BASE + degrees(atan2(y, x))

    shoulder = HOME_SHOULDER + q1_deg - 90.0
    elbow = HOME_ELBOW + q2_deg

    wrist = HOME_WRIST - (
        tool_pitch - (q1_deg - 90.0) - q2_deg
    )

    return JointSolution(
        base=checked_angle("Base", base),
        shoulder=checked_angle("Shoulder", shoulder),
        elbow=checked_angle("Elbow", elbow),
        wrist=checked_angle("Wrist", wrist),
        x=x,
        y=y,
        z=z,
    )

"""Three-link Cartesian kinematics for the ArmRobotics arm.

Coordinates are millimeters from the shoulder/base model origin.
X is forward, Y is right, Z is up.
Angles and wrist calibration are approximate until physically verified.
"""

from dataclasses import dataclass
from math import acos, atan2, cos, degrees, hypot, isfinite, radians, sin

LINK1 = 120.0
LINK2 = 120.0
TOOL_LENGTH = 175.0

HOME_BASE = 94.0
HOME_SHOULDER = 180.0
HOME_ELBOW = 180.0
HOME_WRIST = 20.0
HOME_GRIPPER = 180.0

HOME_SHOULDER_PITCH = 170.0
HOME_ELBOW_BEND = -120.0
HOME_TOOL_PITCH = 0.0
WRIST_DEGREES_PER_SERVO = 90.0 / (144.0 - 20.0)

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
    if not isfinite(angle) or angle < SERVO_MIN - 1e-6 or angle > SERVO_MAX + 1e-6:
        raise IKError(f"{name} requires {angle:.2f} degrees, outside 0-180.")
    return round(max(SERVO_MIN, min(SERVO_MAX, angle)), 2)


def forward_kinematics(base, shoulder, elbow, wrist=HOME_WRIST):
    """Return closed-gripper tip XYZ in mm, relative to the base model origin."""
    yaw = radians(float(base) - HOME_BASE)
    q1 = radians(HOME_SHOULDER_PITCH + float(shoulder) - HOME_SHOULDER)
    bend = radians(HOME_ELBOW_BEND - (float(elbow) - HOME_ELBOW))
    q2 = q1 + bend
    home_q2 = HOME_SHOULDER_PITCH + HOME_ELBOW_BEND
    tool_pitch = q2 + radians(HOME_TOOL_PITCH - home_q2) + radians(
        (float(wrist) - HOME_WRIST) * WRIST_DEGREES_PER_SERVO
    )
    reach = LINK1 * cos(q1) + LINK2 * cos(q2) + TOOL_LENGTH * cos(tool_pitch)
    z = LINK1 * sin(q1) + LINK2 * sin(q2) + TOOL_LENGTH * sin(tool_pitch)
    return (reach * cos(yaw), reach * sin(yaw), z)


def inverse_kinematics(x, y, z, tool_pitch=0.0, elbow_up=False):
    """Solve XYZ of the closed-gripper tip at a specified absolute tool pitch."""
    x, y, z, tool_pitch = map(float, (x, y, z, tool_pitch))
    if not all(map(isfinite, (x, y, z, tool_pitch))):
        raise IKError("Coordinates and pitch must be finite numbers.")

    radius = hypot(x, y)
    pitch = radians(tool_pitch)
    wrist_r = radius - TOOL_LENGTH * cos(pitch)
    wrist_z = z - TOOL_LENGTH * sin(pitch)
    d = hypot(wrist_r, wrist_z)

    if d > LINK1 + LINK2 + 1e-6 or d < abs(LINK1 - LINK2) - 1e-6:
        raise IKError("Target is outside the two-arm reach for this tool pitch.")

    c = (d*d - LINK1*LINK1 - LINK2*LINK2) / (2 * LINK1 * LINK2)
    bend = acos(max(-1.0, min(1.0, c)))
    if not elbow_up:
        bend = -bend

    q1 = atan2(wrist_z, wrist_r) - atan2(
        LINK2 * sin(bend), LINK1 + LINK2 * cos(bend)
    )
    q2 = q1 + bend
    yaw = degrees(atan2(y, x)) if radius > 1e-8 else 0.0

    shoulder = HOME_SHOULDER + degrees(q1) - HOME_SHOULDER_PITCH
    elbow = HOME_ELBOW - (degrees(bend) - HOME_ELBOW_BEND)
    home_q2 = HOME_SHOULDER_PITCH + HOME_ELBOW_BEND
    wrist = HOME_WRIST + (
        tool_pitch - (degrees(q2) + HOME_TOOL_PITCH - home_q2)
    ) / WRIST_DEGREES_PER_SERVO

    result = JointSolution(
        base=checked_angle("Base", HOME_BASE + yaw),
        shoulder=checked_angle("Shoulder", shoulder),
        elbow=checked_angle("Elbow", elbow),
        wrist=checked_angle("Wrist", wrist),
        x=x, y=y, z=z,
    )
    actual = forward_kinematics(
        result.base, result.shoulder, result.elbow, result.wrist
    )
    if hypot(actual[0] - x, actual[1] - y) > 1.0 or abs(actual[2] - z) > 1.0:
        raise IKError("Requested pose cannot be reproduced within 1 mm.")
    return result

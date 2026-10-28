
"""Inverse and forward kinematics for ArmRobotics 2026."""
from dataclasses import dataclass
from math import acos, atan2, cos, degrees, hypot, radians, sin

LINK1 = 120.0  # Upper arm, mm
LINK2 = 120.0  # Forearm, mm

# Initial calibration assumptions. Adjust after testing individual joints.
BASE_CENTER = 90.0
BASE_DIRECTION = 1.0

SHOULDER_VERTICAL = 90.0
SHOULDER_DIRECTION = 1.0

# Servo angle when the two links form a straight line.
ELBOW_STRAIGHT = 90.0
ELBOW_DIRECTION = 1.0

WRIST_REFERENCE = 90.0
WRIST_DIRECTION = -1.0

SERVO_MIN = 0.0
SERVO_MAX = 180.0


class IKError(ValueError):
    """Invalid or unreachable target."""


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
    if not SERVO_MIN <= angle <= SERVO_MAX:
        raise IKError(
            f"{name} requires {angle:.1f} degrees; "
            "outside the 0-180 degree servo range."
        )
    return round(angle, 1)


def inverse_kinematics(x, y, z, tool_pitch=0.0, elbow_up=False):
    """Solve a Cartesian target in mm.

    z is relative to the shoulder pivot.
    tool_pitch is measured relative to horizontal, in degrees.
    """
    x, y, z = float(x), float(y), float(z)

    r = hypot(x, y)
    distance = hypot(r, z)

    if distance > LINK1 + LINK2 + 1e-6:
        raise IKError(f"Target exceeds maximum reach of {LINK1 + LINK2:.0f} mm.")

    if distance < abs(LINK1 - LINK2) - 1e-6:
        raise IKError("Target is inside the minimum reachable radius.")

    c2 = (
        r * r + z * z - LINK1 * LINK1 - LINK2 * LINK2
    ) / (2.0 * LINK1 * LINK2)
    c2 = max(-1.0, min(1.0, c2))

    q2 = acos(c2)
    if not elbow_up:
        q2 = -q2

    q1 = atan2(z, r) - atan2(
        LINK2 * sin(q2),
        LINK1 + LINK2 * cos(q2),
    )

    q1_deg = degrees(q1)
    q2_deg = degrees(q2)
    base_deg = degrees(atan2(y, x))

    base = BASE_CENTER + BASE_DIRECTION * base_deg

    # q1 is measured from horizontal, so vertical means 90 degrees.
    shoulder = SHOULDER_VERTICAL + SHOULDER_DIRECTION * (q1_deg - 90.0)

    # q2=0 means both links are straight.
    elbow = ELBOW_STRAIGHT + ELBOW_DIRECTION * q2_deg

    # Compensate wrist rotation to achieve the requested tool pitch.
    wrist = WRIST_REFERENCE + WRIST_DIRECTION * (
        float(tool_pitch) - q1_deg - q2_deg
    )

    return JointSolution(
        base=checked_angle("Base", base),
        shoulder=checked_angle("Shoulder", shoulder),
        elbow=checked_angle("Elbow", elbow),
        wrist=checked_angle("Wrist", wrist),
        x=x, y=y, z=z,
    )


def forward_kinematics(base, shoulder, elbow):
    """Return the modeled forearm endpoint (x, y, z), in mm."""
    base_deg = (float(base) - BASE_CENTER) / BASE_DIRECTION
    q1_deg = 90.0 + (
        float(shoulder) - SHOULDER_VERTICAL
    ) / SHOULDER_DIRECTION
    q2_deg = (
        float(elbow) - ELBOW_STRAIGHT
    ) / ELBOW_DIRECTION

    q1 = radians(q1_deg)
    q2 = radians(q2_deg)

    r = LINK1 * cos(q1) + LINK2 * cos(q1 + q2)
    z = LINK1 * sin(q1) + LINK2 * sin(q1 + q2)
    base_angle = radians(base_deg)

    return (
        r * cos(base_angle),
        r * sin(base_angle),
        z,
    )

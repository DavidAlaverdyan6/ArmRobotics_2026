
"""Inverse kinematics, forward kinematics, and Cartesian paths."""

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, isfinite, radians, sin, sqrt, ceil


LINK1 = 120.0
LINK2 = 120.0

BASE_CENTER = 90.0
BASE_DIRECTION = 1.0
SHOULDER_VERTICAL = 90.0
SHOULDER_DIRECTION = 1.0
ELBOW_STRAIGHT = 90.0
ELBOW_DIRECTION = 1.0
WRIST_REFERENCE = 90.0
WRIST_DIRECTION = -1.0

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


@dataclass(frozen=True)
class CartesianWaypoint:
    x: float
    y: float
    z: float
    base: float
    shoulder: float
    elbow: float
    wrist: float


@dataclass(frozen=True)
class CartesianTrajectory:
    start: tuple
    target: tuple
    distance_mm: float
    waypoints: tuple


def _finite(value, name):
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise IKError(f"{name} must be a number.")

    if not isfinite(value):
        raise IKError(f"{name} must be finite.")

    return value


def checked_angle(name, angle):
    if not isfinite(angle) or not SERVO_MIN <= angle <= SERVO_MAX:
        raise IKError(
            f"{name} requires {angle:.1f} degrees, outside 0-180."
        )
    return round(angle, 2)


def inverse_kinematics(x, y, z, tool_pitch=0.0, elbow_up=False):
    """Solve a Cartesian target in mm; Z is relative to the shoulder."""
    x = _finite(x, "x")
    y = _finite(y, "y")
    z = _finite(z, "z")
    pitch = _finite(tool_pitch, "tool_pitch")

    r = hypot(x, y)
    distance = hypot(r, z)

    if distance > LINK1 + LINK2 + 1e-6:
        raise IKError(f"Target exceeds {LINK1 + LINK2:.0f} mm reach.")

    if distance < abs(LINK1 - LINK2) - 1e-6:
        raise IKError("Target is inside the minimum reachable radius.")

    c2 = (
        r*r + z*z - LINK1*LINK1 - LINK2*LINK2
    ) / (2.0 * LINK1 * LINK2)
    c2 = max(-1.0, min(1.0, c2))

    # atan2 form makes the elbow branch explicit.
    s2 = sqrt(max(0.0, 1.0 - c2*c2))
    if not elbow_up:
        s2 = -s2

    q2 = atan2(s2, c2)
    q1 = atan2(z, r) - atan2(
        LINK2 * sin(q2),
        LINK1 + LINK2 * cos(q2),
    )

    q1_deg = degrees(q1)
    q2_deg = degrees(q2)
    base_deg = degrees(atan2(y, x))

    base = BASE_CENTER + BASE_DIRECTION * base_deg
    shoulder = SHOULDER_VERTICAL + SHOULDER_DIRECTION * (q1_deg - 90.0)
    elbow = ELBOW_STRAIGHT + ELBOW_DIRECTION * q2_deg
    wrist = WRIST_REFERENCE + WRIST_DIRECTION * (
        pitch - q1_deg - q2_deg
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

    r = LINK1*cos(q1) + LINK2*cos(q1 + q2)
    z = LINK1*sin(q1) + LINK2*sin(q1 + q2)
    a = radians(base_deg)

    return r*cos(a), r*sin(a), z


def cartesian_interpolation(
    start,
    target,
    tool_pitch=0.0,
    elbow_up=False,
    step_mm=4.0,
):
    """Plan a straight Cartesian path; does not move hardware."""
    if len(start) != 3 or len(target) != 3:
        raise IKError("Start and target must each contain x, y, z.")

    start = tuple(_finite(v, "start coordinate") for v in start)
    target = tuple(_finite(v, "target coordinate") for v in target)
    step_mm = _finite(step_mm, "step_mm")

    if step_mm <= 0:
        raise IKError("step_mm must be greater than zero.")

    # Validate both ends before generating a trajectory.
    inverse_kinematics(*start, tool_pitch=tool_pitch, elbow_up=elbow_up)
    inverse_kinematics(*target, tool_pitch=tool_pitch, elbow_up=elbow_up)

    dx = target[0] - start[0]
    dy = target[1] - start[1]
    dz = target[2] - start[2]
    distance = sqrt(dx*dx + dy*dy + dz*dz)

    # Smoothstep changes timing along the straight path, not its geometry.
    # Factor 1.5 keeps the maximum spacing approximately within step_mm.
    count = max(1, ceil(distance * 1.5 / step_mm))
    waypoints = []

    for i in range(1, count + 1):
        t = i / count
        s = t*t*(3.0 - 2.0*t)

        x = start[0] + dx*s
        y = start[1] + dy*s
        z = start[2] + dz*s

        solution = inverse_kinematics(
            x, y, z,
            tool_pitch=tool_pitch,
            elbow_up=elbow_up,
        )

        waypoints.append(CartesianWaypoint(
            x=x, y=y, z=z,
            base=solution.base,
            shoulder=solution.shoulder,
            elbow=solution.elbow,
            wrist=solution.wrist,
        ))

    return CartesianTrajectory(
        start=start,
        target=target,
        distance_mm=distance,
        waypoints=tuple(waypoints),
    )

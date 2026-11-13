#!/usr/bin/env python3
"""ArmRobotics main route: navigation and fixed robot tasks."""

import math
import time

import smbus
from telemetrix import telemetrix

import line_tracker
import motors
from forward_line_pid import ForwardLinePID
from navigation_core import FreshFrames
from turn_alignment import align_to_crossbar_center
from stm32_reset import STM32_RESET_GPIO, reset_stm32
from sensors.vl53l0x_sensor import VL53L0XSensor

PROGRAM_BUILD = "hsv-blue-light-filter-2026-11-11.43"

# Previously tested differential2 line-following speeds.
SERIAL_PORT = "/dev/serial0"
BASE_SPEED = 255
MAX_SPEED = 255
MIN_SPEED = 0
MANEUVER_SPEED = 255
MANEUVER_MAX_SPEED = 255
KP = 100
KI = 0
KD = 5
MAX_CORRECTION = 255
TARGET_POSITION = line_tracker.CENTER_POSITION
# Task/maneuver timing remains unchanged. the 25 Hz cadence is used
# only by route line-following and post-turn reacquisition below.
LOOP_DELAY = 0.02
ROUTE_LOOP_DELAY = 0.04
ROUTE_JUNCTION_POLL_DELAY = 0.01
ROUTE_LINE_LOSS_CONFIRM = 3
ROUTE_LINE_REVERSE_SPEED = 80
ROUTE_LINE_REVERSE_TIMEOUT = 5.0
ROUTE_LINE_REVERSE_CONFIRM = 3

# Line-strength limits reused by the button positioning task.
START_CENTER_BLACK_MIN = 0.30
START_OUTER_BLACK_MAX = 0.70

# Junction recognition is separate from line tracking.
BLACK_ON = 0.85
BLACK_OFF = 0.65
# A rubbed-off gray line can darken the edge sensor while the inward sensor
# remains only moderately dark. Real tape covers both sensors together.
JUNCTION_SIDE_SUPPORT_ON = 0.75
JUNCTION_SIDE_SUPPORT_OFF = 0.55
JUNCTION_CENTER_MIN = 0.50
JUNCTION_CONFIRM = 2
ROUTE_FAST_EDGE_BLACK_MIN = 0.90
ROUTE_FAST_SIDE_BLACK_MIN = 0.75
ROUTE_FAST_CENTER_BLACK_MIN = 0.70
JUNCTION_CLEAR = 3
ROUTE_JUNCTION_CLEAR = 4
# Forward line following is always at base PWM 255. No acceleration ramp.
# After any gyro turn, differential2 centers the robot again.
REACQUIRE_TIMEOUT = 2.0
REACQUIRE_TOLERANCE = 0.65
REACQUIRE_CONFIRM = 4
ROUTE_TURN_LINE_LOSS_SECONDS = 1.0
ROUTE_TURN_REVERSE_SPEED = 80
ROUTE_TURN_REVERSE_TIMEOUT = 5.0
ROUTE_TURN_REVERSE_CONFIRM = 3

I2C_BUS = 1
MPU_ADDR = 0x68
GYRO_Z_REG = 0x47
GYRO_SCALE = 131.0
GYRO_TIMEOUT = 5.0
GYRO_CALIBRATION_SAMPLES = 20
GYRO_CALIBRATION_SETTLE_SECONDS = 0.08
GYRO_CALIBRATION_SAMPLE_DELAY = 0.001
GYRO_DIRECTION_SETTLE_SECONDS = 0.03
GYRO_DIRECTION_MIN_RATE = 8.0
GYRO_DIRECTION_CONFIRM_SAMPLES = 3
GYRO_REVERSE_CONFIRM_SECONDS = 0.05
PIVOT_SETTLE_SECONDS = 0.1
TURN_ANGLE = 75.0
TURN_90_ANGLE = 90.0
TURN_PWM_FAST = 180
TURN_PWM_SLOW = 150
TURN_PWM_EDGE = 100
TURN_90_SLOWDOWN_ANGLE = 60.0
TURN_90_SENSOR_ON = 0.75
TURN_90_CENTER_ON = 0.55
TURN_90_CENTER_CONFIRM = 2
TURN_90_EDGE_CONFIRM = 2
SLOW_90_PWM_FAST = 110
SLOW_90_PWM_SLOW = 70
SLOW_90_REVERSE_SECONDS = 0.15
SLOW_90_REVERSE_SPEED = 80
# main_old's uncalibrated alignment PWM and 0.50 s travel.
# Keep this independent of BASE_SPEED and MANEUVER_SPEED.
TURN_90_FORWARD_SPEED = 80
TURN_90_FORWARD_MAX_SPEED = 120
TURN_90_FORWARD_TIME = 0.5
TURN_TOLERANCE_DEG = 8.0
TURN_CORRECTION_START_DEG = 4.0

BUTTON_BUMPER_START_CM = 20.0
BUTTON_BUMPER_START_TOLERANCE_CM = 1.0
BUTTON_BUMPER_RAM_SECONDS = 1.0
BUTTON_BUMPER_APPROACH_TIMEOUT = 18.0
BUTTON_BUMPER_SPEED = 255
BUTTON_BUMPER_STATUS_SECONDS = 0.25
BUTTON_RETURN_JUNCTION_DELAY = 0.5
BUTTON_RETURN_JUNCTION_CONFIRM = 4
BUTTON_RETURN_CENTER_BLACK_MIN = 0.70
BUTTON_RETURN_CENTER_SENSOR_COUNT = 2
BUTTON_DEPART_SECONDS = 0.4
BUTTON_FORWARD_MIN_PWM = 40

# Charging: 9 s red blink, then 2 s solid green; total 11 s before R.
CHARGE_SECONDS = 11.0
CHARGE_RED_SECONDS = 9.0
CHARGE_BLINK_HALF_PERIOD = 0.5
# Reverse line-following geometry (all distances are millimetres).  The line
# tracker is 70 mm in front of the axle, so it becomes a trailing sensor while
# reversing; it cannot use the forward controller unchanged.
RETURN_SPEED = 130
RETURN_SLOW_SPEED = 115
REVERSE_MIN_PWM = 80
REVERSE_MAX_PWM = 150
REVERSE_TRACKER_OFFSET_MM = 70.0
REVERSE_WHEELBASE_MM = 226.0
REVERSE_SENSOR_SPAN_MM = 60.0
REVERSE_POSITION_MM_PER_UNIT = REVERSE_SENSOR_SPAN_MM / 7.0

# Separate reverse PD/gyro observer.  Tune these on the real robot only.
REVERSE_LATERAL_KP = 2.6          # PWM / mm from estimated axle error
REVERSE_LATERAL_KD = 0.055        # PWM / (mm/s), suppresses oscillation
REVERSE_HEADING_KP = 0.70         # PWM / degree from MPU heading error
REVERSE_YAW_RATE_KD = 0.12        # PWM / (degree/s), damps yaw rate
REVERSE_REFERENCE_FOLLOW = 2.5    # 1/s: lets course reference follow curves
REVERSE_REFERENCE_WINDOW_MM = 7.0
REVERSE_MAX_BRAKE_PWM = 70
REVERSE_PWM_SLEW_PER_LOOP = 7
REVERSE_LINE_LOSS_TIMEOUT = 0.8
BUTTON_REVERSE_LINE_LOSS_TIMEOUT = 5.0
REVERSE_GYRO_SPEED = 90
# Set to -1 only if a brief bench test proves the physical motor sides are
# wired opposite to the documented LEFT/RIGHT convention.
REVERSE_STEER_SIGN = 1
RETURN_MAX_TIME = 24.0

RETURN_FORWARD_TIME = 0.5

FIRE_APPROACH_SPEED = 255
FIRE_APPROACH_MAX_SPEED = 255
FIRE_APPROACH_TIMEOUT = 12.0
FIRE_FINAL_ADVANCE_SECONDS = 0.3
FIRE_LINE_RECOVERY_SPEED = 80
FIRE_LINE_RECOVERY_TIMEOUT = 5.0
FIRE_LINE_RECOVERY_CONFIRM = 2
FIRE_LINE_BLACKNESS_MIN = 0.75
FIRE_RETURN_MAX_TIME = 40.0

# CUBE1: arm stays at this XYZ with the gripper open while the chassis moves.
CUBE1_STAGE_X_MM = 250.0
CUBE1_STAGE_Y_MM = 0.0
CUBE1_STAGE_Z_MM = 100.0
CUBE1_X_MM = 200.0
CUBE1_Y_MM = 0.0
CUBE1_Z_MM = -180.0
CUBE1_TOOL_PITCH_DEG = -90.0
CUBE1_PIT_ALL_BLACK_MIN = 0.85
CUBE1_PIT_CONFIRM_FRAMES = 4
CUBE1_EMERGENCY_TIMEOUT = 8.0
CUBE1_BLINK_HALF_PERIOD = 0.25
CUBE1_DROP_CONFIRM_FRAMES = 3
CUBE1_DROP_TIMEOUT = 20.0
CUBE1_APPROACH_SPEED = 255
CUBE1_APPROACH_MAX_SPEED = 255
CUBE1_SENSOR_CONFIRM = 3
CUBE1_LINE_LOSS_CONFIRM = 5

# COLOR: stop, find a stable colored marker, and show that color on the RGB LED.
# Pixel classification (including the saturated-blue filter that rejects the
# blue cast from white floor and black tape) lives in color_marker_detector.py.
COLOR_MARKER_TIMEOUT = 10.0
COLOR_MARKER_HOLD_SECONDS = 1.0
COLOR_MARKER_MIN_AREA = 700
COLOR_MARKER_STABLE_FRAMES = 3
COLOR_ARM_ANGLES = {
    "base": 90.0,
    "shoulder": 160.0,
    "elbow": 180.0,
    "wrist": 0.0,
    "gripper": 0.0,
}
COLOR_LED_HOLD = None

# Automatic route: one active command for each junction; Q stops.
ROUTE = [
    # "BUTTON",
    # "L90",
    "CUBE1",
    "R",
    "L",
    "R",
    "CHARGING",
    "COLOR",
    "L",
    "Q",
    # "FIREFIGHTER",
    # "S",
    # "L",
    # "R",
    # "R",
    # "S",
    # "R",
    # "S",
    # "L",
    # "L",
    # "S",
    # "Q",
]

ALLOWED_ROUTE_COMMANDS = {
    "L", "R", "L90", "R90", "L90SLOW", "R90SLOW", "S", "SLOW",
    "BUTTON", "CHARGING", "FIREFIGHTER", "CUBE1", "COLOR", "Q",
}


line_follower = line_tracker.LineTrackingEngine(
    base_speed=BASE_SPEED,
    max_speed=MAX_SPEED,
    min_speed=MIN_SPEED,
    kp=KP,
    kd=KD,
    target_position=TARGET_POSITION,
)


forward_line_pid = ForwardLinePID(motor_driver=motors)


def stop():
    """Stop chassis PWM immediately, then clear navigation controller state."""
    motors.motor_stop()
    forward_line_pid.reset()
    line_follower.stop()


def differential_drive(state, speed=BASE_SPEED, max_speed=MAX_SPEED):
    """Use exact differential2.py PID maths at constant base PWM 255.

    Legacy per-task speed arguments are intentionally ignored, per the
    selected all-forward-255 policy. Reverse uses its separate controller.
    """
    return forward_line_pid.drive(state)


def forward_drive_straight(speed=None, max_speed=None):
    """Open-loop forward fallback at full PWM; keep caller API stable."""
    forward_line_pid.reset()
    motors.set_speeds(BASE_SPEED, BASE_SPEED)
    return True


def differential_reverse(state, speed, controller=None):
    """Drive the dedicated reverse controller (legacy fallback for tests)."""
    if controller is not None:
        return controller.drive(state, speed)
    return line_follower.reverse(state, speed)


class JunctionDetector:
    """Only branch sensing. It never calculates line position or steers motors."""

    def __init__(self):
        self.left_black = False
        self.right_black = False

    @staticmethod
    def hysteresis(previous, value):
        return value > BLACK_OFF if previous else value >= BLACK_ON

    def read(self, state):
        values = state["normalized"]
        if len(values) != 8:
            raise ValueError("Expected eight line sensor values")

        left_edge = self.hysteresis(self.left_black, values[0])
        right_edge = self.hysteresis(self.right_black, values[7])
        left_support = values[1] >= (
            JUNCTION_SIDE_SUPPORT_OFF
            if self.left_black else JUNCTION_SIDE_SUPPORT_ON
        )
        right_support = values[6] >= (
            JUNCTION_SIDE_SUPPORT_OFF
            if self.right_black else JUNCTION_SIDE_SUPPORT_ON
        )
        self.left_black = left_edge and left_support
        self.right_black = right_edge and right_support

        side_pattern = self.left_black or self.right_black

        # An outer sensor alone means the robot is off-center, NOT a junction.
        center_present = any(
            values[index] >= JUNCTION_CENTER_MIN
            for index in (2, 3, 4, 5)
        )
        candidate = (
            state["position"] is not None
            and center_present
            and side_pattern
        )

        return self.left_black, self.right_black, candidate


def strong_route_junction(state):
    """A full black branch can be accepted in one fresh sensor snapshot."""
    values = state.get("normalized", [])
    if state.get("position") is None or len(values) != 8:
        return False
    center_count = sum(
        values[index] >= ROUTE_FAST_CENTER_BLACK_MIN
        for index in (2, 3, 4, 5)
    )
    left_branch = (
        values[0] >= ROUTE_FAST_EDGE_BLACK_MIN
        and values[1] >= ROUTE_FAST_SIDE_BLACK_MIN
    )
    right_branch = (
        values[7] >= ROUTE_FAST_EDGE_BLACK_MIN
        and values[6] >= ROUTE_FAST_SIDE_BLACK_MIN
    )
    return center_count >= 2 and (left_branch or right_branch)


def gyro_raw(bus):
    hi = bus.read_byte_data(MPU_ADDR, GYRO_Z_REG)
    lo = bus.read_byte_data(MPU_ADDR, GYRO_Z_REG + 1)
    value = (hi << 8) | lo
    return value - 65536 if value >= 32768 else value


def gyro_rate(bus, offset):
    return (gyro_raw(bus) - offset) / GYRO_SCALE


def gyro_setup(bus):
    bus.write_byte_data(MPU_ADDR, 0x6B, 0)
    bus.write_byte_data(MPU_ADDR, 0x1B, 0)
    time.sleep(0.2)


def gyro_calibrate(bus, settle_seconds=GYRO_CALIBRATION_SETTLE_SECONDS):
    stop()
    time.sleep(settle_seconds)
    values = []
    for _ in range(GYRO_CALIBRATION_SAMPLES):
        values.append(gyro_raw(bus))
        time.sleep(GYRO_CALIBRATION_SAMPLE_DELAY)
    return sum(values) / len(values)


class Heading:
    def __init__(self, bus, offset, left_positive):
        self.bus = bus
        self.offset = offset
        self.sign = left_positive
        self.angle = 0.0
        self.last = time.monotonic()
        self.last_rate = 0.0

    def read(self):
        now = time.monotonic()
        rate = self.sign * gyro_rate(self.bus, self.offset)
        self.angle += rate * max(0.0, now - self.last)
        self.last = now
        self.last_rate = rate
        return self.angle


def _wrap_degrees(angle):
    """Keep a heading difference in [-180, 180) degrees."""
    return (angle + 180.0) % 360.0 - 180.0


class ReverseLineController:
    """Geometry-aware line controller for a tracker ahead of the axle.

    The tracker measures the line at x=+70 mm.  During reverse motion that
    point trails the axle, so raw sensor error alone is phase-lagged and makes
    an ordinary PD loop oscillate.  The MPU heading supplies the missing
    orientation term; while centred, its reference slowly follows bends in
    the track instead of forcing the robot to a fixed global compass course.
    """

    def __init__(
        self, heading, line_loss_timeout=REVERSE_LINE_LOSS_TIMEOUT,
        edge_recovery=False,
    ):
        self.heading = heading
        self.line_loss_timeout = line_loss_timeout
        self.edge_recovery = edge_recovery
        self.reference_heading = None
        self.previous_axle_error = None
        self.previous_time = None
        self.previous_pwm = None
        self.line_lost_since = None
        self.last_angle = 0.0
        self.last_axle_error = None
        self.mode = "LINE"

    @staticmethod
    def _slew(previous, target):
        if previous is None:
            return target
        step = REVERSE_PWM_SLEW_PER_LOOP
        return max(previous - step, min(previous + step, target))

    def _wheel_command(self, base_pwm, steering):
        """Brake one reverse wheel only; slew-rate-limit both PWM values."""
        base = max(REVERSE_MIN_PWM, min(REVERSE_MAX_PWM, round(base_pwm)))
        brake = min(REVERSE_MAX_BRAKE_PWM, abs(steering))
        left = right = -base

        # Positive steering means brake the right wheel.  With reverse motion
        # this yields the opposite body reaction to forward travel.  The sign
        # is a single hardware calibration point, not a PWM swap workaround.
        if steering * REVERSE_STEER_SIGN >= 0:
            right = -max(REVERSE_MIN_PWM, base - round(brake))
        else:
            left = -max(REVERSE_MIN_PWM, base - round(brake))

        old_left, old_right = self.previous_pwm or (None, None)
        left = self._slew(old_left, left)
        right = self._slew(old_right, right)
        self.previous_pwm = (left, right)
        motors.set_speeds(left, right)
        return True

    def drive(self, state, speed):
        now = time.monotonic()
        angle = self.heading.read()
        rate = self.heading.last_rate
        self.last_angle = angle
        position = state.get("position") if state else None

        if position is None:
            # A short gyro-only bridge prevents an ADC gap from causing a
            # sudden stop.  Most tasks bound this interval; CUBE1 deliberately
            # keeps its gyro-only return active until S8 sees the junction.
            if self.line_lost_since is None:
                self.line_lost_since = now
            if (
                self.line_loss_timeout is not None
                and now - self.line_lost_since > self.line_loss_timeout
            ):
                return False
            return self.drive_gyro(min(speed, REVERSE_GYRO_SPEED), angle, rate)

        self.line_lost_since = None
        sensor_error_mm = position * REVERSE_POSITION_MM_PER_UNIT
        if self.reference_heading is None:
            self.reference_heading = angle
        heading_error = _wrap_degrees(angle - self.reference_heading)

        # The tracker displacement is measured at x=+70 mm, not at the axle.
        # Project the heading component away to obtain the axle-line error.
        axle_error = sensor_error_mm - (
            REVERSE_TRACKER_OFFSET_MM
            * math.sin(math.radians(heading_error))
        )
        self.last_axle_error = axle_error

        dt = 0.0 if self.previous_time is None else now - self.previous_time
        derivative = 0.0
        if (self.previous_axle_error is not None
                and 0.005 <= dt <= 0.25):
            derivative = (axle_error - self.previous_axle_error) / dt
        self.previous_axle_error = axle_error
        self.previous_time = now

        # On a curve the compass heading must be allowed to follow the local
        # track direction, but only once the lateral error is near centre.
        if abs(sensor_error_mm) <= REVERSE_REFERENCE_WINDOW_MM and dt > 0:
            follow = min(1.0, REVERSE_REFERENCE_FOLLOW * dt)
            self.reference_heading += heading_error * follow
            heading_error = _wrap_degrees(angle - self.reference_heading)

        steering = (
            REVERSE_LATERAL_KP * axle_error
            + REVERSE_LATERAL_KD * derivative
            + REVERSE_HEADING_KP * heading_error
            + REVERSE_YAW_RATE_KD * rate
        )
        # Slow somewhat on large deviations, preserving room for differential
        # braking while never exceeding the specified 150-PWM reverse limit.
        base = max(
            RETURN_SLOW_SPEED,
            speed - min(25.0, abs(axle_error) * 0.8),
        )
        if self.edge_recovery and abs(position) >= 1.5:
            # BUTTON can reverse with only S7/S8 seeing the line. Give the
            # outer wheel the full reverse PWM and brake the inner wheel;
            # both wheels remain in reverse instead of pivoting in place.
            base = REVERSE_MAX_PWM
            steering = math.copysign(
                REVERSE_MAX_BRAKE_PWM, sensor_error_mm
            )
        self.mode = "LINE"
        return self._wheel_command(base, steering)

    def drive_gyro(self, speed, angle=None, rate=None):
        """Reverse on a fixed gyro heading, without using the line position."""
        if angle is None:
            angle = self.heading.read()
        if rate is None:
            rate = self.heading.last_rate
        if self.reference_heading is None:
            self.reference_heading = angle
        heading_error = _wrap_degrees(angle - self.reference_heading)
        steering = (
            REVERSE_HEADING_KP * heading_error
            + REVERSE_YAW_RATE_KD * rate
        )
        self.mode = "GYRO"
        self.last_angle = angle
        return self._wheel_command(speed, steering)


def pivot(bus, offset, direction, angle_target=TURN_ANGLE):
    """Original L/R pivot, also used by the existing B/C/F tasks."""
    if direction not in ("L", "R"):
        raise ValueError("Turn direction must be L or R")

    left = direction == "L"
    motors.set_speeds(
        -TURN_PWM_SLOW if left else TURN_PWM_FAST,
        TURN_PWM_FAST if left else -TURN_PWM_SLOW,
    )
    start = previous = time.monotonic()
    angle = signed = 0.0
    try:
        while angle < angle_target:
            now = time.monotonic()
            rate = gyro_rate(bus, offset)
            dt = max(0.0, now - previous)
            previous = now
            angle += abs(rate) * dt
            signed += rate * dt
            if now - start >= GYRO_TIMEOUT:
                print("\nTurn timeout.")
                return False, None
            time.sleep(0.005)
        # Always return the gyro polarity for a physical LEFT turn.  For a
        # right turn the observed polarity is the opposite.  Reverse driving
        # can therefore use the same heading correction after either turn.
        left_sign = None
        if abs(signed) > 20:
            observed_sign = 1 if signed > 0 else -1
            left_sign = observed_sign if left else -observed_sign
        return True, left_sign
    finally:
        stop()
        time.sleep(PIVOT_SETTLE_SECONDS)


def turn_90_pwm(angle, side_seen, slow=False):
    """Consistent PWM ladder for all standard 90-degree turns."""
    pwm = (TURN_PWM_EDGE if side_seen
           else TURN_PWM_SLOW if angle >= TURN_90_SLOWDOWN_ANGLE
           else TURN_PWM_FAST)
    return min(pwm, SLOW_90_PWM_FAST) if slow else pwm


def pivot_90(
    bus,
    offset,
    direction,
    angle_target=TURN_90_ANGLE,
    slow=False,
    calibration=None,
):
    """Shared gyro + line-tracker turn. The preceding calibrated pulse belongs
    to turn_90(), not individual route tasks.

    Begin at PWM180; at 60 degrees use PWM150; when the outer sensor on the
    turning side confirms black use PWM100; stop on a confirmed center-line
    capture. If the sensors miss the line, finish to the gyro 90-degree limit.
    A gyro fault / missing ADC data stops the robot instead of driving blindly.
    """
    if direction not in ("L", "R"):
        raise ValueError("Turn direction must be L or R")
    stop()
    left = direction == "L"
    start = previous = time.monotonic()
    angle = signed = 0.0
    gyro_sign = candidate_sign = None
    direction_samples = 0
    reverse_since = None
    side_hits = center_hits = 0
    side_seen = False
    captured_line = False
    frames = FreshFrames()
    # The wide crossing must clear before a new side-line detection is trusted.
    # This avoids completing the turn on the crossing being exited.
    side_clear_hits = 0
    edge_armed = False
    sensor_index = 0 if left else 7
    center_indices = (3, 4)
    last_sensor_report = 0.0

    try:
        while True:
            now = time.monotonic()
            rate = gyro_rate(bus, offset)
            dt = max(0.0, min(now - previous, 0.1))
            previous = now
            if not math.isfinite(rate) or abs(rate) > 1000:
                raise RuntimeError("GYRO_TURN: invalid angular rate")
            if gyro_sign is None:
                if (now - start >= GYRO_DIRECTION_SETTLE_SECONDS
                        and abs(rate) >= GYRO_DIRECTION_MIN_RATE):
                    observed_sign = 1 if rate > 0 else -1
                    direction_samples = (
                        direction_samples + 1
                        if observed_sign == candidate_sign else 1
                    )
                    candidate_sign = observed_sign
                    if direction_samples >= GYRO_DIRECTION_CONFIRM_SAMPLES:
                        gyro_sign = candidate_sign
                else:
                    direction_samples = 0
                    candidate_sign = None
            if gyro_sign is not None:
                directed = rate * gyro_sign
                if directed < -GYRO_DIRECTION_MIN_RATE:
                    if reverse_since is None:
                        reverse_since = now
                    elif now - reverse_since >= GYRO_REVERSE_CONFIRM_SECONDS:
                        raise RuntimeError("GYRO_TURN: gyro direction reversed")
                else:
                    reverse_since = None
                angle = max(0.0, angle + directed * dt)
                signed += rate * dt

            if calibration is not None and angle >= TURN_90_SLOWDOWN_ANGLE:
                if not line_tracker.all_channels_fresh():
                    raise RuntimeError("GYRO_TURN: stale ADC during line capture")
                state = line_tracker.update(calibration)
                values = state.get("normalized", [])
                if len(values) != 8:
                    raise RuntimeError("GYRO_TURN: expected 8 line sensors")
                if frames.accept(state, now):
                    # The original crossbar must have cleared before we trust
                    # the side sensor as a *new* line. A short repeated black
                    # sample is not sufficient to switch speeds.
                    if values[sensor_index] < TURN_90_SENSOR_ON:
                        side_clear_hits += 1
                        if side_clear_hits >= TURN_90_EDGE_CONFIRM:
                            edge_armed = True
                    elif edge_armed:
                        side_hits += 1
                        if side_hits >= TURN_90_EDGE_CONFIRM:
                            if not side_seen:
                                print(f"\\nGYRO_TURN {direction}: S{sensor_index+1} sees line; PWM100.")
                            side_seen = True
                    if side_seen:
                        if (min(values[i] for i in center_indices) >=
                                TURN_90_CENTER_ON):
                            center_hits += 1
                            if center_hits >= TURN_90_CENTER_CONFIRM:
                                captured_line = True
                                print(f"\\nGYRO_TURN {direction}: center sensors confirmed.")
                                break
                        else:
                            center_hits = 0

            # Gyro is the hard fallback even if neither outer nor center
            # sensors ever detects the new line.
            if angle >= angle_target:
                print(f"\\nGYRO_TURN {direction}: gyro {angle_target:g} degree fallback.")
                break
            if now - start >= GYRO_TIMEOUT or (now - start > 0.7 and angle < 2):
                print(f"\\nGYRO_TURN {direction}: timeout/no rotation at {angle:.1f} degrees")
                return False, None

            pwm = turn_90_pwm(angle, side_seen, slow=slow)
            motors.set_speeds(-pwm if left else pwm, pwm if left else -pwm)
            time.sleep(0.005)

        stop()
        # Measure coast without starting a second arbitrary correction turn.
        settle_until = time.monotonic() + 0.05
        while time.monotonic() < settle_until:
            now = time.monotonic()
            signed += gyro_rate(bus, offset) * max(
                0.0, min(now - previous, 0.1)
            )
            previous = now
            time.sleep(0.005)
        final_angle = abs(signed)
        print(
            f"\\nGYRO_TURN {direction}: target={angle_target:.1f}, "
            f"gyro={final_angle:.1f}, error={final_angle-angle_target:+.1f}; "
            f"finish={'line' if captured_line else 'gyro fallback'}"
        )
        left_sign = None
        if abs(signed) > 5:
            observed_sign = 1 if signed > 0 else -1
            left_sign = observed_sign if left else -observed_sign
        # A properly confirmed center line is success even if wheel inertia
        # makes the gyro angle differ; otherwise the gyro fallback must be
        # close to the requested angle.
        tolerance = min(TURN_TOLERANCE_DEG, angle_target * 0.2)
        return (captured_line or abs(final_angle-angle_target) <= tolerance), left_sign
    finally:
        stop()
        time.sleep(0.02)


def turn_90(calibration, bus, offset, direction, *, slow=False):
    """ONE shared R90/L90: calibrate.py crossing pulse then sensor/gyro turn.

    All tasks call this same function; no per-task PWM, angle or timing hacks.
    """
    if not align_to_crossbar_center(calibration):
        return False, None
    return pivot_90(
        bus, offset, direction, angle_target=TURN_90_ANGLE,
        slow=slow, calibration=calibration,
    )


def reverse_for_short_turn(seconds, speed):
    """Back away briefly from a compact post-CHARGING turn area."""
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            line_follower.reverse_straight(speed)
            time.sleep(LOOP_DELAY)
    finally:
        stop()
    return True


def reverse_to_line_after_route_turn(calibration):
    """Back up slowly until the tracker sees a line on consecutive frames."""
    deadline = time.monotonic() + ROUTE_TURN_REVERSE_TIMEOUT
    confirmed = 0
    try:
        while time.monotonic() < deadline:
            state = line_tracker.update(calibration)
            confirmed = confirmed + 1 if state["position"] is not None else 0
            if confirmed >= ROUTE_TURN_REVERSE_CONFIRM:
                print("\nROUTE TURN: line found while reversing.")
                return True
            line_follower.reverse_straight(ROUTE_TURN_REVERSE_SPEED)
            time.sleep(ROUTE_LOOP_DELAY)
        print("\nROUTE TURN: reverse recovery timed out; stopping.")
        return False
    finally:
        stop()


def reverse_to_line_on_route(calibration, freshness):
    """Back up after route line loss until fresh frames confirm the line."""
    deadline = time.monotonic() + ROUTE_LINE_REVERSE_TIMEOUT
    confirmed = 0
    print("\nROUTE: line lost; reversing slowly to find it.")
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            state = line_tracker.update(calibration)
            if not line_tracker.all_channels_fresh():
                stop()
                confirmed = 0
                time.sleep(ROUTE_JUNCTION_POLL_DELAY)
                continue
            if freshness.accept(state, now):
                confirmed = (
                    confirmed + 1 if state["position"] is not None else 0
                )
                if confirmed >= ROUTE_LINE_REVERSE_CONFIRM:
                    print("\nROUTE: line recovered; resuming route.")
                    return True
            line_follower.reverse_straight(ROUTE_LINE_REVERSE_SPEED)
            time.sleep(ROUTE_JUNCTION_POLL_DELAY)
        print("\nROUTE: reverse recovery timed out; robot stopped.")
        return False
    finally:
        stop()


def reacquire_line(calibration, *, recover_after_line_loss=False):
    """Use the shared differential2 centering after every turn."""
    deadline = time.monotonic() + REACQUIRE_TIMEOUT
    stable = 0
    lost_since = None
    recovery_attempted = False

    try:
        while time.monotonic() < deadline:
            state = line_tracker.update(calibration)
            position = state["position"]

            if position is None:
                now = time.monotonic()
                if lost_since is None:
                    lost_since = now
                    if recover_after_line_loss:
                        deadline = max(
                            deadline,
                            now + ROUTE_TURN_LINE_LOSS_SECONDS
                            + ROUTE_LOOP_DELAY,
                        )
                if (
                    recover_after_line_loss
                    and now - lost_since >= ROUTE_TURN_LINE_LOSS_SECONDS
                ):
                    stop()
                    if recovery_attempted:
                        print("\nROUTE TURN: line lost again after recovery.")
                        return False
                    print(
                        "\nROUTE TURN: line missing for 1 second; "
                        "reversing slowly to find it."
                    )
                    if not reverse_to_line_after_route_turn(calibration):
                        return False
                    recovery_attempted = True
                    lost_since = None
                    stable = 0
                    deadline = time.monotonic() + REACQUIRE_TIMEOUT
                    continue
                if recover_after_line_loss:
                    stop()
                else:
                    forward_drive_straight(
                        speed=MANEUVER_SPEED,
                        max_speed=MANEUVER_MAX_SPEED,
                    )
                stable = 0
            else:
                lost_since = None
                differential_drive(
                    state,
                    speed=MANEUVER_SPEED,
                    max_speed=MANEUVER_MAX_SPEED,
                )
                if abs(position - TARGET_POSITION) <= REACQUIRE_TOLERANCE:
                    stable += 1
                    if stable >= REACQUIRE_CONFIRM:
                        return True
                else:
                    stable = 0

            time.sleep(ROUTE_LOOP_DELAY)

        print("\nLine not centered yet; returning control to route tracking.")
        return not recover_after_line_loss

    finally:
        stop()


def follow_for(calibration, seconds, *, speed=MANEUVER_SPEED,
               max_speed=MANEUVER_MAX_SPEED):
    """Timed forward movement: unchanged differential2 line-following."""
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            state = line_tracker.update(calibration)
            if state["position"] is None:
                forward_drive_straight(
                    speed=speed,
                    max_speed=max_speed,
                )
            else:
                differential_drive(
                    state, speed=speed, max_speed=max_speed
                )
            time.sleep(LOOP_DELAY)
        return True
    finally:
        stop()
        time.sleep(0.15)


def activate_arm_home(board):
    """Enable all arm servos once and command the safe base pose."""
    from arm_web import START_ANGLES
    from roboarm import get_shared_arm

    arm, _ = get_shared_arm(board)
    arm.write(**START_ANGLES)
    print("ARM: active in base pose (90, 170, 180, 0, 180).")


def button_depart_and_check_line(calibration):
    """Leave the turn straight, then inspect the line before approaching."""
    deadline = time.monotonic() + BUTTON_DEPART_SECONDS
    try:
        while time.monotonic() < deadline:
            forward_drive_straight(
                speed=MANEUVER_SPEED,
                max_speed=MANEUVER_MAX_SPEED,
            )
            time.sleep(LOOP_DELAY)
    finally:
        stop()

    state = line_tracker.update(calibration)
    print(f"\nBUTTON: after straight departure, line pos={state['position']}.")
    if state["position"] is None:
        print("BUTTON: line missing after departure; stopping.")
        return False
    return True


def button_safe_forward_drive(state):
    """Same exact 255-PWM PID as the main route, including edge lines."""
    return differential_drive(state)


def button_bumper_approach(calibration, sensor):
    """Use the chassis bumper to press the button; the arm never moves."""
    deadline = time.monotonic() + BUTTON_BUMPER_APPROACH_TIMEOUT
    ram_started = False
    ram_deadline = None
    last_status = 0.0

    try:
        while time.monotonic() < deadline:
            distance_cm = sensor.read_distance_mm() / 10.0
            if not math.isfinite(distance_cm):
                print("\nBUTTON: invalid distance reading; stopping.")
                return False

            state = line_tracker.update(calibration)
            if not ram_started:
                # Begin the bounded 180-PWM bumper run at 20 ± 1 cm.
                if distance_cm <= (
                    BUTTON_BUMPER_START_CM + BUTTON_BUMPER_START_TOLERANCE_CM
                ):
                    ram_started = True
                    ram_deadline = (
                        time.monotonic() + BUTTON_BUMPER_RAM_SECONDS
                    )
                    print(
                        f"\nBUTTON: {distance_cm:.1f} cm; "
                        f"starting {BUTTON_BUMPER_RAM_SECONDS:g}-second "
                        f"bumper press at PWM {BUTTON_BUMPER_SPEED}."
                    )
                elif state["position"] is None:
                    stop()
                    time.sleep(LOOP_DELAY)
                    continue
                else:
                    button_safe_forward_drive(state)
                    time.sleep(LOOP_DELAY)
                    continue

            if time.monotonic() >= ram_deadline:
                stop()
                print(
                    f"\nBUTTON: bumper run complete; final distance "
                    f"{distance_cm:.1f} cm. Returning to junction."
                )
                return True

            now = time.monotonic()
            if now - last_status >= BUTTON_BUMPER_STATUS_SECONDS:
                print(
                    f"\rBUTTON RAM distance={distance_cm:.1f} cm",
                    end="",
                    flush=True,
                )
                last_status = now
            if state["position"] is None:
                # Continue straight only during the bounded bumper press.
                forward_drive_straight(
                    speed=BUTTON_BUMPER_SPEED,
                    max_speed=BUTTON_BUMPER_SPEED,
                )
            else:
                differential_drive(
                    state,
                    speed=BUTTON_BUMPER_SPEED,
                    max_speed=BUTTON_BUMPER_SPEED,
                )
            time.sleep(LOOP_DELAY)

        print("\nBUTTON: bumper approach timeout.")
        return False
    finally:
        stop()



def button_return(
    calibration,
    detector,
    bus,
    offset,
    left_sign,
    label="BUTTON",
    accept_s8_only=False,
    accept_any_junction=False,
    max_return_time=RETURN_MAX_TIME,
    line_loss_timeout=REVERSE_LINE_LOSS_TIMEOUT,
):
    """Return using the geometry-aware reverse line controller."""
    heading = Heading(bus, offset, left_sign)
    reverse_controller = ReverseLineController(
        heading,
        line_loss_timeout=line_loss_timeout,
        edge_recovery=label == "BUTTON",
    )
    start = time.monotonic()
    cleared_count = 0
    outer_confirm_count = 0
    candidate_since = None
    last_status = 0.0

    try:
        while (
            max_return_time is None
            or time.monotonic() - start < max_return_time
        ):
            state = line_tracker.update(calibration)
            left, right, candidate = detector.read(state)
            now = time.monotonic()

            # During BUTTON reverse the line can sit on S7/S8 throughout.
            # That narrow edge line still means the starting junction has
            # cleared; requiring S8 to go white would never arm detection.
            if label == "BUTTON":
                if not candidate:
                    cleared_count += 1
                elif cleared_count < JUNCTION_CLEAR:
                    cleared_count = 0
            elif not left and not right:
                cleared_count += 1

            if accept_any_junction:
                outer_confirm_count = (
                    outer_confirm_count + 1
                    if (
                        cleared_count >= JUNCTION_CLEAR
                        and (candidate or left or right)
                    )
                    else 0
                )
                if outer_confirm_count >= JUNCTION_CONFIRM:
                    print(f"\n{label}: return junction found.")
                    return True
            elif accept_s8_only:
                outer_confirm_count = (
                    outer_confirm_count + 1
                    if cleared_count >= JUNCTION_CLEAR and right
                    else 0
                )
                if outer_confirm_count >= JUNCTION_CONFIRM:
                    print(
                        f"\n{label}: return junction found by S8 "
                        "(S1 is ignored for this task)."
                    )
                    return True
            else:
                # The BUTTON spur joins the main line at a T. In reverse the
                # sensor bar can see just one side of that junction. Require
                # the centre line and a supported outer branch for two frames.
                # A broad straight stripe is not a junction. Also allow at
                # least half a second of actual reversing before detection.
                values = state.get("normalized", [])
                center_black_count = (
                    sum(
                        values[index] >= BUTTON_RETURN_CENTER_BLACK_MIN
                        for index in (2, 3, 4, 5)
                    )
                    if len(values) == 8 else 0
                )
                main_junction = (
                    candidate
                    and center_black_count
                    >= BUTTON_RETURN_CENTER_SENSOR_COUNT
                )
                outer_confirm_count = (
                    outer_confirm_count + 1
                    if cleared_count >= JUNCTION_CLEAR
                    and main_junction
                    and now - start >= BUTTON_RETURN_JUNCTION_DELAY
                    else 0
                )
                if outer_confirm_count >= BUTTON_RETURN_JUNCTION_CONFIRM:
                    print(f"\n{label}: return junction found.")
                    return True

            # S1 has no road to detect on the CUBE1 return.  Do not let it
            # affect either junction detection or the approach speed.
            candidate_visible = right if accept_s8_only else left != right
            if candidate_visible:
                if candidate_since is None:
                    candidate_since = now
            elif accept_s8_only or (not left and not right):
                candidate_since = None

            speed = (
                RETURN_SLOW_SPEED if candidate_since is not None else RETURN_SPEED
            )
            if not differential_reverse(state, speed, reverse_controller):
                print(f"\n{label}: line lost too long while reversing.")
                return False

            if now - last_status >= 1.0:
                position = state["position"]
                position_text = (
                    "None" if position is None else f"{position:+.3f}"
                )
                print(
                    f'\r\033[2K{label} BACK pos={position_text} '
                    f'S1={int(left)} S8={int(right)} '
                    f'armed={int(cleared_count >= JUNCTION_CLEAR)}',
                    end="", flush=True,
                )
                last_status = now
            time.sleep(LOOP_DELAY)

        print(f"\n{label}: reverse timeout.")
        return False

    finally:
        stop()


def button_task(calibration, detector, sensor, bus, offset, board):
    print("\nBUTTON: execute shared L90.")
    ok, left_sign = turn_90(calibration, bus, offset, "L")
    if not ok or left_sign is None:
        print("\nBUTTON: initial L90/gyro check failed.")
        return False

    if not reacquire_line(calibration):
        return False

    if not button_bumper_approach(calibration, sensor):
        return False

    if not button_return(
        calibration,
        detector,
        bus,
        offset,
        left_sign,
        line_loss_timeout=BUTTON_REVERSE_LINE_LOSS_TIMEOUT,
    ):
        return False

    print("\nB: execute shared R90.")
    ok, _ = turn_90(calibration, bus, offset, "R")
    return ok and reacquire_line(calibration)


def cube1_arm_action(board):
    """Two exact requested poses; no chassis centering between R90 and arm."""
    from roboarm import get_shared_arm
    from button_arm import _execute_plan
    import arm_web

    stop()
    arm, _ = get_shared_arm(board)
    arm_web.arm = arm
    try:
        # Pre-position with automatic / free wrist, open gripper throughout.
        arm.write(gripper=0)
        first, free_pitch = arm_web.plan_auto_path({
            "x": CUBE1_STAGE_X_MM, "y": CUBE1_STAGE_Y_MM,
            "z": CUBE1_STAGE_Z_MM, "orientation_mode": "auto",
        })
        print(f"CUBE1: stage 1 (250,0,100), free pitch {free_pitch:.1f}, gripper open.")
        _execute_plan(arm, first, arm_web.STEP_DELAY)
        arm.write(gripper=0)

        # Final posture locks the gripper's absolute pitch at -90 degrees.
        second = arm_web.plan_target_pitch_path({
            "x": CUBE1_X_MM, "y": CUBE1_Y_MM, "z": CUBE1_Z_MM,
            "tool_pitch": CUBE1_TOOL_PITCH_DEG,
        })
        print("CUBE1: stage 2 (200,0,-180), pitch -90, gripper open.")
        _execute_plan(arm, second, arm_web.STEP_DELAY)
        arm.write(gripper=0)
        return True
    except Exception as exc:
        print(f"CUBE1: arm motion failed: {exc}")
        stop()
        return False


def cube1_fold_arm(board, reason="task endpoint reached"):
    """Raise and fold the arm after either CUBE1 completion condition."""
    from arm_web import START_ANGLES, AUTO_MAX_STEP_DEG, STEP_DELAY
    from button_arm import _return_home
    from roboarm import get_shared_arm

    stop()
    print(f"\nCUBE1: {reason}; raising and folding arm home.")
    try:
        arm, newly_attached = get_shared_arm(board)
        if newly_attached:
            arm.write(**START_ANGLES)
        else:
            _return_home(
                arm, START_ANGLES, AUTO_MAX_STEP_DEG, STEP_DELAY
            )
        # _return_home also applies the configured home gripper angle.
        print("CUBE1: arm is home; ready for the final right turn.")
        return True
    except Exception as exc:
        print(f"CUBE1: could not fold arm home: {exc}")
        return False
    finally:
        stop()


def cube1_create_indicator():
    """Create the RGB indicator using this robot's swapped red/blue wiring."""
    return create_robot_rgb_indicator()


def create_robot_rgb_indicator():
    """Create the physical RGB LED with its measured red/blue wire swap."""
    import led
    import robot_pin_config as pins

    rgb = led.RGBLED(pins.LED_BLUE, pins.LED_GREEN, pins.LED_RED)
    # gpiozero already initializes to off; repeat it explicitly before use so
    # no task can inherit a previous color while ownership changes.
    rgb.off()
    return rgb


class Cube1Signal:
    """Non-blocking, continuously refreshed RGB status; no background GPIO writes."""
    def __init__(self, led):
        self.led = led
        self.mode = "off"
        self.last_visible = None

    def set_mode(self, mode):
        if mode not in ("off", "green_blink", "green", "red_blink"):
            raise ValueError(mode)
        if mode != self.mode:
            self.mode = mode
            self.last_visible = None
        self.tick()

    def tick(self, now=None):
        if now is None:
            now = time.monotonic()
        show = self.mode == "green" or (
            self.mode.endswith("_blink")
            and int(now / CUBE1_BLINK_HALF_PERIOD) % 2 == 0
        )
        color = "green" if self.mode.startswith("green") else "red"
        state = (show, color if show else None)
        if state != self.last_visible:
            if show:
                getattr(self.led, color)()
            else:
                self.led.off()
            self.last_visible = state

    def close(self):
        self.led.off()
        self.led.close()


def cube1_transport(calibration, detector, cube_sensor, indicator):
    """Perform CUBE1 search, carry, and bounded pit emergency.

    Return "drop" when cube disappears AFTER the second crossing, or
    "emergency" only when the PIT is confirmed AFTER that crossing has cleared.
    A full-black junction must never be mistaken for the pit.
    """
    deadline = time.monotonic() + CUBE1_DROP_TIMEOUT
    phase = "search"
    clear_count = 0
    junction_armed = False
    crossing_count = 0
    second_crossing_seen = False
    second_crossing_cleared = False
    post_cross_clear_count = 0
    present_count = 0
    missing_count = 0
    pit_count = 0
    emergency_deadline = None
    last_status = 0.0

    def enter_emergency(reason, now):
        nonlocal phase, emergency_deadline
        if phase != "emergency":
            print(f"\\nCUBE1 EMERGENCY: {reason}; RED blink until pit after second crossing.")
            phase = "emergency"
            indicator.set_mode("red_blink")
            # Give time to reach the second crossing before the pit timer.
            emergency_deadline = now + CUBE1_DROP_TIMEOUT

    try:
        while time.monotonic() < deadline or phase == "emergency":
            now = time.monotonic()
            if phase == "emergency" and now >= emergency_deadline:
                print("\\nCUBE1: emergency timeout before confirmed pit; STOP.")
                return None
            state = line_tracker.update(calibration)
            values = state.get("normalized", ())
            if not line_tracker.all_channels_fresh() or len(values) != 8:
                print("\\nCUBE1: ADC stale; STOP.")
                return None
            indicator.tick(now)

            left, right, candidate = detector.read(state)
            if not left and not right:
                clear_count += 1
            else:
                clear_count = 0
            if clear_count >= JUNCTION_CLEAR:
                junction_armed = True

            at_junction = junction_armed and candidate
            crossing_count = crossing_count + 1 if at_junction else 0
            if crossing_count >= JUNCTION_CONFIRM and not second_crossing_seen:
                second_crossing_seen = True
                post_cross_clear_count = 0
                print("\\nCUBE1: second crossing reached.")
                if phase == "search":
                    enter_emergency("cube NOT found before second crossing", now)
                elif phase == "carry":
                    phase = "past_junction"
                    missing_count = 0
                    indicator.set_mode("green")
                    print("\\nCUBE1: GREEN solid, keep moving until cube disappears.")

            # Suppress pit recognition across the width of the crossing.
            # Demand confirmed non-crossing after the second crossing,
            # and only then start checking the all-black condition.
            if second_crossing_seen and not second_crossing_cleared:
                if (
                    not candidate and not left and not right
                    and not all(v >= CUBE1_PIT_ALL_BLACK_MIN for v in values)
                ):
                    post_cross_clear_count += 1
                    if post_cross_clear_count >= JUNCTION_CLEAR:
                        second_crossing_cleared = True
                        if phase == "emergency":
                            emergency_deadline = now + CUBE1_EMERGENCY_TIMEOUT
                        pit_count = 0
                        print("\\nCUBE1: second crossing cleared; PIT detection armed.")
                else:
                    post_cross_clear_count = 0

            cube_visible = cube_sensor.detected()
            if cube_visible is None:
                print("\\nCUBE1: cube IR sensor stale; STOP.")
                return None

            if phase == "search" and not second_crossing_seen:
                if cube_visible:
                    present_count += 1
                    if present_count >= CUBE1_SENSOR_CONFIRM:
                        phase = "carry"
                        missing_count = 0
                        indicator.set_mode("green_blink")
                        print("\\nCUBE1: cube found; GREEN blinking.")
                else:
                    present_count = 0
            elif phase == "carry" and not second_crossing_seen:
                if cube_visible:
                    missing_count = 0
                else:
                    missing_count += 1
                    if missing_count >= CUBE1_DROP_CONFIRM_FRAMES:
                        enter_emergency("cube lost BEFORE second crossing", now)
            elif phase == "past_junction":
                if not cube_visible:
                    missing_count += 1
                    if missing_count >= CUBE1_DROP_CONFIRM_FRAMES:
                        print("\\nCUBE1: cube dropped AFTER second crossing.")
                        return "drop"
                else:
                    missing_count = 0

            if phase == "emergency" and second_crossing_cleared:
                black = all(value >= CUBE1_PIT_ALL_BLACK_MIN for value in values)
                pit_count = pit_count + 1 if black else 0
                if pit_count >= CUBE1_PIT_CONFIRM_FRAMES:
                    print("\\nCUBE1: all 8 black AFTER crossing; reversing to find line.")
                    return "emergency"

            if state.get("position") is None:
                # Emergency may traverse a short gap; ordinary phases stop
                # rather than accelerating blind at full PWM.
                if phase != "emergency":
                    print("\\nCUBE1: line lost before drop; STOP.")
                    return None
                forward_drive_straight()
            else:
                differential_drive(state)
            if now - last_status >= 0.5:
                print(f"\\rCUBE1 phase={phase} cube={int(cube_visible)} "
                      f"crossing_passed={int(second_crossing_cleared)}",
                      end="", flush=True)
                last_status = now
            time.sleep(LOOP_DELAY)

        print("\\nCUBE1: search/transport timeout; STOP.")
        return None
    finally:
        stop()


def cube1_emergency_reverse_to_line(calibration):
    """In emergency, move backwards until the 8-channel line becomes visible."""
    deadline = time.monotonic() + ROUTE_LINE_REVERSE_TIMEOUT
    seen = 0
    try:
        while time.monotonic() < deadline:
            if not line_tracker.all_channels_fresh():
                return False
            state = line_tracker.update(calibration)
            if state.get("position") is not None:
                seen += 1
                if seen >= ROUTE_LINE_REVERSE_CONFIRM:
                    print("\nCUBE1: line recovered behind pit.")
                    return True
            else:
                seen = 0
            line_follower.reverse_straight(ROUTE_LINE_REVERSE_SPEED)
            time.sleep(ROUTE_LOOP_DELAY)
        print("\nCUBE1: could not find line behind pit.")
        return False
    finally:
        stop()


def cube1_task(calibration, detector, bus, offset, board, distance_sensor):
    """R90 -> arm stages -> cube transport or emergency -> reverse -> R90."""
    from firefighter import Firefighter

    indicator = None
    stop()
    try:
        print("\nCUBE1: execute shared R90 with calibrated approach.")
        ok, left_sign = turn_90(calibration, bus, offset, "R")
        if not ok or left_sign is None:
            print("\nCUBE1: initial R90 failed.")
            return False
        # Explicitly NO reacquire_line() here. Move arm immediately.
        stop()
        if not cube1_arm_action(board):
            return False

        cube_sensor = Firefighter(board)
        cube_sensor.initialize()
        cube_sensor.wait_for_sample(after=time.monotonic())
        indicator = Cube1Signal(cube1_create_indicator())
        status = cube1_transport(calibration, detector, cube_sensor, indicator)
        if status not in ("drop", "emergency"):
            return False

        # Fold arm before reverse; it is no longer holding the cube.
        if not cube1_fold_arm(board, reason=status):
            return False
        if status == "emergency":
            if not cube1_emergency_reverse_to_line(calibration):
                return False
        # Return on the line to the same junction. Keep the existing
        # geometry-aware reverse controller with gyro heading protection.
        if not button_return(
            calibration, detector, bus, offset, left_sign,
            label="CUBE1", accept_s8_only=True, line_loss_timeout=None,
        ):
            return False
        print("\nCUBE1: final shared R90.")
        ok, _ = turn_90(calibration, bus, offset, "R")
        return ok and reacquire_line(calibration)
    finally:
        stop()
        if indicator is not None:
            indicator.close()


def firefighter_reverse_until_line(calibration):
    """Emergency slow reverse after the flame approach loses the line."""
    stop()
    print(
        "\nF: line lost during flame search; emergency stop. "
        "Reversing slowly to find it."
    )
    deadline = time.monotonic() + FIRE_LINE_RECOVERY_TIMEOUT
    confirmed = 0
    gray_reported = False
    try:
        while time.monotonic() < deadline:
            state = line_tracker.update(calibration)
            values = state.get("normalized", [])
            raw = state.get("raw", [])
            strong_sensors = [
                index for index, value in enumerate(values)
                if value >= FIRE_LINE_BLACKNESS_MIN
            ]
            if calibration is not None and len(raw) == len(values) == 8:
                strong_sensors = [
                    index for index in strong_sensors
                    if abs(raw[index] - calibration["black"][index])
                    < abs(raw[index] - calibration["gray"][index])
                ]
                if not strong_sensors and not gray_reported and any(
                    abs(raw[index] - calibration["gray"][index])
                    < abs(raw[index] - calibration["black"][index])
                    and abs(raw[index] - calibration["gray"][index])
                    < abs(raw[index] - calibration["white"][index])
                    for index in range(8)
                ):
                    print("\nF: gray line passed while reversing.")
                    gray_reported = True
            black_line = (
                state.get("position") is not None
                and len(values) == 8
                and bool(strong_sensors)
            )
            if black_line:
                confirmed += 1
                if confirmed >= FIRE_LINE_RECOVERY_CONFIRM:
                    stop()
                    print(
                        "\nF: line found; switching to the normal "
                        "line-following return."
                    )
                    return True
            else:
                confirmed = 0

            line_follower.reverse_straight(FIRE_LINE_RECOVERY_SPEED)
            time.sleep(LOOP_DELAY)

        print("\nF: line recovery timeout; robot remains stopped.")
        return False
    finally:
        stop()


def firefighter_task(
    calibration, detector, bus, offset, board, camera_stream=None,
):
    """Approach fire; spray on confirmation or once at the line endpoint."""
    from firefighter import (
        Firefighter,
        firefighting_pose,
        scan_for_confirmed_flame,
    )
    from roboarm import get_shared_arm
    from arm_web import START_ANGLES, AUTO_MAX_STEP_DEG, STEP_DELAY
    from button_arm import _return_home

    fire = Firefighter(board)
    stop()
    try:
        pose = {**START_ANGLES, **firefighting_pose()}
        fire.initialize()
        fire.wait_for_sample()
        print("\nF: turn LEFT.")
        ok, left_sign = pivot(bus, offset, "L")
        if not ok or left_sign is None:
            return False

        # No forward movement until arm positioning completes.
        stop()
        arm, newly_attached = get_shared_arm(board)
        if newly_attached:
            arm.write(**START_ANGLES)
        else:
            _return_home(arm, START_ANGLES, AUTO_MAX_STEP_DEG, STEP_DELAY)
        time.sleep(0.6)
        # Open before moving into the firefighting pose so the gripper cannot
        # obstruct the water stream while the arm is being positioned.
        arm.open_gripper()
        _return_home(arm, pose, AUTO_MAX_STEP_DEG, STEP_DELAY)
        time.sleep(0.3)

        # Servo movement can outlast the flame sample taken before the turn.
        # Wait for a new report before the robot starts approaching the flame.
        fire.wait_for_sample(after=time.monotonic())
        deadline = time.monotonic() + FIRE_APPROACH_TIMEOUT
        line_end_fallback = False
        while time.monotonic() < deadline:
            flame_detected = fire.detected()
            if flame_detected is None:
                # Never keep driving while the flame input is stale.
                stop()
                fire.wait_for_sample()
                continue

            if flame_detected:
                stop()
                confirmed = fire.confirm_flame()
                if not confirmed:
                    confirmed = scan_for_confirmed_flame(
                        fire, arm, pose["base"]
                    )
                if confirmed:
                    if camera_stream is not None:
                        camera_stream.save_latest_task_frame(
                            "FIREFIGHTER: flame confirmed"
                        )
                    print(
                        "\nF: flame confirmed for 0.5 seconds; advancing "
                        f"for {FIRE_FINAL_ADVANCE_SECONDS:g} seconds."
                    )
                    if not follow_for(
                        calibration,
                        FIRE_FINAL_ADVANCE_SECONDS,
                        speed=FIRE_APPROACH_SPEED,
                        max_speed=FIRE_APPROACH_MAX_SPEED,
                    ):
                        return False
                    print("\nF: final advance complete. Water ON.")
                    fire.spray()
                    # Discard reports taken during spraying. Keep the input
                    # subscribed: there is no sensor power-control pin.
                    fire.wait_for_sample(after=time.monotonic())
                    if fire.confirm_flame():
                        print("\nF: flame remains. Second and final 5 s spray.")
                        fire.spray()
                    break
                # A transient signal and an empty scan must not trigger water.
                continue
            state = line_tracker.update(calibration)
            if state["position"] is None:
                stop()
                if camera_stream is not None:
                    camera_stream.save_latest_task_frame(
                        "FIREFIGHTER: fallback spray at line end"
                    )
                print(
                    "\nF: end of line reached without confirmed flame; "
                    "fallback water spray ON for 5 seconds."
                )
                fire.spray()
                if not firefighter_reverse_until_line(calibration):
                    return False
                line_end_fallback = True
                break
            differential_drive(
                state,
                speed=FIRE_APPROACH_SPEED,
                max_speed=FIRE_APPROACH_MAX_SPEED,
            )
            time.sleep(LOOP_DELAY)
        else:
            print("\nF: flame approach timeout.")
            return False

        if line_end_fallback:
            print("\nF: fallback spray complete; returning along the line.")

        stop()
        _return_home(arm, START_ANGLES, AUTO_MAX_STEP_DEG, STEP_DELAY)
        if not button_return(
            calibration,
            detector,
            bus,
            offset,
            left_sign,
            label="FIREFIGHTER",
            accept_any_junction=True,
            max_return_time=FIRE_RETURN_MAX_TIME,
            line_loss_timeout=BUTTON_REVERSE_LINE_LOSS_TIMEOUT,
        ):
            return False
        print("\nF: execute shared R90.")
        ok, _ = turn_90(calibration, bus, offset, "R")
        return ok and reacquire_line(calibration)
    finally:
        try:
            stop()
        finally:
            fire.off()


def charging_task():
    """Honor the 11-second charging pause even if its visual LED fails.

    A display-only failure must not cancel the route. Motors remain stopped
    throughout the pause; navigation still validates the subsequent turn.
    """
    stop()
    rgb = None
    try:
        try:
            rgb = create_robot_rgb_indicator()
        except Exception as exc:
            print(f"\nC: RGB unavailable ({exc}); continuing timed charge without LED.")

        start = time.monotonic()
        red_end = start + CHARGE_RED_SECONDS
        park_end = start + CHARGE_SECONDS

        def indicate(method):
            nonlocal rgb
            if rgb is None:
                return
            try:
                getattr(rgb, method)()
            except Exception as exc:
                print(f"\nC: RGB indicator failed ({exc}); charging timer continues.")
                try:
                    rgb.close()
                except Exception:
                    pass
                rgb = None

        print("\nC: charging pause; red blink for 9 seconds if LED available.")
        while time.monotonic() < red_end:
            indicate("red")
            time.sleep(max(0.0, min(
                CHARGE_BLINK_HALF_PERIOD, red_end - time.monotonic()
            )))
            indicate("off")
            time.sleep(max(0.0, min(
                CHARGE_BLINK_HALF_PERIOD, red_end - time.monotonic()
            )))
        print("\nC: remaining 2 seconds; green LED if available.")
        indicate("green")
        time.sleep(max(0.0, park_end - time.monotonic()))
        print("\nC: charging pause complete; proceeding to validated right turn.")
        return True
    finally:
        if rgb is not None:
            try:
                rgb.off()
                rgb.close()
            except Exception:
                pass
        stop()


def color_marker_task(calibration, board, bus, offset, camera_stream=None):
    """Run COLOR's R90, then pose the arm and detect a rotated-frame color."""
    from color_marker_detector import run_color_marker_task
    from roboarm import get_shared_arm

    stop()
    marker = None
    try:
        print("\nCOLOR: execute shared R90 with calibrated crossing.")
        ok, _ = turn_90(calibration, bus, offset, "R")
        if not ok or not reacquire_line(calibration, recover_after_line_loss=True):
            return False

        arm, _ = get_shared_arm(board)
        print(
            "\nCOLOR: moving arm to base=90, shoulder=160, "
            "elbow=180, wrist=0, gripper=0."
        )
        arm.write(**COLOR_ARM_ANGLES)
        # Claim and initialize the LED while it is still off. Confirmation
        # then changes only its color; no GPIO object is created between a
        # RED/BLUE decision and the visible indication.
        global COLOR_LED_HOLD
        if COLOR_LED_HOLD is not None:
            try:
                COLOR_LED_HOLD.off()
                COLOR_LED_HOLD.close()
            except Exception:
                pass
        COLOR_LED_HOLD = create_robot_rgb_indicator()

        def light_confirmed_color(match):
            getattr(COLOR_LED_HOLD, match["color"])()
            print(
                f"COLOR: {match['color'].upper()} indicator ON; "
                f"holding for {COLOR_MARKER_HOLD_SECONDS:g} second; "
                "LED map R=GPIO22 G=GPIO27 B=GPIO17."
            )

        marker = run_color_marker_task(
            timeout_s=COLOR_MARKER_TIMEOUT,
            hold_seconds=COLOR_MARKER_HOLD_SECONDS,
            min_object_area=COLOR_MARKER_MIN_AREA,
            stable_frames=COLOR_MARKER_STABLE_FRAMES,
            control_led=False,
            confirmation_callback=light_confirmed_color,
            preview_port=None if camera_stream is not None else 8000,
            camera_stream=camera_stream,
            task_frame_callback=(
                None if camera_stream is None else
                lambda frame, match: camera_stream.save_task_frame(
                    frame,
                    f"COLOR detected {match['color'].upper()}",
                )
            ),
        )
        return marker is not None
    except Exception as exc:
        print(f"\nCOLOR: camera/LED error: {exc}")
        return False
    finally:
        if marker is None and COLOR_LED_HOLD is not None:
            try:
                COLOR_LED_HOLD.off()
                COLOR_LED_HOLD.close()
            except Exception:
                pass
            COLOR_LED_HOLD = None
        stop()


def load_route():
    """Read the user-edited ROUTE list before starting the robot."""
    if not ROUTE:
        raise ValueError(
            "ROUTE is empty. Add your junction commands to the ROUTE list "
            "in main.py before starting main.py."
        )

    route = []
    for index, command in enumerate(ROUTE, 1):
        if not isinstance(command, str):
            raise ValueError(f"ROUTE entry {index} is not a string.")
        cmd = command.strip().upper()
        if cmd not in ALLOWED_ROUTE_COMMANDS:
            raise ValueError(
                f"Invalid ROUTE entry #{index}: {command!r}. "
                "Allowed: L, R, L90, R90, L90SLOW, R90SLOW, S, BUTTON, CHARGING, "
                "SLOW, FIREFIGHTER, CUBE1, COLOR, Q."
            )
        if cmd == "Q" and index != len(ROUTE):
            raise ValueError("Q must be the last ROUTE entry.")
        route.append(cmd)
    return route


def recover_stm32_connection(old_board, calibration):
    """Reopen the STM32 and resubscribe ADC after an actual stale-data fault.

    Only called from ordinary route following, not during an arm task. If
    recovery fails, the caller aborts; never blindly arm the old connection.
    """
    try:
        stop()
    except Exception:
        pass
    line_tracker.shutdown()
    try:
        old_board.shutdown()
    except Exception as exc:
        print(f"STM32 RECOVERY: old connection close error: {exc}")
    reset_stm32()
    board = None
    try:
        board = telemetrix.Telemetrix(com_port=SERIAL_PORT)
        # Preserve the differential2.py order during recovery as well.
        line_tracker.initialize(board)
        if not line_tracker.wait_for_callbacks(timeout=6.0):
            raise RuntimeError("Not all eight ADC callbacks returned")
        deadline = time.monotonic() + 2.0
        while not line_tracker.all_channels_fresh():
            if time.monotonic() >= deadline:
                raise RuntimeError("ADC remained stale after reconnect")
            time.sleep(ROUTE_LOOP_DELAY)
        if line_tracker.update(calibration).get("position") is None:
            raise RuntimeError("No track line after reconnect; stop safely")
        motors.initialize(board)
        motors.motor_stop()
        # Reset shared STM32-backed servo handles before reattaching.
        import roboarm
        roboarm._shared_board = None
        roboarm._shared_arm = None
        activate_arm_home(board)
        motors.motor_stop()
        print("STM32 RECOVERY: all eight ADC channels fresh and line visible.")
        return board
    except Exception:
        try:
            motors.motor_stop()
        except Exception:
            pass
        if board is not None:
            try:
                board.shutdown()
            except Exception:
                pass
        raise


def main():
    global COLOR_LED_HOLD
    board = bus = sensor = None
    camera_stream = None
    offset_90 = None
    motors_on = tracker_on = False
    adc_stale_since = None
    stm32_recovery_count = 0
    STM32_RECOVERY_LIMIT = 2
    STM32_RECOVERY_DELAY_SECONDS = 0.75

    try:
        route = load_route()
        print(f"MAIN BUILD: {PROGRAM_BUILD}")
        print(f"Automatic ROUTE loaded: {route}")
        print(
            f"Resetting STM32 through Raspberry Pi BCM{STM32_RESET_GPIO}..."
        )
        reset_stm32()
        # Match the verified differential2.py / ADC.py startup sequence:
        # subscribe to every analog input BEFORE motor or servo pin commands.
        # No output pins are configured and no movement is commanded until
        # all eight ADC channels have returned valid, fresh samples.
        board = telemetrix.Telemetrix(com_port=SERIAL_PORT)
        line_tracker.initialize(board)
        tracker_on = True

        calibration = line_tracker.load_calibration()
        if calibration is None:
            print("Calibration missing or sensor pins changed; run python calibrate.py.")
            return

        print("Waiting for all eight ADC callbacks...")
        if not line_tracker.wait_for_callbacks(timeout=6.0):
            received = line_tracker.callback_status()
            missing = [i + 1 for i, ready in enumerate(received) if not ready]
            startup_state = line_tracker.update(calibration)
            print(
                "ERROR: ADC callbacks missing for "
                f"S={missing}; motors remain stopped."
            )
            print(f"Latest RAW values: {startup_state['raw']}")
            return

        freshness = FreshFrames()
        ready_deadline = time.monotonic() + 2.0
        while not line_tracker.all_channels_fresh():
            if time.monotonic() >= ready_deadline:
                print("ERROR: ADC channels are stale or out of sync; motors remain uninitialized.")
                print(f"Latest RAW values: {line_tracker.update(calibration)['raw']}")
                return
            time.sleep(ROUTE_LOOP_DELAY)

        startup_state = line_tracker.update(calibration)
        freshness.accept(startup_state, time.monotonic())
        print(f"All eight ADC channels ready: {startup_state['raw']}")
        print(f"Initial line position: {startup_state['position']}")

        # As in differential2.py, configure motor outputs only after the ADC
        # subscriptions have succeeded. Keep chassis PWM at zero while servos
        # are attached and moved to the starting pose.
        motors.initialize(board)
        motors_on = True
        motors.motor_stop()
        activate_arm_home(board)
        motors.motor_stop()

        # The MJPEG preview is optional; a streaming-only failure should
        # not abort line navigation. COLOR can open its own capture later.
        try:
            from color_marker_detector import ContinuousCamera
            camera_stream = ContinuousCamera(port=8000)
            camera_stream.start()
        except Exception as exc:
            print(f"\nCAMERA STREAM unavailable ({exc}); route continues without preview.")
            if camera_stream is not None:
                try:
                    camera_stream.stop()
                except Exception:
                    pass
            camera_stream = None

        print("Starting route; line tracker will self-center the robot.")

        detector = JunctionDetector()
        print(
            f"Differential2 ready: target {TARGET_POSITION}, "
            f"base PWM {BASE_SPEED}, max PWM {MAX_SPEED}. "
            "Automatic route active."
        )

        route_index = 0
        junction_count = 0
        candidate_count = 0
        clear_count = 0
        latched = False
        adc_paused = False
        route_line_loss_count = 0
        last_position_status = 0.0

        while True:
            state = line_tracker.update(calibration)
            if not freshness.accept(state, time.monotonic()):
                if not line_tracker.all_channels_fresh():
                    if adc_stale_since is None:
                        stop()
                        adc_stale_since = time.monotonic()
                        print(
                            "\nADC STALE: motor stop, waiting briefly before reboot. "
                            f"RAW={state['raw']}"
                        )
                    adc_paused = True
                    candidate_count = 0
                    clear_count = 0
                    route_line_loss_count = 0
                    if time.monotonic() - adc_stale_since >= STM32_RECOVERY_DELAY_SECONDS:
                        if stm32_recovery_count >= STM32_RECOVERY_LIMIT:
                            raise RuntimeError(
                                "ADC stale after maximum STM32 recovery attempts"
                            )
                        stm32_recovery_count += 1
                        print(f"STM32 RECOVERY {stm32_recovery_count}/{STM32_RECOVERY_LIMIT}: "
                              f"route remains at step {route_index+1}/{len(route)}.")
                        board = recover_stm32_connection(board, calibration)
                        freshness = FreshFrames()
                        fresh_state = line_tracker.update(calibration)
                        freshness.accept(fresh_state, time.monotonic())
                        detector = JunctionDetector()
                        # The just-recovered crossing is not a new command.
                        latched = fresh_state.get("crossing") is not None
                        clear_count = 0
                        candidate_count = 0
                        route_line_loss_count = 0
                        adc_paused = False
                        adc_stale_since = None
                        offset_90 = None
                        # Servo outputs were reinitialized on the new board.
                time.sleep(ROUTE_JUNCTION_POLL_DELAY)
                continue
            if adc_paused:
                print("\nADC ready: all 8 channels fresh; route resumed.")
                adc_paused = False
            adc_stale_since = None
            position = state["position"]
            route_line_loss_count = (
                route_line_loss_count + 1 if position is None else 0
            )
            if route_line_loss_count >= ROUTE_LINE_LOSS_CONFIRM:
                stop()
                if not reverse_to_line_on_route(calibration, freshness):
                    break
                route_line_loss_count = 0
                candidate_count = 0
                clear_count = 0
                detector = JunctionDetector()
                continue
            # Use the same crossing result shown by show_line_position.py.
            # The task routines retain their own detector and maneuver rules.
            crossing = state["crossing"]
            left = crossing in ("LEFT", "BOTH")
            right = crossing in ("RIGHT", "BOTH")
            candidate = crossing is not None
            if not left and not right:
                clear_count += 1
            else:
                clear_count = 0

            if clear_count >= ROUTE_JUNCTION_CLEAR:
                latched = False

            if not latched and candidate:
                candidate_count += 1
            else:
                candidate_count = 0

            if (
                candidate_count >= JUNCTION_CONFIRM
                or (
                    not latched and candidate
                    and strong_route_junction(state)
                )
            ):
                stop()
                latched = True
                candidate_count = 0
                junction_count += 1

                # COLOR's result stays lit while driving and is cleared only
                # when the following junction has actually been confirmed.
                if COLOR_LED_HOLD is not None:
                    try:
                        COLOR_LED_HOLD.off()
                        COLOR_LED_HOLD.close()
                    except Exception:
                        pass
                    COLOR_LED_HOLD = None
                    print("\nCOLOR: next junction reached; indicator OFF.")

                if route_index >= len(route):
                    print("\nROUTE finished. Robot stopped.")
                    break
                cmd = route[route_index]
                route_index += 1
                print(
                    f"\nJUNCTION #{junction_count}: S1={int(left)} "
                    f"S8={int(right)}; ROUTE {route_index}/{len(route)} -> {cmd}"
                )
                if cmd == "Q":
                    print("ROUTE: stop command.")
                    break

                if cmd == "COLOR":
                    if bus is None:
                        bus = smbus.SMBus(I2C_BUS)
                        gyro_setup(bus)
                    if offset_90 is None:
                        offset_90 = gyro_calibrate(bus, settle_seconds=0.05)
                    ok = color_marker_task(
                        calibration, board, bus, offset_90, camera_stream
                    )
                elif cmd == "SLOW":
                    # Do not call follow_for(): junction detection must remain
                    # active because the following junction is very close.
                    ok = True
                elif cmd in (
                    "L", "R", "L90", "R90", "L90SLOW", "R90SLOW",
                    "BUTTON", "CHARGING",
                    "FIREFIGHTER", "CUBE1",
                ):
                    # Gyro stays unloaded until a maneuver requires it.
                    if bus is None:
                        bus = smbus.SMBus(I2C_BUS)
                        gyro_setup(bus)
                    if cmd in (
                        "L90", "R90", "L90SLOW", "R90SLOW", "BUTTON"
                    ):
                        if offset_90 is None:
                            offset_90 = gyro_calibrate(bus, settle_seconds=0.05)
                        offset = offset_90
                    else:
                        offset = gyro_calibrate(bus)

                    if cmd == "BUTTON":
                        if sensor is None:
                            sensor = VL53L0XSensor()
                            sensor.initialize()
                        ok = button_task(
                            calibration, detector, sensor, bus, offset, board
                        )
                    elif cmd == "FIREFIGHTER":
                        ok = firefighter_task(
                            calibration, detector, bus, offset, board,
                            camera_stream,
                        )
                    elif cmd == "CUBE1":
                        if sensor is None:
                            sensor = VL53L0XSensor()
                            sensor.initialize()
                        ok = cube1_task(
                            calibration, detector, bus, offset, board, sensor
                        )
                    elif cmd == "CHARGING":
                        ok = charging_task()
                        if ok:
                            # Same 80-degree R turn and differential reacquire.
                            ok, _ = pivot(bus, offset, "R")
                            ok = ok and reacquire_line(calibration)
                    elif cmd in ("L90", "R90", "L90SLOW", "R90SLOW"):
                        slow_turn = cmd in ("L90SLOW", "R90SLOW")
                        ok, _ = turn_90(
                            calibration, bus, offset, cmd[0], slow=slow_turn
                        )
                        if ok and slow_turn:
                            print(
                                f"\nROUTE: slow {cmd}; "
                                f"reversing for "
                                f"{SLOW_90_REVERSE_SECONDS:g} seconds."
                            )
                            reverse_for_short_turn(
                                SLOW_90_REVERSE_SECONDS,
                                SLOW_90_REVERSE_SPEED,
                            )
                        ok = ok and reacquire_line(
                            calibration, recover_after_line_loss=True
                        )
                    else:
                        ok, _ = pivot(bus, offset, cmd)
                        ok = ok and reacquire_line(
                            calibration, recover_after_line_loss=True
                        )
                else:
                    ok = follow_for(calibration, 0.5)

                if not ok:
                    print("\nManeuver failed. Robot stopped.")
                    break
                route_line_loss_count = 0
                if route_index >= len(route):
                    stop()
                    print("\nAll ROUTE commands completed. Robot stopped.")
                    break

                continue

            # Exact differential2 PID, fixed 255 base, no acceleration gate.
            if position is None:
                stop()
            else:
                differential_drive(state)

            # Navigation still reads sensors every loop; only the terminal
            # status is throttled. Keep it short enough not to wrap a TTY.
            now = time.monotonic()
            if now - last_position_status >= 1.0:
                status = (
                    "NO LINE: paused"
                    if position is None else f"LINE pos={position:+.3f}"
                )
                print(
                    f"\r\033[2K{status} S1={int(left)} S8={int(right)} "
                    f"CROSSING={crossing or 'None'}",
                    end="", flush=True,
                )
                last_position_status = now

            time.sleep(ROUTE_JUNCTION_POLL_DELAY)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as exc:
        import traceback
        print(f"\nERROR: {exc}")
        traceback.print_exc()  # A Python TypeError is NOT proof of an STM32 fault.
    finally:
        if camera_stream is not None:
            try:
                camera_stream.stop()
            except Exception as exc:
                print(f"CAMERA cleanup error: {exc}")
        if COLOR_LED_HOLD is not None:
            try:
                COLOR_LED_HOLD.off()
                COLOR_LED_HOLD.close()
            except Exception:
                pass
            COLOR_LED_HOLD = None
        if motors_on:
            try:
                stop()
            except Exception:
                pass
        if sensor is not None:
            try:
                sensor.close()
            except Exception:
                pass
        if tracker_on:
            try:
                line_tracker.shutdown()
            except Exception:
                pass
        if board is not None:
            try:
                board.shutdown()
            except (Exception, KeyboardInterrupt):
                pass
        if bus is not None:
            try:
                bus.close()
            except Exception:
                pass
        print("\nRobot stopped.")


if __name__ == "__main__":
    main()

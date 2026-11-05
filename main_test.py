#!/usr/bin/env python3
"""ArmRobotics: differential2 line following + junction commands + button task."""

import math
import time

import smbus
from telemetrix import telemetrix

import line_tracker
import motors
from sensors.vl53l0x_sensor import VL53L0XSensor

# Previously tested differential2 line-following speeds.
SERIAL_PORT = "/dev/serial0"
BASE_SPEED = 180
MAX_SPEED = 255
MIN_SPEED = -80
MANEUVER_SPEED = 100
MANEUVER_MAX_SPEED = 170
KP = 50.0
KD = 2.0
TARGET_POSITION = line_tracker.CENTER_POSITION
LOOP_DELAY = 0.02

# Line-strength limits reused by the button positioning task.
START_CENTER_BLACK_MIN = 0.30
START_OUTER_BLACK_MAX = 0.70

# Junction recognition is separate from line tracking.
BLACK_ON = 0.85
BLACK_OFF = 0.65
JUNCTION_SIDE_SUPPORT_ON = 0.55
JUNCTION_SIDE_SUPPORT_OFF = 0.35
JUNCTION_CENTER_MIN = 0.50
JUNCTION_CONFIRM = 2
JUNCTION_CLEAR = 3
POST_TURN_JUNCTION_IGNORE_SECONDS = 0.2
POST_JUNCTION_ACCEL_SECONDS = 0.5
STRAIGHT_MAX_SPEED = 255
STRAIGHT_ACCEL_RATE = 180.0
STRAIGHT_DECEL_RATE = 300.0
STRAIGHT_CONFIRM_SECONDS = 0.25
STRAIGHT_POSITION_TOLERANCE = 0.35
STRAIGHT_DERROR_LIMIT = 8.0

# After any gyro turn, differential2 centers the robot again.
REACQUIRE_TIMEOUT = 1.2
REACQUIRE_TOLERANCE = 0.75
REACQUIRE_CONFIRM = 2

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
TURN_PWM_FAST = 255
TURN_PWM_SLOW = 90
# main_old's uncalibrated alignment PWM and 0.50 s travel.
# Keep this independent of BASE_SPEED and MANEUVER_SPEED.
TURN_90_FORWARD_SPEED = 80
TURN_90_FORWARD_MAX_SPEED = 120
TURN_90_FORWARD_TIME = 0.5
TURN_TOLERANCE_DEG = 8.0
TURN_CORRECTION_START_DEG = 4.0

BUTTON_DISTANCE_CM = 21.5
BUTTON_DISTANCE_TOLERANCE_CM = 0.5
BUTTON_SLOWDOWN_CM = 28.0
BUTTON_MIN_CLEARANCE_CM = 18.0
BUTTON_CENTER_TOLERANCE = 0.25
BUTTON_ADJUST_SPEED = 70
BUTTON_ADJUST_SECONDS = 0.08
BUTTON_SETTLE_SECONDS = 0.20
# Button tip target (mm, arm_web coordinate system). Arm moves in two stages.
BUTTON_X_MM = 350.0
BUTTON_Y_MM = 0.0
BUTTON_Z_MM = -50.0

# Charging: 9 s red blink, then 2 s solid green; total 11 s before R.
CHARGE_SECONDS = 11.0
CHARGE_RED_SECONDS = 9.0
CHARGE_BLINK_HALF_PERIOD = 0.5
BUTTON_MAX_TIME = 18.0
BUTTON_CONFIRM = 3
BUTTON_CLEAR_TIMEOUT = 3.0

RETURN_SPEED = 150
RETURN_SLOW_SPEED = 95
RETURN_GYRO_KP = 2.0
RETURN_MAX_DELTA = 85
RETURN_MAX_TIME = 15.0
RETURN_MAX_ANGLE = 30.0
RETURN_RECOVERY_ANGLE = 12.0
RETURN_RECOVERY_SAFE = 6.0
RETURN_RECOVERY_TIMEOUT = 1.5

RETURN_FORWARD_TIME = 0.3
BUTTON_FINAL_RIGHT_ANGLE = 80.0

FIRE_APPROACH_SPEED = 70
FIRE_APPROACH_MAX_SPEED = 100
FIRE_APPROACH_TIMEOUT = 12.0
FIRE_FINAL_RIGHT_ANGLE = 300.0

# Automatic route: one entry for each detected junction, in order.
# L=left, R=right, S=straight, B=press button, C=charging, F=firefighter, Q=stop.
# Example: ROUTE = ["L", "S", "B", "R", "C", "F", "Q"]
# The commented entries below are placeholders: edit them before launching.
ROUTE = [
    "B", 
    "L",
    "R",
    "S",
    "R",
    "L",
    "C", 
    "S",
    "R",
    "S", 
    "L",
    "R",
    "R",
    "L90",
    "Q",

]
ALLOWED_ROUTE_COMMANDS = {
    "L", "R", "L90", "R90", "S", "B", "C", "F", "Q"
}


line_follower = line_tracker.LineTrackingEngine(
    base_speed=BASE_SPEED,
    max_speed=MAX_SPEED,
    min_speed=MIN_SPEED,
    kp=KP,
    kd=KD,
    target_position=TARGET_POSITION,
)


def stop():
    line_follower.stop()


def differential_drive(state, speed=BASE_SPEED, max_speed=MAX_SPEED):
    """Compatibility wrapper around the separate line-following engine."""
    return line_follower.drive(state, speed=speed, max_speed=max_speed)


def post_junction_drive_limits(elapsed):
    """Ramp both base PWM and the wheel PWM ceiling after a maneuver."""
    fraction = max(0.0, min(1.0, elapsed / POST_JUNCTION_ACCEL_SECONDS))
    start_speed = min(MANEUVER_SPEED, BASE_SPEED)
    start_max = min(MANEUVER_MAX_SPEED, MAX_SPEED)
    speed = start_speed + (BASE_SPEED - start_speed) * fraction
    max_speed = start_max + (MAX_SPEED - start_max) * fraction
    return speed, max_speed


def update_straight_speed(state, left, right, candidate, controller, now):
    """Raise speed only while the line is stably centered and branch-free."""
    position = state.get("position") if state else None
    previous_time = controller["previous_time"]
    dt = 0.0 if previous_time is None else max(0.0, now - previous_time)
    controller["previous_time"] = now

    if position is None:
        controller["current_speed"] = BASE_SPEED
        controller["previous_error"] = None
        controller["straight_since"] = None
        return BASE_SPEED, MAX_SPEED

    error = position - TARGET_POSITION
    previous_error = controller["previous_error"]
    derivative = 0.0
    if previous_error is not None and dt > 0.0:
        derivative = (error - previous_error) / dt
    controller["previous_error"] = error

    stable = (
        not left and not right and not candidate
        and abs(error) <= STRAIGHT_POSITION_TOLERANCE
        and abs(derivative) <= STRAIGHT_DERROR_LIMIT
    )
    if stable:
        if controller["straight_since"] is None:
            controller["straight_since"] = now
        if now - controller["straight_since"] >= STRAIGHT_CONFIRM_SECONDS:
            controller["current_speed"] = min(
                STRAIGHT_MAX_SPEED,
                controller["current_speed"] + STRAIGHT_ACCEL_RATE * dt,
            )
    else:
        controller["straight_since"] = None
        controller["current_speed"] = max(
            BASE_SPEED,
            controller["current_speed"] - STRAIGHT_DECEL_RATE * dt,
        )

    return controller["current_speed"], min(
        STRAIGHT_MAX_SPEED,
        max(MANEUVER_MAX_SPEED, controller["current_speed"]),
    )


def differential_reverse(state, speed, heading_correction=0.0):
    """Compatibility wrapper for reverse line following."""
    return line_follower.reverse(
        state,
        speed,
        heading_correction=heading_correction,
        max_delta=RETURN_MAX_DELTA,
    )


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

    def read(self):
        now = time.monotonic()
        rate = gyro_rate(self.bus, self.offset)
        self.angle += self.sign * rate * max(0.0, now - self.last)
        self.last = now
        return self.angle


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
        sign = (
            (1 if signed > 0 else -1)
            if left and abs(signed) > 20
            else None
        )
        return True, sign
    finally:
        stop()
        time.sleep(PIVOT_SETTLE_SECONDS)


def pivot_90(bus, offset, direction, angle_target=TURN_90_ANGLE):
    """Controlled pivot with the tested 90-degree correction logic."""
    if direction not in ("L", "R"):
        raise ValueError("Turn direction must be L or R")

    stop()
    left = direction == "L"
    start = previous = time.monotonic()
    angle = signed = 0.0
    gyro_sign = None
    candidate_sign = None
    direction_samples = 0
    reverse_since = None
    last_rate = 0.0
    lead = 5.6425 if left else 6.125

    try:
        while True:
            now = time.monotonic()
            rate = gyro_rate(bus, offset)
            dt = max(0.0, min(now - previous, 0.1))
            previous = now
            if not math.isfinite(rate) or abs(rate) > 1000:
                raise RuntimeError("GYRO_TURN: invalid angular rate")
            # Ignore pre-drive/coasting samples. Learn polarity only from a
            # consistent response after the pivot motors have started.
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
            last_rate = abs(rate)
            stop_angle = angle_target - max(
                lead, min(12.0, last_rate * 0.04)
            )
            if angle >= stop_angle:
                break
            if now - start >= GYRO_TIMEOUT or (now - start > 0.7 and angle < 2):
                print(f"\nGYRO_TURN timeout/no motion at {angle:.1f} degrees")
                return False, None
            pwm = TURN_PWM_SLOW if angle >= angle_target - 25 else TURN_PWM_FAST
            motors.set_speeds(-pwm if left else pwm, pwm if left else -pwm)
            time.sleep(0.005)

        stop()
        settle_until = time.monotonic() + 0.05
        while time.monotonic() < settle_until:
            now = time.monotonic()
            signed += gyro_rate(bus, offset) * max(
                0.0, min(now - previous, 0.1)
            )
            previous = now
            time.sleep(0.005)

        final_angle = abs(signed)
        if (angle_target >= 45
                and TURN_CORRECTION_START_DEG < angle_target - final_angle <= 20):
            correction_deadline = time.monotonic() + 0.2
            while time.monotonic() < correction_deadline:
                now = time.monotonic()
                signed += gyro_rate(bus, offset) * max(
                    0.0, min(now - previous, 0.1)
                )
                previous = now
                if abs(signed) >= angle_target - lead:
                    break
                motors.set_speeds(-70 if left else 70, 70 if left else -70)
                time.sleep(0.005)
            stop()
            final_angle = abs(signed)

        tolerance = min(TURN_TOLERANCE_DEG, angle_target * 0.2)
        print(
            f"\nGYRO_TURN {direction}: target={angle_target:.1f}, "
            f"gyro={final_angle:.1f}, error={final_angle-angle_target:+.1f}"
        )
        sign = (
            (1 if signed > 0 else -1)
            if left and abs(signed) > 5
            else None
        )
        return abs(final_angle - angle_target) <= tolerance, sign

    finally:
        stop()
        time.sleep(0.02)


def reacquire_line(calibration):
    """Use differential2 driving after EVERY turn, including edge-only line."""
    deadline = time.monotonic() + REACQUIRE_TIMEOUT
    stable = 0

    try:
        while time.monotonic() < deadline:
            state = line_tracker.update(calibration)
            position = state["position"]

            if position is None:
                line_follower.drive_straight(
                    speed=MANEUVER_SPEED,
                    max_speed=MANEUVER_MAX_SPEED,
                )
                stable = 0
            else:
                differential_drive(
                    state, speed=MANEUVER_SPEED, max_speed=MANEUVER_MAX_SPEED
                )
                if abs(position - TARGET_POSITION) <= REACQUIRE_TOLERANCE:
                    stable += 1
                    if stable >= REACQUIRE_CONFIRM:
                        return True
                else:
                    stable = 0

            time.sleep(LOOP_DELAY)

        print("\nLine not centered yet; returning control to route tracking.")
        return True

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
                line_follower.drive_straight(
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


def button_position_ready(state, distance_cm):
    """Check distance and use line alignment when it is available."""
    position = state["position"]
    if not (0 < distance_cm):
        return False
    distance_ready = (
        abs(distance_cm - BUTTON_DISTANCE_CM)
        <= BUTTON_DISTANCE_TOLERANCE_CM
    )
    if not distance_ready:
        return False
    if position is None:
        return True
    values = state["normalized"]
    return (
        abs(position - TARGET_POSITION) <= BUTTON_CENTER_TOLERANCE
        and max(values[2:6]) >= START_CENTER_BLACK_MIN
        and max(values[0], values[7]) <= START_OUTER_BLACK_MAX
    )


def button_approach(calibration, detector, sensor):
    """Stop at 21.5 cm, then verify distance and line before moving the arm."""
    start = time.monotonic()
    cleared_count = 0
    left_initial_junction = False
    ready_count = 0

    try:
        while time.monotonic() - start < BUTTON_MAX_TIME:
            state = line_tracker.update(calibration)
            left, right, _ = detector.read(state)

            if not left and not right:
                cleared_count += 1
            else:
                cleared_count = 0

            if cleared_count >= JUNCTION_CLEAR:
                left_initial_junction = True

            if (
                not left_initial_junction
                and time.monotonic() - start > BUTTON_CLEAR_TIMEOUT
            ):
                print("\nB: initial junction did not clear.")
                return False

            distance_cm = sensor.read_distance_mm() / 10.0
            if distance_cm <= 0:
                stop()
                print("\nB: invalid distance reading; arm remains off.")
                return False

            if (state["position"] is None
                    and (not left_initial_junction
                         or distance_cm > BUTTON_SLOWDOWN_CM)):
                ready_count = 0
                speed = (
                    BUTTON_ADJUST_SPEED
                    if distance_cm <= BUTTON_SLOWDOWN_CM
                    else MANEUVER_SPEED
                )
                line_follower.drive_straight(
                    speed=speed,
                    max_speed=MANEUVER_MAX_SPEED,
                )
                time.sleep(LOOP_DELAY)
                continue

            if not left_initial_junction:
                # The range sensor may already see the button immediately
                # after the turn. Clear the junction before fine positioning.
                if distance_cm < BUTTON_MIN_CLEARANCE_CM:
                    stop()
                    print(
                        f"\nB: only {distance_cm:.1f} cm before leaving "
                        "junction; arm remains off."
                    )
                    return False
                speed = (
                    BUTTON_ADJUST_SPEED if distance_cm <= BUTTON_SLOWDOWN_CM
                    else MANEUVER_SPEED
                )
                differential_drive(
                    state, speed=speed,
                    max_speed=MANEUVER_MAX_SPEED
                )
                time.sleep(LOOP_DELAY)
                continue

            if distance_cm > BUTTON_SLOWDOWN_CM:
                ready_count = 0
                differential_drive(
                    state, speed=MANEUVER_SPEED,
                    max_speed=MANEUVER_MAX_SPEED
                )
                time.sleep(LOOP_DELAY)
                continue

            # All checks below use fresh readings after the motors have stopped.
            stop()
            time.sleep(BUTTON_SETTLE_SECONDS)
            state = line_tracker.update(calibration)
            distance_cm = sensor.read_distance_mm() / 10.0
            if distance_cm <= 0:
                print("\nB: distance missing while stopped; arm remains off.")
                return False

            line_text = (
                "---" if state["position"] is None
                else f"{state['position']:+.3f}"
            )
            print(
                f"\rB ALIGN distance={distance_cm:.1f} cm "
                f"line={line_text}",
                end="", flush=True,
            )

            if button_position_ready(state, distance_cm):
                ready_count += 1
                if ready_count >= BUTTON_CONFIRM:
                    print(
                        f"\nB: stopped at {distance_cm:.1f} cm, "
                        f"line position {line_text}. Starting arm."
                    )
                    return True
                continue

            ready_count = 0
            if distance_cm < BUTTON_DISTANCE_CM - BUTTON_DISTANCE_TOLERANCE_CM:
                if state["position"] is None:
                    line_follower.reverse_straight(BUTTON_ADJUST_SPEED)
                else:
                    differential_reverse(state, BUTTON_ADJUST_SPEED)
            else:
                # Also use a short forward steering pulse when the distance is
                # correct but the line is still off center.
                if state["position"] is None:
                    line_follower.drive_straight(
                        speed=BUTTON_ADJUST_SPEED,
                        max_speed=BUTTON_ADJUST_SPEED + 40,
                    )
                else:
                    differential_drive(
                        state, speed=BUTTON_ADJUST_SPEED,
                        max_speed=BUTTON_ADJUST_SPEED + 40,
                    )
            time.sleep(BUTTON_ADJUST_SECONDS)
            stop()

        print("\nB: could not verify 21.5 cm and centered line; arm remains off.")
        return False

    finally:
        stop()


def button_return(calibration, detector, bus, offset, left_sign):
    """Reverse on the differential2 sensor position, with gyro protection."""
    heading = Heading(bus, offset, left_sign)
    start = time.monotonic()
    cleared_count = 0
    candidate_since = None
    recovery_since = None

    try:
        while time.monotonic() - start < RETURN_MAX_TIME:
            state = line_tracker.update(calibration)
            left, right, _ = detector.read(state)
            now = time.monotonic()
            angle = heading.read()

            if not left and not right:
                cleared_count += 1

            # Preserve the original B return-junction rule.
            if cleared_count >= JUNCTION_CLEAR and left and right:
                print("\nB: return junction found (both outer sensors).")
                return True

            if abs(angle) >= RETURN_MAX_ANGLE:
                print("\nB: reverse gyro angle limit reached.")
                return False

            if left != right:
                if candidate_since is None:
                    candidate_since = now
            elif not left and not right:
                candidate_since = None

            if abs(angle) >= RETURN_RECOVERY_ANGLE:
                if recovery_since is None:
                    recovery_since = now
            elif abs(angle) <= RETURN_RECOVERY_SAFE:
                recovery_since = None

            if (
                recovery_since is not None
                and now - recovery_since >= RETURN_RECOVERY_TIMEOUT
            ):
                print("\nB: reverse heading recovery timeout.")
                return False

            if state["position"] is not None:
                speed = (
                    RETURN_SLOW_SPEED
                    if candidate_since is not None or recovery_since is not None
                    else RETURN_SPEED
                )
                differential_reverse(
                    state, speed, RETURN_GYRO_KP * angle
                )
            else:
                speed = (
                    RETURN_SLOW_SPEED
                    if candidate_since is not None or recovery_since is not None
                    else RETURN_SPEED
                )
                line_follower.reverse_straight(
                    speed,
                    heading_correction=RETURN_GYRO_KP * angle,
                    max_delta=RETURN_MAX_DELTA,
                )

            print(
                f'\rB BACK pos={state["position"]} angle={angle:+.1f} '
                f'S1={int(left)} S8={int(right)}',
                end="", flush=True,
            )
            time.sleep(LOOP_DELAY)

        print("\nB: reverse timeout.")
        return False

    finally:
        stop()


def button_task(calibration, detector, sensor, bus, offset, board):
    print("\nB: turn LEFT.")
    ok, left_sign = pivot(bus, offset, "L")
    if not ok or left_sign is None:
        print("\nB: initial turn/gyro check failed.")
        return False

    if not reacquire_line(calibration):
        return False

    if not button_approach(calibration, detector, sensor):
        return False

    # The drive motors stay stopped until the arm maneuver finishes.
    if not button_arm_action(board):
        return False

    if not button_return(calibration, detector, bus, offset, left_sign):
        return False

    print("\nB: move forward for 0.3 seconds.")
    if not follow_for(calibration, RETURN_FORWARD_TIME):
        return False

    print(f"\nB: turn RIGHT by {BUTTON_FINAL_RIGHT_ANGLE:g} degrees.")
    ok, _ = pivot(
        bus, offset, "R", angle_target=BUTTON_FINAL_RIGHT_ANGLE
    )
    return ok and reacquire_line(calibration)


def button_arm_action(board):
    """Move arm to the tested button XYZ, then return home before reversing."""
    from button_arm import move_to_xyz_and_home

    stop()
    print(
        f"\nB: moving arm automatically to X={BUTTON_X_MM:g}, "
        f"Y={BUTTON_Y_MM:g}, Z={BUTTON_Z_MM:g} mm."
    )
    try:
        ok = move_to_xyz_and_home(
            board, BUTTON_X_MM, BUTTON_Y_MM, BUTTON_Z_MM
        )
        if ok:
            print("B: arm returned to home. Reversing to junction.")
        return ok
    except Exception as exc:
        print(f"\nB: arm movement failed: {exc}")
        print("B: drive stopped. Check arm position before continuing.")
        return False
    finally:
        stop()


def firefighter_task(calibration, detector, bus, offset, board):
    """Left turn, stationary arm pose, confirmed flame, at most two sprays."""
    from firefighter import Firefighter, firefighting_pose
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
        _return_home(arm, pose, AUTO_MAX_STEP_DEG, STEP_DELAY)
        time.sleep(0.3)

        # Servo movement can outlast the flame sample taken before the turn.
        # Wait for a new report before the robot starts approaching the flame.
        fire.wait_for_sample(after=time.monotonic())
        deadline = time.monotonic() + FIRE_APPROACH_TIMEOUT
        while time.monotonic() < deadline:
            flame_detected = fire.detected()
            if flame_detected is None:
                # Never keep driving while the flame input is stale.
                stop()
                fire.wait_for_sample()
                continue

            if flame_detected:
                stop()
                if fire.confirm_flame():
                    print("\nF: flame confirmed for 1 second. Water ON.")
                    fire.spray()
                    # Discard reports taken during spraying. Keep the input
                    # subscribed: there is no sensor power-control pin.
                    fire.wait_for_sample(after=time.monotonic())
                    if fire.confirm_flame():
                        print("\nF: flame remains. Second and final 5 s spray.")
                        fire.spray()
                    break
                # A transient signal must not trigger the pump.
                continue
            state = line_tracker.update(calibration)
            if not differential_drive(
                state, speed=FIRE_APPROACH_SPEED,
                max_speed=FIRE_APPROACH_MAX_SPEED,
            ):
                line_follower.drive_straight(
                    speed=FIRE_APPROACH_SPEED,
                    max_speed=FIRE_APPROACH_MAX_SPEED,
                )
            time.sleep(LOOP_DELAY)
        else:
            print("\nF: flame approach timeout.")
            return False

        stop()
        _return_home(arm, START_ANGLES, AUTO_MAX_STEP_DEG, STEP_DELAY)
        if not button_return(calibration, detector, bus, offset, left_sign):
            return False
        if not follow_for(calibration, RETURN_FORWARD_TIME):
            return False
        print(f"\nF: turn RIGHT by {FIRE_FINAL_RIGHT_ANGLE:g} degrees.")
        ok, _ = pivot(bus, offset, "R", angle_target=FIRE_FINAL_RIGHT_ANGLE)
        return ok and reacquire_line(calibration)
    finally:
        try:
            stop()
        finally:
            fire.off()


def charging_task():
    """Stop for 11 s: blink RED for 9 s, GREEN for 2 s, then turn R."""
    stop()
    rgb = None
    try:
        # The existing led.RGBLED handles the robot's active-low RGB pins.
        import led
        import robot_pin_config as pins

        # Physical red/blue LED channels are swapped on this robot:
        # previous RED command lit BLUE. Swap only here, not global pins.
        rgb = led.RGBLED(pins.LED_BLUE, pins.LED_GREEN, pins.LED_RED)
        start = time.monotonic()
        red_end = start + CHARGE_RED_SECONDS
        park_end = start + CHARGE_SECONDS

        print("\nC: robot stopped. Red blinking for 9 seconds.")
        while time.monotonic() < red_end:
            rgb.red()
            time.sleep(
                max(0.0, min(CHARGE_BLINK_HALF_PERIOD,
                             red_end - time.monotonic()))
            )
            rgb.off()
            time.sleep(
                max(0.0, min(CHARGE_BLINK_HALF_PERIOD,
                             red_end - time.monotonic()))
            )

        print("\nC: green ON for remaining 2 seconds.")
        rgb.green()
        time.sleep(max(0.0, park_end - time.monotonic()))
        print("\nC: charging complete; turning RIGHT.")
        return True

    except Exception as exc:
        print(f"\nC: RGB charging error: {exc}")
        print("C: not turning until the light problem is resolved.")
        return False

    finally:
        if rgb is not None:
            try:
                rgb.off()
                rgb.close()
            except Exception:
                pass
        stop()


def load_route():
    """Read the user-edited ROUTE list before starting the robot."""
    if not ROUTE:
        raise ValueError(
            "ROUTE is empty. Add your junction commands to the ROUTE list "
            "in main_ggs_interactive.py before starting."
        )

    route = []
    for index, command in enumerate(ROUTE, 1):
        if not isinstance(command, str):
            raise ValueError(f"ROUTE entry {index} is not a string.")
        cmd = command.strip().upper()
        if cmd not in ALLOWED_ROUTE_COMMANDS:
            raise ValueError(
                f"Invalid ROUTE entry #{index}: {command!r}. "
                "Allowed: L, R, L90, R90, S, B, C, F, Q."
            )
        if cmd == "Q" and index != len(ROUTE):
            raise ValueError("Q must be the last ROUTE entry.")
        route.append(cmd)
    return route


def main():
    board = bus = sensor = None
    offset_90 = None
    motors_on = tracker_on = False

    try:
        route = load_route()
        print(f"Automatic ROUTE loaded: {route}")
        # Use the same connection and ADC subscription order as ADC.py:
        # connect once, then immediately register all eight callbacks.
        board = telemetrix.Telemetrix(com_port=SERIAL_PORT)
        line_tracker.initialize(board)
        tracker_on = True

        calibration = line_tracker.load_calibration()
        if calibration is None:
            print("No calibration found; run python calibrate.py.")
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

        startup_state = line_tracker.update(calibration)
        print(f"All eight ADC channels ready: {startup_state['raw']}")
        print(f"Initial line position: {startup_state['position']}")

        # Configure the motor outputs only after ADC subscriptions are alive.
        motors.initialize(board)
        motors_on = True

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
        junction_ignore_until = 0.0
        acceleration_started = None
        straight_speed = {
            "current_speed": BASE_SPEED,
            "previous_error": None,
            "previous_time": None,
            "straight_since": None,
        }

        while True:
            # EXACT same reading as the working standalone differential2.py.
            state = line_tracker.update(calibration)
            position = state["position"]
            left, right, candidate = detector.read(state)

            if not left and not right:
                clear_count += 1
            else:
                clear_count = 0

            if clear_count >= JUNCTION_CLEAR:
                latched = False

            if (time.monotonic() >= junction_ignore_until
                    and not latched and candidate):
                candidate_count += 1
            else:
                candidate_count = 0

            if candidate_count >= JUNCTION_CONFIRM:
                stop()
                latched = True
                candidate_count = 0
                junction_count += 1

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

                if cmd in ("L", "R", "L90", "R90", "B", "C", "F"):
                    # Gyro stays unloaded until a maneuver requires it.
                    if bus is None:
                        bus = smbus.SMBus(I2C_BUS)
                        gyro_setup(bus)
                    if cmd in ("L90", "R90"):
                        if offset_90 is None:
                            offset_90 = gyro_calibrate(bus, settle_seconds=0.05)
                        offset = offset_90
                    else:
                        offset = gyro_calibrate(bus)

                    if cmd == "B":
                        if sensor is None:
                            sensor = VL53L0XSensor()
                            sensor.initialize()
                        ok = button_task(
                            calibration, detector, sensor, bus, offset, board
                        )
                    elif cmd == "F":
                        ok = firefighter_task(
                            calibration, detector, bus, offset, board
                        )
                    elif cmd == "C":
                        ok = charging_task()
                        if ok:
                            # Same 80-degree R turn and differential reacquire.
                            ok, _ = pivot(bus, offset, "R")
                            ok = ok and reacquire_line(calibration)
                    elif cmd in ("L90", "R90"):
                        ok = follow_for(
                            calibration, TURN_90_FORWARD_TIME,
                            speed=TURN_90_FORWARD_SPEED,
                            max_speed=TURN_90_FORWARD_MAX_SPEED,
                        )
                        if ok:
                            ok, _ = pivot_90(
                                bus,
                                offset,
                                cmd[0],
                                angle_target=TURN_90_ANGLE,
                            )
                        ok = ok and reacquire_line(calibration)
                    else:
                        ok, _ = pivot(bus, offset, cmd)
                        ok = ok and reacquire_line(calibration)
                else:
                    ok = follow_for(calibration, 0.5)

                if not ok:
                    print("\nManeuver failed. Robot stopped.")
                    break
                acceleration_started = time.monotonic()
                straight_speed["current_speed"] = MANEUVER_SPEED
                straight_speed["previous_error"] = None
                straight_speed["previous_time"] = None
                straight_speed["straight_since"] = None
                if cmd in ("L", "R", "L90", "R90", "B", "C", "F"):
                    junction_ignore_until = (
                        time.monotonic() + POST_TURN_JUNCTION_IGNORE_SECONDS
                    )
                    candidate_count = 0
                if route_index >= len(route):
                    stop()
                    print("\nAll ROUTE commands completed. Robot stopped.")
                    break

                continue

            # Only the tested differential2.py decides how to follow the line.
            if position is None:
                stop()
                if acceleration_started is not None:
                    acceleration_started = time.monotonic()
                print(
                    '\rNO LINE: paused; route resumes when line returns. '
                    f'raw={state["raw"]}',
                    end="", flush=True,
                )
            else:
                if acceleration_started is None:
                    speed, max_speed = update_straight_speed(
                        state, left, right, candidate,
                        straight_speed, time.monotonic(),
                    )
                    differential_drive(state, speed=speed, max_speed=max_speed)
                else:
                    elapsed = time.monotonic() - acceleration_started
                    speed, max_speed = post_junction_drive_limits(elapsed)
                    differential_drive(state, speed=speed, max_speed=max_speed)
                    if elapsed >= POST_JUNCTION_ACCEL_SECONDS:
                        acceleration_started = None
                        straight_speed["current_speed"] = speed
                        straight_speed["previous_error"] = None
                        straight_speed["previous_time"] = None
                        straight_speed["straight_since"] = None
                print(
                    f"\rLINE pos={position:+.3f} "
                    f"S1={int(left)} S8={int(right)}",
                    end="", flush=True,
                )

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as exc:
        print(f"\nERROR: {exc}")
    finally:
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
            except Exception:
                pass
        if bus is not None:
            try:
                bus.close()
            except Exception:
                pass
        print("\nRobot stopped.")


if __name__ == "__main__":
    main()

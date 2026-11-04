#!/usr/bin/env python3
"""ArmRobotics: differential2 line following + junction commands + button task."""

import argparse
import math
import time
from pathlib import Path

import smbus
from telemetrix import telemetrix

import line_tracker
import motors
from navigation_core import (FreshFrames, JunctionTracker, LineMonitor,
                             MotionCalibration, JUNCTION_CONFIRM_FRAMES,
                             turn_stop_angle)


# Start from differential2.py's working line-following configuration.
SERIAL_PORT = "/dev/serial0"
BASE_SPEED = 100
CRUISE_SPEED = 160
MAX_SPEED = 170
MIN_SPEED = -80
MANEUVER_SPEED = 100
MANEUVER_MAX_SPEED = 200
KP = 30.0
TARGET_POSITION = 0.08
LOOP_DELAY = 0.04

# Initial line position check; the robot centres at low speed after ADC/gyro readiness.
START_CENTER_TOLERANCE = 0.45
START_CENTER_BLACK_MIN = 0.30
START_OUTER_BLACK_MAX = 0.70

# Junction recognition is separate from line tracking.
BLACK_ON = 0.85
BLACK_OFF = 0.65
JUNCTION_CENTER_MIN = 0.55
JUNCTION_CLEAR = 4

# After any gyro turn, differential2 centers the robot again.
REACQUIRE_TIMEOUT = 3.0
REACQUIRE_TOLERANCE = 0.85
REACQUIRE_CONFIRM = 4
REACQUIRE_SPEED = 80

I2C_BUS = 1
MPU_ADDR = 0x68
GYRO_Z_REG = 0x47
GYRO_SCALE = 131.0
GYRO_TIMEOUT = 5.0
GYRO_CALIBRATION_SETTLE_SECONDS = 0.08
PIVOT_SETTLE_SECONDS = 0.1
TURN_ANGLE = 90.0
TURN_PWM_FAST = 130
TURN_PWM_SLOW = 90
TURN_TOLERANCE_DEG = 8.0
TURN_CORRECTION_START_DEG = 4.0
STARTUP_TIMEOUT_S = 8.0
START_LINE_CONFIRM_FRAMES = 4
START_CENTER_SPEED = 80
START_CENTER_TIMEOUT_S = 4.0
JUNCTION_TIMEOUT_S = 3.0
RECOVERY_TIMEOUT_S = 5.0
RECOVERY_SEARCH_DEG = 12.0
MOTION_CALIBRATION_FILE = Path(__file__).with_name("motion_calibration.json")
# Fallback only until a measured motion_calibration.json exists. This is a
# bounded cautious movement, not a claim that PWM 80 equals a known distance.
UNCALIBRATED_ALIGN_PWM = 80
UNCALIBRATED_ALIGN_SECONDS = 0.50

BUTTON_DISTANCE_CM = 22.0
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
BUTTON_Z_MM = -40.0

# Charging: three seconds red blink, two seconds green; total five seconds.
CHARGE_SECONDS = 5.0
CHARGE_RED_SECONDS = 3.0
CHARGE_BLINK_HALF_PERIOD = 0.5
BUTTON_MAX_TIME = 18.0
BUTTON_CONFIRM = 3
BUTTON_CLEAR_TIMEOUT = 3.0

RETURN_SPEED = 150
RETURN_SLOW_SPEED = 95
RETURN_GYRO_KP = 2.0
RETURN_MAX_DELTA = 85
RETURN_MAX_TIME = 15.0
RETURN_LINE_LOST = 0.4
RETURN_MAX_ANGLE = 30.0
RETURN_RECOVERY_ANGLE = 12.0
RETURN_RECOVERY_SAFE = 6.0
RETURN_RECOVERY_TIMEOUT = 1.5

RETURN_FORWARD_TIME = 0.3
BUTTON_FINAL_RIGHT_ANGLE = 80.0

FIRE_APPROACH_SPEED = 70
FIRE_APPROACH_MAX_SPEED = 100
FIRE_APPROACH_TIMEOUT = 12.0
FIRE_FINAL_RIGHT_ANGLE = 80.0

# Automatic route: one entry for each detected junction, in order.
# L=left, R=right, S=straight, B=press button, C=charging, F=firefighter, Q=stop.
# Example: ROUTE = ["L", "S", "B", "R", "C", "F", "Q"]
# The commented entries below are placeholders: edit them before launching.
ROUTE = [
    "S",
    "L",
    "R",
    "S",
    "R",
    "L",
    "C",
    "S",
    "R",
    "F",
    "L",
    "R",
    "R",
    "L",
    "Q",
    # "S",
    # "S",
    # "L",
    # "L",
    # "R",
    # "L",
    # "R",
    # "Q",
]
ALLOWED_ROUTE_COMMANDS = {"L", "R", "S", "B", "C", "F", "Q"}


def clamp(value, low, high):
    return max(low, min(high, value))


def stop():
    motors.motor_stop()


_task_frames = None


def read_fresh(calibration):
    """Blocking read for legacy B/F helpers; a repeated ADC snapshot is invalid."""
    deadline = time.monotonic() + 0.35
    while time.monotonic() < deadline:
        state = line_tracker.update(calibration)
        if _task_frames.accept(state, time.monotonic()):
            return state
        time.sleep(0.005)
    stop()
    raise RuntimeError("ADC stale during task")


def differential_drive(state, speed=BASE_SPEED, max_speed=MAX_SPEED):
    """The differential2 P-controller, optionally with a maneuver speed cap."""
    if state is None or state["position"] is None:
        stop()
        return False

    position = state["position"]
    error = position - TARGET_POSITION
    correction = KP * error

    left_speed = clamp(speed + correction, MIN_SPEED, max_speed)
    right_speed = clamp(speed - correction, MIN_SPEED, max_speed)

    motors.set_speeds(round(left_speed), round(right_speed))
    return True


def differential_reverse(state, speed, heading_correction=0.0):
    """Same calibrated line position and P correction, adapted for reverse."""
    if state is None or state["position"] is None:
        stop()
        return False

    position = state["position"]
    error = position - TARGET_POSITION
    line_correction = KP * error
    correction = clamp(
        line_correction + heading_correction,
        -RETURN_MAX_DELTA,
        RETURN_MAX_DELTA,
    )

    # Keep both wheels in reverse; a large correction only changes their ratio.
    left_speed = clamp(-speed + correction, -255, -15)
    right_speed = clamp(-speed - correction, -255, -15)
    motors.set_speeds(round(left_speed), round(right_speed))
    return True


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

        self.left_black = self.hysteresis(self.left_black, values[0])
        self.right_black = self.hysteresis(self.right_black, values[7])

        # An outer sensor alone means the robot is off-center, NOT a junction.
        center_present = any(
            values[index] >= JUNCTION_CENTER_MIN
            for index in (2, 3, 4, 5)
        )
        candidate = (
            state["position"] is not None
            and center_present
            and (self.left_black or self.right_black)
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
    identity = bus.read_byte_data(MPU_ADDR, 0x75)
    if identity not in (0x68, 0x69):
        raise RuntimeError(f"MPU-6050 identity invalid: 0x{identity:02x}")


def gyro_calibrate(bus):
    stop()
    print("\nGyroscope calibration: keep the robot still.")
    time.sleep(GYRO_CALIBRATION_SETTLE_SECONDS)
    values = []
    for _ in range(300):
        values.append(gyro_raw(bus))
        time.sleep(0.005)
    mean = sum(values) / len(values)
    deviation = math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))
    if deviation / GYRO_SCALE > 2.0:
        raise RuntimeError("Gyro calibration unstable; keep robot stationary")
    return mean


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


def pivot(bus, offset, direction, angle_target=TURN_ANGLE,
          timeout_s=GYRO_TIMEOUT):
    """Bounded gyro pivot with a direction-specific, provisional coast lead."""
    if direction not in ("L", "R"):
        raise ValueError("Turn direction must be L or R")

    stop()
    left = direction == "L"
    start = previous = time.monotonic()
    angle = signed = 0.0
    gyro_sign = None
    last_rate = 0.0
    lead = 5.6425 if left else 6.125  # observed excess at PWM 70, not a final calibration

    try:
        while True:
            now = time.monotonic()
            rate = gyro_rate(bus, offset)
            dt = max(0.0, min(now - previous, 0.1))
            previous = now
            if not math.isfinite(rate) or abs(rate) > 1000:
                raise RuntimeError("GYRO_TURN: invalid angular rate")
            if gyro_sign is None and abs(rate) >= 3.0:
                gyro_sign = 1 if rate > 0 else -1
            if gyro_sign is not None:
                directed = rate * gyro_sign
                if directed < -5.0:
                    raise RuntimeError("GYRO_TURN: gyro direction reversed")
                angle += max(0.0, directed) * dt
            signed += rate * dt
            last_rate = abs(rate)
            if angle >= turn_stop_angle(direction, angle_target, last_rate):
                break
            if now - start >= timeout_s or (now - start > 0.7 and angle < 2):
                print(f"GYRO_TURN timeout/no motion at {angle:.1f} degrees")
                return False, None
            pwm = TURN_PWM_SLOW if angle >= angle_target - 25 else TURN_PWM_FAST
            motors.set_speeds(-pwm if left else pwm, pwm if left else -pwm)
            time.sleep(0.005)
        stop()
        settle_until = time.monotonic() + 0.25
        while time.monotonic() < settle_until:
            now = time.monotonic()
            rate = gyro_rate(bus, offset)
            signed += rate * max(0.0, min(now - previous, 0.1))
            previous = now
            time.sleep(0.01)
        final_angle = abs(signed)
        if (angle_target >= 45
                and TURN_CORRECTION_START_DEG < angle_target - final_angle <= 20):
            # One bounded correction at the observed minimum working PWM.
            correction_deadline = time.monotonic() + 0.3
            while time.monotonic() < correction_deadline:
                now = time.monotonic()
                rate = gyro_rate(bus, offset)
                signed += rate * max(0.0, min(now - previous, 0.1))
                previous = now
                if abs(signed) >= angle_target - lead:
                    break
                motors.set_speeds(-70 if left else 70, 70 if left else -70)
                time.sleep(0.01)
            stop()
            settle_until = time.monotonic() + 0.2
            while time.monotonic() < settle_until:
                now = time.monotonic()
                signed += gyro_rate(bus, offset) * max(0.0, min(now - previous, 0.1))
                previous = now
                time.sleep(0.01)
            final_angle = abs(signed)
        print(f"GYRO_TURN {direction}: target={angle_target:.1f}, "
              f"gyro={final_angle:.1f}, error={final_angle-angle_target:+.1f}, "
              f"lead={lead:.1f}")
        sign = (
            (1 if signed > 0 else -1)
            if left and abs(signed) > 5
            else None
        )
        tolerance = min(TURN_TOLERANCE_DEG, angle_target * 0.2)
        return abs(final_angle - angle_target) <= tolerance, sign

    finally:
        stop()
        time.sleep(PIVOT_SETTLE_SECONDS)


def reacquire_line(calibration):
    """Lock a central branch over distinct complete ADC frames after a turn."""
    print("PRECISION_ALIGN: searching for planned branch")
    deadline = time.monotonic() + REACQUIRE_TIMEOUT
    stable = 0
    frames = FreshFrames()
    search_forward_until = time.monotonic() + 0.45
    last_report = 0.0

    try:
        while time.monotonic() < deadline:
            state = line_tracker.update(calibration)
            if not frames.accept(state, time.monotonic()):
                stop()
                time.sleep(LOOP_DELAY)
                continue
            position = state["position"]
            now = time.monotonic()
            if now - last_report >= 0.25:
                values = state["normalized"]
                center_level = max(values[2:6]) if len(values) >= 6 else 0.0
                outer_level = max(values[0], values[7]) if len(values) >= 8 else 0.0
                print(
                    "PRECISION_ALIGN: "
                    f"pos={position!s} target={TARGET_POSITION:+.2f} "
                    f"center={center_level:.2f} outer={outer_level:.2f} "
                    f"stable={stable}/{REACQUIRE_CONFIRM} "
                    f"raw={state.get('raw', '?')}"
                )
                last_report = now

            line_visible = (position is not None
                            and max(state["normalized"][2:6]) >= START_CENTER_BLACK_MIN)
            centered = (line_visible
                        and abs(position - TARGET_POSITION) <= REACQUIRE_TOLERANCE)
            if not line_visible:
                # The sensor bar is ahead of the axle. Immediately after a
                # turn it can be over the white gap before the exit branch.
                # A short, low-speed straight search lets the bar reach that
                # branch while keeping a hard overall timeout.
                if time.monotonic() < search_forward_until:
                    motors.set_speeds(70, 70)
                else:
                    stop()
                stable = 0
            else:
                # Keep a bounded low-speed search moving while the line is
                # visible. The outer sensors may still be over the junction
                # immediately after a turn; rejecting that frame entirely
                # left the robot stationary and unable to reacquire the exit.
                differential_drive(
                    state, speed=REACQUIRE_SPEED,
                    max_speed=REACQUIRE_SPEED + 40
                )
                if centered:
                    stable += 1
                    if stable >= REACQUIRE_CONFIRM:
                        print("LINE_LOCK: four fresh central frames")
                        return True
                else:
                    stable = 0

            time.sleep(LOOP_DELAY)

        print("\nDifferential2: could not re-center after turn.")
        return False

    finally:
        stop()


def follow_for(calibration, seconds):
    """Timed forward movement: unchanged differential2 line-following."""
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            state = read_fresh(calibration)
            if state["position"] is None:
                stop()
                print("\nLine missing during timed forward movement.")
                return False
            differential_drive(
                state, speed=MANEUVER_SPEED, max_speed=MANEUVER_MAX_SPEED
            )
            time.sleep(LOOP_DELAY)
        return True
    finally:
        stop()
        time.sleep(0.15)


def button_position_ready(state, distance_cm):
    """Check the stopped robot's distance and lateral line position."""
    position = state["position"]
    if position is None or not (0 < distance_cm):
        return False
    values = state["normalized"]
    return (
        abs(distance_cm - BUTTON_DISTANCE_CM) <= BUTTON_DISTANCE_TOLERANCE_CM
        and abs(position - TARGET_POSITION) <= BUTTON_CENTER_TOLERANCE
        and max(values[2:6]) >= START_CENTER_BLACK_MIN
        and max(values[0], values[7]) <= START_OUTER_BLACK_MAX
    )


def button_approach(calibration, detector, sensor):
    """Stop at corrected 220 mm, then verify line before moving the arm."""
    start = time.monotonic()
    cleared_count = 0
    left_initial_junction = False
    ready_count = 0
    missing_since = None

    try:
        while time.monotonic() - start < BUTTON_MAX_TIME:
            state = read_fresh(calibration)
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

            if state["position"] is None:
                stop()
                ready_count = 0
                if missing_since is None:
                    missing_since = time.monotonic()
                elif time.monotonic() - missing_since >= 0.5:
                    print("\nB: line lost on approach.")
                    return False
                time.sleep(LOOP_DELAY)
                continue
            missing_since = None

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
            state = read_fresh(calibration)
            distance_cm = sensor.read_distance_mm() / 10.0
            if distance_cm <= 0 or state["position"] is None:
                print("\nB: distance or line missing while stopped; arm remains off.")
                return False

            print(
                f"\rB ALIGN distance={distance_cm:.1f} cm "
                f"line={state['position']:+.3f}",
                end="", flush=True,
            )

            if button_position_ready(state, distance_cm):
                ready_count += 1
                if ready_count >= BUTTON_CONFIRM:
                    print(
                        f"\nB: stopped at {distance_cm:.1f} cm, "
                        f"line position {state['position']:+.3f}. Starting arm."
                    )
                    return True
                continue

            ready_count = 0
            if distance_cm < BUTTON_DISTANCE_CM - BUTTON_DISTANCE_TOLERANCE_CM:
                differential_reverse(state, BUTTON_ADJUST_SPEED)
            else:
                # Also use a short forward steering pulse when the distance is
                # correct but the line is still off center.
                differential_drive(
                    state, speed=BUTTON_ADJUST_SPEED,
                    max_speed=BUTTON_ADJUST_SPEED + 40,
                )
            time.sleep(BUTTON_ADJUST_SECONDS)
            stop()

        print("\nB: could not verify 22 cm and centered line; arm remains off.")
        return False

    finally:
        stop()


def button_return(calibration, detector, bus, offset, left_sign):
    """Reverse on the differential2 sensor position, with gyro protection."""
    print("REVERSE_TRACKING: returning to junction")
    heading = Heading(bus, offset, left_sign)
    start = time.monotonic()
    last_line = start
    cleared_count = 0
    junction_cleared = False
    both_count = 0
    last_report = 0.0
    candidate_since = None
    recovery_since = None

    try:
        while time.monotonic() - start < RETURN_MAX_TIME:
            state = read_fresh(calibration)
            left, right, _ = detector.read(state)
            now = time.monotonic()
            angle = heading.read()

            if not left and not right:
                cleared_count += 1
                if cleared_count >= JUNCTION_CLEAR:
                    junction_cleared = True
            else:
                cleared_count = 0

            # Preserve the original B return-junction rule.
            both_count = both_count + 1 if junction_cleared and left and right else 0
            if both_count >= JUNCTION_CONFIRM_FRAMES:
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
                last_line = now
                speed = (
                    RETURN_SLOW_SPEED
                    if candidate_since is not None or recovery_since is not None
                    else RETURN_SPEED
                )
                differential_reverse(
                    state, speed, RETURN_GYRO_KP * angle
                )
            else:
                stop()
                if now - last_line >= RETURN_LINE_LOST:
                    print("\nB: line lost during reverse.")
                    return False

            if now - last_report >= 1.0:
                print(f'REVERSE_TRACKING: pos={state["position"]} '
                      f'angle={angle:+.1f} S1={int(left)} S8={int(right)}')
                last_report = now
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
            state = read_fresh(calibration)
            if not differential_drive(
                state, speed=FIRE_APPROACH_SPEED,
                max_speed=FIRE_APPROACH_MAX_SPEED,
            ):
                print("\nF: line lost on approach.")
                return False
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
    """Stop for 5 s: blink RED for 3 s, GREEN for 2 s, then turn R."""
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

        print("\nC: robot stopped. Red blinking for 3 seconds.")
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
            "in main.py before starting."
        )

    route = []
    for index, command in enumerate(ROUTE, 1):
        if not isinstance(command, str):
            raise ValueError(f"ROUTE entry {index} is not a string.")
        cmd = command.strip().upper()
        if cmd not in ALLOWED_ROUTE_COMMANDS:
            raise ValueError(
                f"Invalid ROUTE entry #{index}: {command!r}. "
                "Allowed: L, R, S, B, C, F, Q."
            )
        if cmd == "Q" and index != len(ROUTE):
            raise ValueError("Q must be the last ROUTE entry.")
        route.append(cmd)
    return route


def startup_alignment_status(state):
    """Check starting line position, not physical heading of the robot.

    Eight sensors arranged in one row can locate the line sideways but
    cannot tell if the whole robot is pointing at an angle to the track.
    """
    position = state["position"]
    if position is None:
        return False, "Line not detected. Place robot over the black line."

    values = state["normalized"]
    center_black = max(values[2:6])
    if center_black < START_CENTER_BLACK_MIN:
        return False, "Center sensors do not see a clear line."

    if max(values[0], values[7]) > START_OUTER_BLACK_MAX:
        return False, "Outer sensor sees black; start on a straight section."

    offset = position - TARGET_POSITION
    if abs(offset) > START_CENTER_TOLERANCE:
        side = "right" if offset > 0 else "left"
        return (
            False,
            f"Line is {side} of center (position={position:+.3f}, "
            f"target={TARGET_POSITION:+.3f}). Reposition robot by hand.",
        )

    return True, f"Centered: position={position:+.3f}, offset={offset:+.3f}."


def wait_for_adc(calibration, frames):
    """Keep drive disarmed until every ADC callback supplies coherent frames."""
    deadline = time.monotonic() + STARTUP_TIMEOUT_S
    good = 0
    while time.monotonic() < deadline:
        state = line_tracker.update(calibration)
        if frames.accept(state, time.monotonic()):
            good += 1
            if good >= START_LINE_CONFIRM_FRAMES:
                return state
        time.sleep(LOOP_DELAY)
    ids = state.get("sample_ids", [0] * 8)
    missing = [str(index + 1) for index, value in enumerate(ids) if not value]
    raise RuntimeError(
        "STARTUP_CHECK: ADC channels not ready; "
        f"missing callbacks S={','.join(missing) or 'none'}; ids={ids}"
    )


def initial_centering(calibration, frames):
    """Use the existing P steering at low speed, only while line is visible."""
    deadline = time.monotonic() + START_CENTER_TIMEOUT_S
    good = 0
    while time.monotonic() < deadline:
        state = line_tracker.update(calibration)
        if not frames.accept(state, time.monotonic()):
            stop()
            time.sleep(LOOP_DELAY)
            continue
        values = state["normalized"]
        if (state["position"] is None
                or (values[0] >= BLACK_ON and values[7] >= BLACK_ON)
                or sum(value >= BLACK_OFF for value in values) > 4):
            stop()
            raise RuntimeError("INITIAL_CENTERING: no unambiguous straight line")
        aligned, _ = startup_alignment_status(state)
        good = good + 1 if aligned else 0
        if good >= START_LINE_CONFIRM_FRAMES:
            stop()
            print("INITIAL_CENTERING complete")
            return
        differential_drive(state, speed=START_CENTER_SPEED, max_speed=START_CENTER_SPEED + 40)
        time.sleep(LOOP_DELAY)
    stop()
    raise RuntimeError("INITIAL_CENTERING timeout")


def recover_line(calibration, frames, bus, offset, last_error):
    """Bounded search around the last heading; never accept an outer-only patch."""
    stop()
    if bus is None:
        return False
    confirmed = 0
    for _ in range(REACQUIRE_CONFIRM):
        state = line_tracker.update(calibration)
        if frames.accept(state, time.monotonic()) and (
                state["position"] is not None
                and abs(state["position"] - TARGET_POSITION) < REACQUIRE_TOLERANCE
                and max(state["normalized"][2:6]) >= START_CENTER_BLACK_MIN):
            confirmed += 1
        else:
            confirmed = 0
        time.sleep(LOOP_DELAY)
    if confirmed >= REACQUIRE_CONFIRM:
        return True
    preferred = "L" if last_error < 0 else "R"
    other = "R" if preferred == "L" else "L"
    deadline = time.monotonic() + RECOVERY_TIMEOUT_S
    for direction in (preferred, other):
        if time.monotonic() >= deadline:
            break
        ok, _ = pivot(bus, offset, direction,
                      angle_target=RECOVERY_SEARCH_DEG, timeout_s=1.5)
        if not ok:
            return False
        confirmed = 0
        while time.monotonic() < deadline:
            state = line_tracker.update(calibration)
            if frames.accept(state, time.monotonic()):
                valid = (state["position"] is not None
                         and abs(state["position"] - TARGET_POSITION) < REACQUIRE_TOLERANCE
                         and max(state["normalized"][2:6]) >= START_CENTER_BLACK_MIN
                         and max(state["normalized"][0], state["normalized"][7]) < BLACK_ON)
                confirmed = confirmed + 1 if valid else 0
                if confirmed >= REACQUIRE_CONFIRM:
                    stop()
                    return True
                if not valid:
                    break
            time.sleep(LOOP_DELAY)
        # Return to the starting orientation before looking the other way.
        if direction == preferred:
            restored, _ = pivot(bus, offset, other,
                                angle_target=RECOVERY_SEARCH_DEG,
                                timeout_s=1.5)
            if not restored:
                return False
    stop()
    return False


def align_axle(calibration, frames, motion, bus, offset,
               allow_no_line=False):
    """Advance from bar trailing edge until axle reaches its measured centre."""
    duration = motion.duration_after_trailing_edge(MANEUVER_SPEED)
    uncalibrated = duration is None or not 0 < duration <= 2.0
    if uncalibrated:
        duration = UNCALIBRATED_ALIGN_SECONDS
        print(
            "AXLE_ALIGNMENT: motion profile missing; using cautious "
            f"PWM {UNCALIBRATED_ALIGN_PWM} for {duration:.2f}s. "
            "Distance accuracy is unverified."
        )
    else:
        print(f"AXLE_ALIGNMENT: measured duration {duration:.3f}s")
    started = time.monotonic()
    last_fresh = started
    heading = Heading(bus, offset, 1)
    try:
        while time.monotonic() - started < duration:
            state = line_tracker.update(calibration)
            now = time.monotonic()
            if frames.accept(state, now):
                last_fresh = now
                angle = heading.read()
                if not math.isfinite(angle) or abs(angle) > 8:
                    raise RuntimeError("AXLE_ALIGNMENT: gyro heading exceeded 8 degrees")
                if state["position"] is None and not allow_no_line:
                    raise RuntimeError("AXLE_ALIGNMENT: line lost")
                if state["position"] is None:
                    speed = UNCALIBRATED_ALIGN_PWM if uncalibrated else MANEUVER_SPEED
                    motors.set_speeds(speed, speed)
                else:
                    speed = UNCALIBRATED_ALIGN_PWM if uncalibrated else MANEUVER_SPEED
                    differential_drive(state, speed=speed,
                                       max_speed=speed + 40)
            elif now - last_fresh > 0.35:
                raise RuntimeError("AXLE_ALIGNMENT: ADC stale")
            time.sleep(LOOP_DELAY)
    finally:
        stop()


def main(argv=None):
    global _task_frames
    parser = argparse.ArgumentParser(description="ArmRobotics route controller")
    parser.add_argument("--diagnostic", action="store_true",
                        help="check ADC and gyro without enabling motors")
    args = parser.parse_args(argv)
    board = bus = sensor = None
    motors_on = tracker_on = False
    state_name = "STARTUP_CHECK"
    status = 0
    try:
        route = load_route()
        board = telemetrix.Telemetrix(com_port=SERIAL_PORT)
        motors.initialize(board)
        motors_on = True
        motors.enable_guard(line_tracker.all_channels_fresh)
        line_tracker.initialize(board)
        tracker_on = True
        calibration = line_tracker.load_calibration()
        if calibration is None:
            raise RuntimeError("STARTUP_CHECK: missing or invalid line calibration")
        frames = FreshFrames()
        _task_frames = frames
        first = wait_for_adc(calibration, frames)
        bus = smbus.SMBus(I2C_BUS)
        gyro_setup(bus)
        offset = gyro_calibrate(bus)
        if first["position"] is None:
            raise RuntimeError("STARTUP_CHECK: no starting line")
        if args.diagnostic:
            print(f"STARTUP_CHECK passed; raw={first['raw']}; "
                  f"position={first['position']:+.3f}; gyro bias={offset:.2f}; motors OFF")
            return
        motion = MotionCalibration.load(MOTION_CALIBRATION_FILE)
        print("STARTUP_CHECK passed; INITIAL_CENTERING")
        motors.arm()
        state_name = "INITIAL_CENTERING"
        initial_centering(calibration, frames)
        state_name = "FOLLOW_LINE"
        detector = JunctionTracker()
        line_monitor = LineMonitor()
        route_index = 0
        pending_kind = None
        junction_since = None
        last_error = 0.0
        last_left_pwm = last_right_pwm = 0
        commanded_speed = START_CENTER_SPEED
        heading = Heading(bus, offset, 1)
        gyro_angle = 0.0
        last_gyro_check = 0.0
        last_fresh = time.monotonic()
        last_report = 0.0
        print(f"FOLLOW_LINE: route={route}; target={TARGET_POSITION:+.2f}")
        while True:
            state = line_tracker.update(calibration)
            now = time.monotonic()
            if not frames.accept(state, now):
                if now - last_fresh > 0.35:
                    raise RuntimeError(f"{state_name}: ADC data stale or incomplete")
                time.sleep(LOOP_DELAY)
                continue
            last_fresh = now
            event, resolved_kind = detector.observe(state, now)
            if detector.count and not detector.latched and pending_kind is None:
                state_name = "JUNCTION_CANDIDATE"
            elif pending_kind is None and state_name == "JUNCTION_CANDIDATE":
                state_name = "FOLLOW_LINE"
            if event and event != "EXIT":
                pending_kind = event
                junction_since = now
                state_name = "JUNCTION_CONFIRM"
                print(f"JUNCTION_CONFIRM: {event}")
            if pending_kind and now - junction_since > JUNCTION_TIMEOUT_S:
                raise RuntimeError("JUNCTION_CONFIRM: bar did not clear")
            if event == "EXIT" and pending_kind:
                stop()
                state_name = "CENTER_DETECTED"
                if route_index >= len(route):
                    raise RuntimeError("ROUTE exhausted at junction")
                cmd = route[route_index]
                route_index += 1
                print(f"CENTER_DETECTED: {resolved_kind}; ROUTE {route_index}/{len(route)} -> {cmd}")
                pending_kind = None
                junction_since = None
                if cmd == "Q":
                    print("SAFE_STOP: route complete")
                    break
                available = {
                    "L": ("LEFT", "T", "CROSS"),
                    "B": ("LEFT", "T", "CROSS"),
                    "F": ("LEFT", "T", "CROSS"),
                    "R": ("RIGHT", "T", "CROSS"),
                    "C": ("RIGHT", "T", "CROSS"),
                }
                if cmd in available and resolved_kind not in available[cmd]:
                    raise RuntimeError(f"ROUTE {cmd}: branch absent at {resolved_kind}")
                if cmd == "S" and state["position"] is None:
                    raise RuntimeError("ROUTE S: no forward line at junction exit")
                if cmd != "S":
                    state_name = "AXLE_ALIGNMENT"
                    align_axle(calibration, frames, motion, bus, offset,
                               allow_no_line=resolved_kind in ("T", "LEFT", "RIGHT"))
                    offset = gyro_calibrate(bus)
                    state_name = "GYRO_TURN"
                    if cmd == "B":
                        if sensor is None:
                            from sensors.vl53l0x_sensor import VL53L0XSensor
                            sensor = VL53L0XSensor()
                            sensor.initialize()
                        ok = button_task(calibration, detector, sensor, bus, offset, board)
                    elif cmd == "F":
                        ok = firefighter_task(calibration, detector, bus, offset, board)
                    elif cmd == "C":
                        ok = charging_task()
                        if ok:
                            ok, _ = pivot(bus, offset, "R")
                            ok = ok and reacquire_line(calibration)
                    else:
                        ok, _ = pivot(bus, offset, cmd)
                        if ok:
                            state_name = "PRECISION_ALIGN"
                            ok = reacquire_line(calibration)
                            state_name = "LINE_LOCK"
                    if not ok:
                        raise RuntimeError(f"{cmd} maneuver or LINE_LOCK failed")
                    heading = Heading(bus, offset, 1)
                    gyro_angle = 0.0
                if route_index >= len(route):
                    stop()
                    print("SAFE_STOP: route complete after final command")
                    break
                state_name = "FOLLOW_LINE"
                line_monitor.missing = 0
                continue
            if now - last_gyro_check >= 0.1:
                gyro_angle = heading.read()
                if not math.isfinite(gyro_angle):
                    raise RuntimeError("Gyroscope unavailable during navigation")
                last_gyro_check = now
            lost, predicted = line_monitor.observe(
                state, now, TARGET_POSITION,
                left_pwm=last_left_pwm, right_pwm=last_right_pwm,
                direction="FORWARD", gyro_angle=gyro_angle,
                route_index=route_index)
            if state["position"] is None:
                stop()
                if lost:
                    state_name = "LINE_RECOVERY"
                    print("LINE_RECOVERY: line missing on fresh ADC frames")
                    if not recover_line(calibration, frames, bus, offset,
                                        line_monitor.last_error):
                        raise RuntimeError("LINE_RECOVERY: search exhausted")
                    heading = Heading(bus, offset, 1)
                    gyro_angle = 0.0
                    state_name = "FOLLOW_LINE"
                    line_monitor.missing = 0
                time.sleep(LOOP_DELAY)
                continue
            last_error = state["position"] - TARGET_POSITION
            # Retain the working P steering; slow at a branch and near array edges.
            stable_center = (len(line_monitor.history) >= 8
                             and abs(last_error) < 0.25
                             and abs(line_monitor.derivative) < 2.0)
            target_speed = (MANEUVER_SPEED if pending_kind or predicted
                            else CRUISE_SPEED if stable_center else BASE_SPEED)
            commanded_speed += clamp(target_speed - commanded_speed, -20, 10)
            speed = commanded_speed
            if target_speed > BASE_SPEED and commanded_speed < target_speed:
                state_name = "ADAPTIVE_ACCELERATION"
            elif pending_kind is None:
                state_name = "FOLLOW_LINE"
            differential_drive(state, speed=speed, max_speed=MAX_SPEED)
            correction = KP * last_error
            last_left_pwm = round(clamp(speed + correction, MIN_SPEED, MAX_SPEED))
            last_right_pwm = round(clamp(speed - correction, MIN_SPEED, MAX_SPEED))
            if now - last_report >= 1.0:
                print(f"{state_name}: position={state['position']:+.2f}; "
                      f"route={route_index}/{len(route)}")
                last_report = now
            time.sleep(LOOP_DELAY)
    except KeyboardInterrupt:
        print("SAFE_STOP: interrupted")
        status = 130
    except Exception as exc:
        print(f"SAFE_STOP: {exc}")
        status = 1
    finally:
        if motors_on:
            try:
                motors.disarm()
            except Exception:
                pass
        if sensor is not None:
            try:
                sensor.close()
            except Exception:
                pass
        if tracker_on:
            line_tracker.shutdown()
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
        print("Robot stopped")
    return status


if __name__ == "__main__":
    raise SystemExit(main())
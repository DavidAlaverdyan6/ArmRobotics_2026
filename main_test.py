#!/usr/bin/env python3

"""ArmRobotics main route: navigation and fixed robot tasks."""



import math

import time



import smbus

from telemetrix import telemetrix



import line_tracker

import motors

from stm32_reset import STM32_RESET_GPIO, reset_stm32

from sensors.vl53l0x_sensor import VL53L0XSensor



PROGRAM_BUILD = "cube-fire-2026-11-10.1"



# Previously tested differential2 line-following speeds.

SERIAL_PORT = "/dev/serial0"

BASE_SPEED = 255

MAX_SPEED = 255

MIN_SPEED = 0

MANEUVER_SPEED = 100

MANEUVER_MAX_SPEED = 170

KP = 100.0

KI = 0.0  # Existing LineTrackingEngine is PD; integral contribution remains zero.

KD = 5.0

MAX_CORRECTION = 255  # Reference differential2 setting; current LineTrackingEngine handles clamping.

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

JUNCTION_REARM_CLEAR_FRAMES = 3

POST_TURN_JUNCTION_IGNORE_SECONDS = 0.0

POST_JUNCTION_ACCEL_SECONDS = 0.5

STRAIGHT_MAX_SPEED = 255

STRAIGHT_ACCEL_RATE = 180.0

STRAIGHT_DECEL_RATE = 300.0

STRAIGHT_CONFIRM_SECONDS = 0.25

STRAIGHT_POSITION_TOLERANCE = 0.35

STRAIGHT_DERROR_LIMIT = 8.0



# After any gyro turn, differential2 centers the robot again.

REACQUIRE_TIMEOUT = 1.2

REACQUIRE_TOLERANCE = 0.25

REACQUIRE_CONFIRM = 3

REACQUIRE_SPEED = 80

REACQUIRE_MAX_SPEED = 170

REACQUIRE_STEER_GAIN = 1.6



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



BUTTON_BUMPER_START_CM = 20.0

BUTTON_BUMPER_START_TOLERANCE_CM = 1.0

BUTTON_BUMPER_RAM_SECONDS = 1.0

BUTTON_BUMPER_APPROACH_TIMEOUT = 18.0

BUTTON_BUMPER_SPEED = 255

BUTTON_BUMPER_STATUS_SECONDS = 0.25



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

CUBE_LOST_FINAL_RIGHT_ANGLE = 80.0



FIRE_APPROACH_SPEED = 70

FIRE_APPROACH_MAX_SPEED = 100

FIRE_APPROACH_TIMEOUT = 12.0

FIRE_LINE_RECOVERY_SPEED = 80

FIRE_LINE_RECOVERY_TIMEOUT = 5.0

FIRE_LINE_RECOVERY_CONFIRM = 2

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
CUBE1_PIT_ALL_WHITE_MAX = 0.15  # Entire sensor row is white.
CUBE1_PIT_CONFIRM_FRAMES = 4
CUBE1_EMERGENCY_TIMEOUT = 8.0
CUBE1_BLINK_HALF_PERIOD = 0.25
CUBE1_DROP_CONFIRM_FRAMES = 3
CUBE1_DROP_TIMEOUT = 20.0
CUBE1_APPROACH_SPEED = 255
CUBE1_APPROACH_MAX_SPEED = 255
CUBE1_SENSOR_CONFIRM = 3
CUBE1_LINE_LOSS_CONFIRM = 5
CUBE1_EMERGENCY_REVERSE_CONFIRM = 3
CUBE1_EMERGENCY_REVERSE_SPEED = 80
# After CUBE1 returns to its starting junction, advance before the final R90.
CUBE1_RETURN_FORWARD_SECONDS = 0.5
CUBE1_RETURN_FORWARD_PWM = 80

# COLOR: stop, find a stable colored marker, and show that color on the RGB LED.

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
BUTTON_BOUNCE_START = (0.0, 150.0, 50.0)
BUTTON_BOUNCE_PRESS = (0.0, 250.0, -50.0)
BUTTON_BOUNCE_GRIPPER_CLOSED = 180.0
BUTTON_BOUNCE_DRIVE_PWM = 130
BUTTON_BOUNCE_MAX_SPEED = 130  # Wheel PWM ceiling during camera-guided approach.
BUTTON_BOUNCE_LOOP_DELAY = 0.04  # Use the validated standalone PID test cadence.
# KP=100, KI=0, KD=5 and MIN_SPEED=0 are inherited from line_follower.
# MAX_CORRECTION=255 is a reference setting; wheel PWM is clamped to 0..130.
BUTTON_BOUNCE_CENTER_FRACTION = 0.20
BUTTON_BOUNCE_RED_AREA = 120
BUTTON_BOUNCE_CONFIRM_FRAMES = 3
BUTTON_BOUNCE_SEARCH_TIMEOUT = 20.0
BUTTON_BOUNCE_REVERSE_LINE_TIMEOUT = 5.0


# Automatic route: one entry for each detected junction, in order.

# L/R/S and L90/R90 are movement commands. Named robot tasks are BUTTON,

# CHARGING, FIREFIGHTER, CUBE1 and COLOR. Q stops the route.

# Example: ROUTE = ["L", "S", "BUTTON", "R", "CHARGING", "CUBE1",

#                   "FIREFIGHTER", "Q"]

# The commented entries below are placeholders: edit them before launching.

ROUTE = [

    "BUTTON",

    "L",

    "CUBE1",

    "L",

    "L",

    "R",

    "CHARGING",

    "R",

    "L",

    "R",

    "R",

    "L",

    "FIREFIGHTER",

    "L",

    "R",

    "S",

    "R",

    "L",

    "R",

    "R",

    "S",

    "L",

    "L",

    "S",

    "Q",





]

ALLOWED_ROUTE_COMMANDS = {

    "L", "R", "L90", "R90", "S",

    "BUTTON", "BUTTON_BOUNCE", "CHARGING", "FIREFIGHTER", "CUBE1", "COLOR", "Q",

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





class JunctionRearmGate:

    """Require a stable centered line before accepting another junction."""



    def __init__(self):

        self.armed = True

        self.clear_count = 0



    def mark_junction(self):

        self.armed = False

        self.clear_count = 0



    def update(self, left, right, position):

        if self.armed:

            return

        centered_clear = (

            not left

            and not right

            and position is not None

            and abs(position - TARGET_POSITION) <= STRAIGHT_POSITION_TOLERANCE

        )

        if not centered_clear:

            self.clear_count = 0

            return

        self.clear_count += 1

        if self.clear_count >= JUNCTION_REARM_CLEAR_FRAMES:

            self.armed = True

            self.clear_count = 0





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



    def __init__(self, heading, line_loss_timeout=REVERSE_LINE_LOSS_TIMEOUT):

        self.heading = heading

        self.line_loss_timeout = line_loss_timeout

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

            self.mode = "GYRO"

            heading_error = (

                0.0 if self.reference_heading is None else

                _wrap_degrees(angle - self.reference_heading)

            )

            steering = (

                REVERSE_HEADING_KP * heading_error

                + REVERSE_YAW_RATE_KD * rate

            )

            return self._wheel_command(REVERSE_GYRO_SPEED, steering)



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

        self.mode = "LINE"

        return self._wheel_command(base, steering)





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

        left_sign = None

        if abs(signed) > 5:

            observed_sign = 1 if signed > 0 else -1

            left_sign = observed_sign if left else -observed_sign

        return abs(final_angle - angle_target) <= tolerance, left_sign



    finally:

        stop()

        time.sleep(0.02)





def aggressive_reacquire_drive(state):

    """Steer sharply at low speed until the sensor array is centered."""

    position = state.get("position") if state else None

    if position is None:

        line_follower.drive_straight(

            speed=REACQUIRE_SPEED,

            max_speed=REACQUIRE_MAX_SPEED,

        )

        return False



    correction = (

        line_follower.steering_correction(position) * REACQUIRE_STEER_GAIN

    )

    left = max(

        MIN_SPEED,

        min(REACQUIRE_MAX_SPEED, REACQUIRE_SPEED + correction),

    )

    right = max(

        MIN_SPEED,

        min(REACQUIRE_MAX_SPEED, REACQUIRE_SPEED - correction),

    )

    motors.set_speeds(round(left), round(right))

    return True





def reacquire_line(calibration):

    """Use differential2 driving after EVERY turn, including edge-only line."""

    deadline = time.monotonic() + REACQUIRE_TIMEOUT

    stable = 0



    try:

        while time.monotonic() < deadline:

            state = line_tracker.update(calibration)

            position = state["position"]



            if not aggressive_reacquire_drive(state):

                stable = 0

            else:

                if abs(position - TARGET_POSITION) <= REACQUIRE_TOLERANCE:

                    stable += 1

                    if stable >= REACQUIRE_CONFIRM:

                        print("\nREACQUIRE: line centered.")

                        return True

                else:

                    stable = 0



            print(

                f"\rREACQUIRE pos={position} N=["

                + " ".join(

                    f"{value:.2f}" for value in state["normalized"]

                )

                + "]",

                end="",

                flush=True,

            )



            time.sleep(LOOP_DELAY)



        print("\nREACQUIRE timeout: line is not centered; stopping maneuver.")

        return False



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





def activate_arm_home(board):

    """Enable all arm servos once and command the safe base pose."""

    from arm_web import START_ANGLES

    from roboarm import get_shared_arm



    arm, _ = get_shared_arm(board)

    arm.write(**START_ANGLES)

    print("ARM: active in base pose (90, 170, 180, 0, 170).")





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

                # Begin the full-speed bumper run at 20 ± 1 cm.

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

                        "full-speed bumper press."

                    )

                elif state["position"] is None:

                    stop()

                    time.sleep(LOOP_DELAY)

                    continue

                else:

                    differential_drive(

                        state,

                        speed=MANEUVER_SPEED,

                        max_speed=MANEUVER_MAX_SPEED,

                    )

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

                # Continue straight only during the bounded two-second press.

                line_follower.drive_straight(

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

        heading, line_loss_timeout=line_loss_timeout

    )

    start = time.monotonic()

    cleared_count = 0

    outer_confirm_count = 0

    candidate_since = None



    try:

        while (

            max_return_time is None

            or time.monotonic() - start < max_return_time

        ):

            state = line_tracker.update(calibration)

            left, right, candidate = detector.read(state)

            now = time.monotonic()



            if not left and not right:

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

            # Preserve the original BUTTON/FIREFIGHTER return rule.

            elif cleared_count >= JUNCTION_CLEAR and left and right:

                print(f"\n{label}: return junction found (both outer sensors).")

                return True



            # S1 has no road to detect on the CUBE1 return.  Do not let it

            # affect either junction detection or the approach speed.

            candidate_visible = right if accept_s8_only else left != right

            if candidate_visible:

                if candidate_since is None:

                    candidate_since = now

            elif accept_s8_only or (not left and not right):

                candidate_since = None



            if state["position"] is not None:

                speed = (

                    RETURN_SLOW_SPEED if candidate_since is not None else RETURN_SPEED

                )

            else:

                speed = (

                    RETURN_SLOW_SPEED if candidate_since is not None else RETURN_SPEED

                )

            if not differential_reverse(state, speed, reverse_controller):

                print(f"\n{label}: line lost too long while reversing.")

                return False



            print(

                f'\r{label} BACK {reverse_controller.mode} '

                f'pos={state["position"]} axle={reverse_controller.last_axle_error} '

                f'angle={reverse_controller.last_angle:+.1f} '

                f'S1={int(left)} S8={int(right)}',

                end="", flush=True,

            )

            time.sleep(LOOP_DELAY)



        print(f"\n{label}: reverse timeout.")

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



    print(

        f"\nB: move forward for {RETURN_FORWARD_TIME:g} seconds."

    )

    if not follow_for(calibration, RETURN_FORWARD_TIME):

        return False



    print("\nB: execute R90.")

    ok, _ = pivot_90(

        bus, offset, "R", angle_target=TURN_90_ANGLE

    )

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
    "drop_line_lost" when the line disappears after that crossing.
    "emergency" applies only when the PIT is confirmed in emergency mode.
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
    line_loss_count = 0
    pit_count = 0
    pit_kind = None
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
            # and only then check for either all-black or all-white.
            if second_crossing_seen and not second_crossing_cleared:
                if (
                    not candidate and not left and not right
                    and not all(v >= CUBE1_PIT_ALL_BLACK_MIN for v in values)
                    and not all(v <= CUBE1_PIT_ALL_WHITE_MAX for v in values)
                ):
                    post_cross_clear_count += 1
                    if post_cross_clear_count >= JUNCTION_CLEAR:
                        second_crossing_cleared = True
                        if phase == "emergency":
                            emergency_deadline = now + CUBE1_EMERGENCY_TIMEOUT
                        pit_count = 0
                        pit_kind = None
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
                        return (
                            "drop_line_lost"
                            if state.get("position") is None
                            else "drop"
                        )
                else:
                    missing_count = 0

            if phase == "emergency" and second_crossing_cleared:
                black = all(value >= CUBE1_PIT_ALL_BLACK_MIN for value in values)
                white = all(value <= CUBE1_PIT_ALL_WHITE_MAX for value in values)
                kind = "black" if black else ("white" if white else None)
                pit_count = pit_count + 1 if kind and kind == pit_kind else (1 if kind else 0)
                pit_kind = kind
                if pit_count >= CUBE1_PIT_CONFIRM_FRAMES:
                    print(f"\\nCUBE1: all 8 {kind} AFTER crossing; reversing to find line.")
                    return "emergency"

            if state.get("position") is None:
                # A confirmed line loss after the second crossing is an
                # alternative successful drop. Stop during confirmation:
                # do not drive blindly past the end of the track.
                if (
                    phase == "past_junction" and second_crossing_seen
                    and not candidate and not left and not right
                ):
                    line_loss_count += 1
                    stop()
                    if line_loss_count >= CUBE1_LINE_LOSS_CONFIRM:
                        print(
                            "\\nCUBE1: line lost after second crossing; "
                            "counting cube as dropped. Reverse to find line."
                        )
                        return "drop_line_lost"
                    time.sleep(LOOP_DELAY)
                    continue
                if phase != "emergency":
                    print("\\nCUBE1: line lost before drop; STOP.")
                    return None
                line_follower.drive_straight(
                    speed=MANEUVER_SPEED,
                    max_speed=MANEUVER_MAX_SPEED,
                )
            else:
                line_loss_count = 0
                differential_drive(state)
            if now - last_status >= 0.5:
                print(f"CUBE1 phase={phase} cube={int(cube_visible)} "
                      f"crossing_passed={int(second_crossing_cleared)}")
                last_status = now
            time.sleep(LOOP_DELAY)

        print("\\nCUBE1: search/transport timeout; STOP.")
        return None
    finally:
        stop()


def cube1_emergency_reverse_to_line(calibration):
    """Reverse until line is visible after an emergency OR a confirmed drop."""
    seen = 0
    try:
        while True:
            if not line_tracker.all_channels_fresh():
                stop()
                return False
            state = line_tracker.update(calibration)
            if state.get("position") is not None:
                seen += 1
                if seen >= CUBE1_EMERGENCY_REVERSE_CONFIRM:
                    print("\nCUBE1: line recovered; reversing to starting junction.")
                    return True
            else:
                seen = 0
            line_follower.reverse_straight(CUBE1_EMERGENCY_REVERSE_SPEED)
            time.sleep(LOOP_DELAY)
    finally:
        stop()


def cube1_final_turn_approach():
    """After reversing to the junction, move forward 0.5s PWM80 before R90.

    No trailing-edge search and no line PID during this short calibrated move.
    """
    if not line_tracker.all_channels_fresh():
        print("\nCUBE1: ADC stale before final forward pulse; stopping.")
        stop()
        return False
    try:
        print(
            f"\nCUBE1: return junction found; forward "
            f"{CUBE1_RETURN_FORWARD_SECONDS:.3f}s PWM {CUBE1_RETURN_FORWARD_PWM}."
        )
        motors.set_speeds(CUBE1_RETURN_FORWARD_PWM, CUBE1_RETURN_FORWARD_PWM)
        time.sleep(CUBE1_RETURN_FORWARD_SECONDS)
        return True
    finally:
        stop()


def cube1_task(calibration, detector, bus, offset, board):

    """Find and carry CUBE1 with the optical sensor, stopping at junction."""

    from firefighter import Firefighter



    indicator = None
    stop()

    try:

        print("\nCUBE1: execute R90.")

        if not follow_for(

            calibration,

            TURN_90_FORWARD_TIME,

            speed=TURN_90_FORWARD_SPEED,

            max_speed=TURN_90_FORWARD_MAX_SPEED,

        ):

            return False

        ok, left_sign = pivot_90(

            bus, offset, "R", angle_target=TURN_90_ANGLE

        )

        if not ok or left_sign is None:

            print("CUBE1: initial turn/gyro check failed.")

            return False

        if not reacquire_line(calibration):

            return False



        # Keep the arm folded while entering and completing the turn.  The

        # gripper is lowered only after the robot is aligned on the new line.

        if not cube1_arm_action(board):
            return False

        cube_sensor = Firefighter(board)
        cube_sensor.initialize()
        # This subscription is new, so accepting its first fresh PB0 report
        # is sufficient. Waiting for a *later* report can time out on an
        # unchanged analog signal even if the initial report was valid.
        try:
            cube_sensor.wait_for_sample()
        except TimeoutError:
            stop()
            print("\nCUBE1: PB0 reported no fresh ADC data; resubscribing once.")
            cube_sensor.initialize()
            try:
                cube_sensor.wait_for_sample()
            except TimeoutError:
                stop()
                print(
                    "\nCUBE1: PB0 still has no reports after resubscription. "
                    "Check PB0 sensor wiring / Telemetrix connection; "
                    "motors remain stopped."
                )
                return False
        indicator = Cube1Signal(cube1_create_indicator())
        status = cube1_transport(calibration, detector, cube_sensor, indicator)
        if status not in ("drop", "drop_line_lost", "emergency"):
            return False

        # Fold arm before reverse; it is no longer holding the cube.
        if not cube1_fold_arm(board, reason=status):
            return False
        if status in ("emergency", "drop_line_lost"):
            if not cube1_emergency_reverse_to_line(calibration):
                return False
        # Return on the line to the same junction. Keep the existing
        # geometry-aware reverse controller with gyro heading protection.
        if not button_return(
            calibration, detector, bus, offset, left_sign,
            label="CUBE1", accept_s8_only=True,
            max_return_time=None, line_loss_timeout=None,
        ):
            return False
        if not cube1_final_turn_approach():
            return False
        print("\nCUBE1: final R90 using main_test.py's existing pivot.")
        ok, _ = pivot_90(
            bus, offset, "R", angle_target=TURN_90_ANGLE
        )
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

    try:

        while time.monotonic() < deadline:

            state = line_tracker.update(calibration)

            if state["position"] is not None:

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





def firefighter_task(calibration, detector, bus, offset, board):

    """Left turn, stationary arm pose, confirmed flame, at most two sprays."""

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

        _return_home(arm, pose, AUTO_MAX_STEP_DEG, STEP_DELAY)

        time.sleep(0.3)



        # Servo movement can outlast the flame sample taken before the turn.

        # Wait for a new report before the robot starts approaching the flame.

        fire.wait_for_sample(after=time.monotonic())

        deadline = time.monotonic() + FIRE_APPROACH_TIMEOUT

        line_loss_abort = False

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

                    print("\nF: flame confirmed for 0.5 seconds. Water ON.")

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

                if not firefighter_reverse_until_line(calibration):

                    return False

                line_loss_abort = True

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



        if line_loss_abort:

            print("\nF: flame search aborted after line loss; no water used.")



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

        if not follow_for(calibration, RETURN_FORWARD_TIME):

            return False

        print("\nF: execute R90.")

        ok, _ = pivot_90(

            bus, offset, "R", angle_target=TURN_90_ANGLE

        )

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





def create_color_rgb_indicator():
    """Same measured red/blue-swapped LED wiring as the current main.py."""
    import led
    import robot_pin_config as pins
    rgb = led.RGBLED(pins.LED_BLUE, pins.LED_GREEN, pins.LED_RED)
    rgb.off()
    return rgb


def color_marker_task(calibration, board, bus, offset):
    """Current COLOR behavior adapted to this file's existing R90 maneuver.
    Turn right using this file's existing 90-degree maneuver, center on line,
    pose the arm, then detect color and hold the indicator until next junction.
    """
    from color_marker_detector import run_color_marker_task
    from roboarm import get_shared_arm
    global COLOR_LED_HOLD

    stop()
    marker = None
    try:
        print("\nCOLOR: execute R90 using this file's existing turn logic.")
        if not follow_for(
            calibration, TURN_90_FORWARD_TIME,
            speed=TURN_90_FORWARD_SPEED,
            max_speed=TURN_90_FORWARD_MAX_SPEED,
        ):
            return False
        ok, _ = pivot_90(bus, offset, "R", angle_target=TURN_90_ANGLE)
        if not ok or not reacquire_line(calibration):
            return False

        arm, _ = get_shared_arm(board)
        print("COLOR: arm base=90 shoulder=160 elbow=180 wrist=0 gripper=0.")
        arm.write(**COLOR_ARM_ANGLES)

        if COLOR_LED_HOLD is not None:
            try:
                COLOR_LED_HOLD.off()
                COLOR_LED_HOLD.close()
            except Exception:
                pass
            COLOR_LED_HOLD = None
        COLOR_LED_HOLD = create_color_rgb_indicator()

        def light_confirmed_color(match):
            getattr(COLOR_LED_HOLD, match["color"])()
            print(
                f"COLOR: {match['color'].upper()} indicator ON; "
                f"holding for {COLOR_MARKER_HOLD_SECONDS:g}s."
            )

        marker = run_color_marker_task(
            timeout_s=COLOR_MARKER_TIMEOUT,
            hold_seconds=COLOR_MARKER_HOLD_SECONDS,
            min_object_area=COLOR_MARKER_MIN_AREA,
            stable_frames=COLOR_MARKER_STABLE_FRAMES,
            control_led=False,
            confirmation_callback=light_confirmed_color,
            preview_port=8000,
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



def button_bounce_arm_pose(board, xyz):
    """Move the closed gripper using the existing auto-pitch IK."""
    import arm_web
    from roboarm import get_shared_arm
    from button_arm import _execute_plan

    stop()
    arm, _ = get_shared_arm(board)
    arm_web.arm = arm
    x, y, z = xyz
    path, pitch = arm_web.plan_auto_path({
        "x": x, "y": y, "z": z, "orientation_mode": "auto",
    })
    arm.write(gripper=BUTTON_BOUNCE_GRIPPER_CLOSED)
    print(f"BUTTON_BOUNCE: arm ({x:g},{y:g},{z:g}), auto pitch {pitch:.1f}.")
    _execute_plan(arm, path, arm_web.STEP_DELAY)
    arm.write(gripper=BUTTON_BOUNCE_GRIPPER_CLOSED)
    return True


def button_bounce_red_centered(objects, width, height):
    """Require the red blob's centroid in the central 20% x 20% rectangle."""
    half_w = width * BUTTON_BOUNCE_CENTER_FRACTION / 2
    half_h = height * BUTTON_BOUNCE_CENTER_FRACTION / 2
    return any(
        obj["color"] == "red"
        and obj["area"] >= BUTTON_BOUNCE_RED_AREA
        and abs(obj["center"][0] - width / 2) <= half_w
        and abs(obj["center"][1] - height / 2) <= half_h
        for obj in objects
    )


def button_bounce_find_red(calibration):
    """Follow the line at base PWM130 while watching for centered red."""
    import cv2
    from color_marker_detector import ContinuousCamera, ColorMarkerDetector

    camera = ContinuousCamera(port=8000)
    color_detector = ColorMarkerDetector(
        min_object_area=BUTTON_BOUNCE_RED_AREA,
        allowed_colors={"red"},
    )
    stop()
    try:
        camera.start()
        sequence = camera.sequence
        confirmed = 0
        deadline = time.monotonic() + BUTTON_BOUNCE_SEARCH_TIMEOUT
        camera.start_task("BUTTON_BOUNCE: searching centered red")
        last_frame_at = time.monotonic()
        while time.monotonic() < deadline:
            if not line_tracker.all_channels_fresh():
                print("\nBUTTON_BOUNCE: stale ADC; stopping.")
                return False
            # Correct steering every loop, not only when camera FPS permits.
            state = line_tracker.update(calibration)
            if state["position"] is None:
                stop()
                print("BUTTON_BOUNCE: line lost; stopping.")
                return False
            captured = camera.get_frame(after_sequence=sequence, timeout=0.01)
            if captured is None:
                if time.monotonic() - last_frame_at > 0.5:
                    stop()
                    print("BUTTON_BOUNCE: camera stalled; stopping.")
                    return False
                differential_drive(
                    state, speed=BUTTON_BOUNCE_DRIVE_PWM, max_speed=BUTTON_BOUNCE_MAX_SPEED
                )
                time.sleep(BUTTON_BOUNCE_LOOP_DELAY)
                continue
            last_frame_at = time.monotonic()
            sequence, frame = captured
            objects, annotated, _ = color_detector.process_frame(frame)
            height, width = frame.shape[:2]
            centered = button_bounce_red_centered(objects, width, height)
            confirmed = confirmed + 1 if centered else 0
            half_w = int(width * BUTTON_BOUNCE_CENTER_FRACTION / 2)
            half_h = int(height * BUTTON_BOUNCE_CENTER_FRACTION / 2)
            cv2.rectangle(
                annotated,
                (width // 2 - half_w, height // 2 - half_h),
                (width // 2 + half_w, height // 2 + half_h),
                (255, 255, 255), 2,
            )
            camera.publish_task_frame(
                annotated,
                f"BUTTON_BOUNCE: red center {confirmed}/{BUTTON_BOUNCE_CONFIRM_FRAMES}",
            )
            if confirmed >= BUTTON_BOUNCE_CONFIRM_FRAMES:
                stop()
                print("\nBUTTON_BOUNCE: centered red confirmed.")
                return True
            differential_drive(
                state, speed=BUTTON_BOUNCE_DRIVE_PWM, max_speed=BUTTON_BOUNCE_MAX_SPEED
            )
            time.sleep(BUTTON_BOUNCE_LOOP_DELAY)
        print("\nBUTTON_BOUNCE: camera search timeout.")
        return False
    finally:
        stop()
        camera.stop()


def button_bounce_task(calibration, detector, bus, offset, board):
    """Normal R, closed-gripper poses, centered red, reverse, CUBE1-style R90."""
    indicator = None
    stop()
    try:
        print("\nBUTTON_BOUNCE: execute ordinary R.")
        ok, left_sign = pivot(bus, offset, "R")
        if not ok or left_sign is None or not reacquire_line(calibration):
            return False
        if not button_bounce_arm_pose(board, BUTTON_BOUNCE_START):
            return False
        if not button_bounce_find_red(calibration):
            return False
        indicator = create_robot_rgb_indicator()
        indicator.red()
        print("BUTTON_BOUNCE: red LED on.")
        if not button_bounce_arm_pose(board, BUTTON_BOUNCE_PRESS):
            return False
        if not button_return(
            calibration, detector, bus, offset, left_sign,
            label="BUTTON_BOUNCE",
            line_loss_timeout=BUTTON_BOUNCE_REVERSE_LINE_TIMEOUT,
        ):
            return False
        # Same PWM80 0.5-second forward pulse as the completed CUBE1.
        if not cube1_final_turn_approach():
            return False
        print("\nBUTTON_BOUNCE: final R90.")
        ok, _ = pivot_90(bus, offset, "R", angle_target=TURN_90_ANGLE)
        return ok and reacquire_line(calibration)
    except Exception as exc:
        print(f"\nBUTTON_BOUNCE: error: {exc}")
        return False
    finally:
        stop()
        if indicator is not None:
            indicator.off()
            indicator.close()


def load_route():

    """Read the user-edited ROUTE list before starting the robot."""

    if not ROUTE:

        raise ValueError(

            "ROUTE is empty. Add your junction commands to the ROUTE list "

            "in main_test.py before starting main_test.py."

        )



    route = []

    for index, command in enumerate(ROUTE, 1):

        if not isinstance(command, str):

            raise ValueError(f"ROUTE entry {index} is not a string.")

        cmd = command.strip().upper()

        if cmd not in ALLOWED_ROUTE_COMMANDS:

            raise ValueError(

                f"Invalid ROUTE entry #{index}: {command!r}. "

                "Allowed: L, R, L90, R90, S, BUTTON, CHARGING, "

                "FIREFIGHTER, CUBE1, COLOR, BUTTON_BOUNCE, Q."

            )

        if cmd == "Q" and index != len(ROUTE):

            raise ValueError("Q must be the last ROUTE entry.")

        route.append(cmd)

    return route





def main():

    global COLOR_LED_HOLD

    board = bus = sensor = None

    offset_90 = None

    motors_on = tracker_on = False



    try:

        route = load_route()

        print(f"MAIN_TEST BUILD: {PROGRAM_BUILD}")

        print(f"Automatic ROUTE loaded: {route}")

        print(

            f"Resetting STM32 through Raspberry Pi BCM{STM32_RESET_GPIO}..."

        )

        reset_stm32()

        # Use the same connection and ADC subscription order as ADC.py:

        # connect once, force the drive outputs to zero, then enable the arm.

        board = telemetrix.Telemetrix(com_port=SERIAL_PORT)

        # Do this before any servo motion or callback wait.  It prevents a

        # previous PWM state from moving the chassis while servos wake up.

        motors.initialize(board)

        motors_on = True

        stop()

        activate_arm_home(board)



        # Only after the arm is held safely at home, register all ADC inputs.

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

        junction_gate = JunctionRearmGate()

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

            now = time.monotonic()

            junction_gate.update(left, right, position)



            if (now >= junction_ignore_until

                    and junction_gate.armed and candidate):

                candidate_count += 1

            else:

                candidate_count = 0



            if candidate_count >= JUNCTION_CONFIRM:

                stop()

                junction_gate.mark_junction()

                candidate_count = 0

                junction_count += 1

                # The confirmed COLOR LED remains lit until the next junction.
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
                    ok = color_marker_task(calibration, board, bus, offset_90)

                elif cmd in (

                    "L", "R", "L90", "R90", "BUTTON", "BUTTON_BOUNCE", "CHARGING",

                    "FIREFIGHTER", "CUBE1",

                ):

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



                    if cmd == "BUTTON":

                        if sensor is None:

                            sensor = VL53L0XSensor()

                            sensor.initialize()

                        ok = button_task(

                            calibration, detector, sensor, bus, offset, board

                        )

                    elif cmd == "BUTTON_BOUNCE":

                        ok = button_bounce_task(
                            calibration, detector, bus, offset, board
                        )

                    elif cmd == "FIREFIGHTER":

                        ok = firefighter_task(

                            calibration, detector, bus, offset, board

                        )

                    elif cmd == "CUBE1":

                        ok = cube1_task(

                            calibration, detector, bus, offset, board

                        )

                    elif cmd == "CHARGING":

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

                if cmd in (

                    "L", "R", "L90", "R90", "BUTTON", "CHARGING",

                    "FIREFIGHTER", "CUBE1", "COLOR", "BUTTON_BOUNCE",

                ):

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

                    f"S1={int(left)} S8={int(right)} "

                    "N=["

                    + " ".join(

                        f"{value:.2f}" for value in state["normalized"]

                    )

                    + "]",

                    end="", flush=True,

                )



            time.sleep(LOOP_DELAY)



    except KeyboardInterrupt:

        print("\nInterrupted by user.")

    except Exception as exc:

        print(f"\nERROR: {exc}")

    finally:

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
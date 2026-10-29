
#!/usr/bin/env python3

import time
import smbus
from telemetrix import telemetrix

import line_tracker
import motors
from sensors.vl53l0x_sensor import VL53L0XSensor


# ================= CONFIGURATION =================

SERIAL_PORT = "/dev/serial0"
UPDATE_INTERVAL = 0.02

LINE_SENSORS = (1, 2, 3, 4, 5, 6)
CENTRAL_SENSORS = (2, 3, 4, 5)

BLACK_THRESHOLD = 0.95
LINE_PRESENT_THRESHOLD = 0.20

BASE_SPEED = 255
TURN_SPEED = 80

LEFT_THRESHOLD = 3.3
RIGHT_THRESHOLD = 3.7


# ================= MPU-6050 =================

I2C_BUS_NUMBER = 1
MPU6050_ADDR = 0x68
GYRO_Z_REGISTER = 0x47
GYRO_SENSITIVITY = 131.0


# ================= TURNING =================

TARGET_ANGLE = 80.0
TURN_FAST_SPEED = 255
TURN_SLOW_SPEED = 90
TURN_TIMEOUT = 5.0

STRAIGHT_TIME = 0.5
ANTIBUG_CHECK_TIME = 0.5


# ================= BUTTON =================

BUTTON_DISTANCE_CM = 10.0
BUTTON_WAIT_TIME = 1.0

BUTTON_SPEED = 130
BUTTON_TURN_SPEED = 65
BUTTON_MAX_TIME = 12.0
BUTTON_CLEAR_TIMEOUT = 3.0

BUTTON_DISTANCE_CONFIRM_FRAMES = 3

# Movement forward after returning
BUTTON_FORWARD_TIME = 0.5
BUTTON_FORWARD_SPEED = 110
BUTTON_FORWARD_TURN_SPEED = 65


# ================= REVERSE CONTROLLER =================

BACKWARD_SPEED = 150

BACKWARD_LINE_KP = 35.0
BACKWARD_GYRO_KP = 2.0

BACKWARD_CENTER = 3.5
BACKWARD_DEADBAND = 0.08

BACKWARD_MAX_CORRECTION = 85
BACKWARD_TRIM = 0

BACKWARD_STEERING_SIGN = 1
BACKWARD_GYRO_SIGN = 1

BACKWARD_ANGLE_WARNING = 5.0
BACKWARD_ANGLE_RECOVERY = 12.0
BACKWARD_ANGLE_SAFE = 6.0
BACKWARD_ANGLE_MAX = 30.0

BACKWARD_RECOVERY_SPEED = 115
BACKWARD_RECOVERY_KP = 4.0
BACKWARD_RECOVERY_MAX_CORRECTION = 95
BACKWARD_RECOVERY_TIMEOUT = 1.5

BACKWARD_MAX_TIME = 15.0
BACKWARD_MIN_TIME = 0.3
BACKWARD_LOST_TIMEOUT = 0.5

BACKWARD_MIN_CENTRAL_SENSORS = 2


# ================= JUNCTIONS =================

JUNCTION_CONFIRM_FRAMES = 3
JUNCTION_CLEAR_FRAMES = 3
JUNCTION_IGNORE_TIME = 1.0

# NEW: reverse junction detection
#
# 1 = stop immediately when BOTH outer
#     sensors detect black in one reading.
#
# 2 = require two consecutive readings.
#
# Use 1 to avoid overshooting a narrow junction.

RETURN_JUNCTION_CONFIRM_FRAMES = 1


# ================= ROUTE =================

ROUTE = [
    "B",
    "L",
    "S",
    "R",
    "R",
    "L",
    "R",
    "L",
    "R",
    "R",
    "S",
    "R",
    "L",
    "L",
    "R",
]


# ================= HELPERS =================

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def clamp_speed(speed):
    return int(clamp(round(speed), -255, 255))


def is_black(value):
    return float(value) > BLACK_THRESHOLD


def read_line(calibration):
    result = line_tracker.update(calibration)
    normalized = result["normalized"]

    if len(normalized) < 8:
        raise ValueError("Expected 8 line sensors.")

    return normalized


def central_black_count(normalized):
    return sum(
        1 for i in CENTRAL_SENSORS
        if float(normalized[i]) > LINE_PRESENT_THRESHOLD
    )


def central_line_present(normalized):
    return central_black_count(normalized) >= 1


def reverse_line_present(normalized):
    return (
        central_black_count(normalized)
        >= BACKWARD_MIN_CENTRAL_SENSORS
    )


def junction_raw(normalized):
    # Normal movement: S1 OR S8
    return (
        is_black(normalized[0])
        or is_black(normalized[7])
    )


def junction_visible(normalized):
    return (
        junction_raw(normalized)
        and central_line_present(normalized)
    )


# ================= NEW RETURN DETECTION =================

def return_junction_visible(normalized):

    s1 = is_black(normalized[0])
    s8 = is_black(normalized[7])

    all_black = all(
        is_black(value)
        for value in normalized[:8]
    )

    # Do not require central sensors.
    #
    # Both outer sensors black = junction.
    # All eight black = junction as well.
    #
    # The all_black check is included
    # explicitly for readability.

    return (s1 and s8) or all_black


def get_line_position(normalized):

    values = [
        clamp(float(normalized[i]), 0.0, 1.0)
        for i in LINE_SENSORS
    ]

    total = sum(values)

    if total < 0.01:
        return None

    weighted_sum = sum(
        (i + 1) * values[i]
        for i in range(6)
    )

    return weighted_sum / total


# ================= FORWARD LINE FOLLOWING =================

def follow_line(
    position,
    base_speed=BASE_SPEED,
    turn_speed=TURN_SPEED
):

    if position < LEFT_THRESHOLD:
        left_speed = turn_speed
        right_speed = base_speed
        direction = "LEFT"

    elif position > RIGHT_THRESHOLD:
        left_speed = base_speed
        right_speed = turn_speed
        direction = "RIGHT"

    else:
        left_speed = base_speed
        right_speed = base_speed
        direction = "STRAIGHT"

    motors.set_speeds(
        clamp_speed(left_speed),
        clamp_speed(right_speed)
    )

    return direction


# ================= MPU-6050 =================

def read_word_2c(bus, register):

    high = bus.read_byte_data(
        MPU6050_ADDR, register
    )

    low = bus.read_byte_data(
        MPU6050_ADDR, register + 1
    )

    value = (high << 8) | low

    if value >= 0x8000:
        value -= 0x10000

    return value


def initialize_mpu(bus):

    bus.write_byte_data(
        MPU6050_ADDR, 0x6B, 0x00
    )
    bus.write_byte_data(
        MPU6050_ADDR, 0x1B, 0x00
    )

    time.sleep(0.2)

    print("MPU-6050 initialized.")


def read_gyro_rate(bus, gyro_offset):

    raw = read_word_2c(
        bus, GYRO_Z_REGISTER
    )

    return (
        raw - gyro_offset
    ) / GYRO_SENSITIVITY


def calibrate_gyro(bus):

    print("Calibrating gyro. Do not move robot.")

    readings = []

    for _ in range(300):
        readings.append(
            read_word_2c(bus, GYRO_Z_REGISTER)
        )
        time.sleep(0.005)

    offset = sum(readings) / len(readings)

    print(f"Gyro offset: {offset:.2f}")
    return offset


class GyroHeading:

    def __init__(self, bus, offset, left_sign=1):
        self.bus = bus
        self.offset = offset
        self.left_sign = left_sign

        self.angle = 0.0
        self.previous_time = None

    def reset(self):
        self.angle = 0.0
        self.previous_time = time.monotonic()

    def update(self):

        rate = read_gyro_rate(
            self.bus, self.offset
        )

        now = time.monotonic()

        if self.previous_time is None:
            self.previous_time = now
            return self.angle

        dt = now - self.previous_time
        self.previous_time = now

        if dt > 0:
            self.angle += (
                rate * self.left_sign * dt
            )

        return self.angle


# ================= GYRO TURN =================

def turn_80_degrees(
    bus,
    gyro_offset,
    direction,
    antibug=False
):

    if direction not in ("LEFT", "RIGHT"):
        raise ValueError("Invalid direction.")

    if antibug:
        if direction == "LEFT":
            left_speed = -TURN_FAST_SPEED
            right_speed = TURN_SLOW_SPEED
        else:
            left_speed = TURN_SLOW_SPEED
            right_speed = -TURN_FAST_SPEED

    else:
        if direction == "LEFT":
            left_speed = -TURN_SLOW_SPEED
            right_speed = TURN_FAST_SPEED
        else:
            left_speed = TURN_FAST_SPEED
            right_speed = -TURN_SLOW_SPEED

    print(f"\nTurning {direction}...")

    angle = 0.0
    signed_angle = 0.0

    start = time.monotonic()
    previous = start

    motors.set_speeds(
        left_speed, right_speed
    )

    try:
        while angle < TARGET_ANGLE:

            rate = read_gyro_rate(
                bus, gyro_offset
            )

            now = time.monotonic()
            dt = now - previous
            previous = now

            if now - start > TURN_TIMEOUT:
                print("\nTURN TIMEOUT")
                return False, None

            angle += abs(rate) * dt
            signed_angle += rate * dt

            print(
                f"\rAngle: {angle:.1f} / "
                f"{TARGET_ANGLE:.1f}",
                end="",
                flush=True
            )

            time.sleep(0.005)

        print("\nTurn complete.")

        left_sign = None

        if (
            direction == "LEFT"
            and not antibug
            and abs(signed_angle) > 20
        ):
            left_sign = (
                1 if signed_angle > 0 else -1
            )

        return True, left_sign

    finally:
        motors.motor_stop()
        time.sleep(0.2)


# ================= ANTIBUG =================

def check_line_after_turn(calibration):

    print("Anti-Bug: checking line...")

    start = time.monotonic()

    while (
        time.monotonic() - start
        < ANTIBUG_CHECK_TIME
    ):

        normalized = read_line(calibration)
        position = get_line_position(normalized)

        if (
            position is not None
            and central_line_present(normalized)
        ):
            print("Anti-Bug: line detected.")
            return True

        time.sleep(0.01)

    print("Anti-Bug: line not found.")
    return False


def execute_turn(
    bus,
    gyro_offset,
    direction,
    calibration
):

    success, left_sign = turn_80_degrees(
        bus,
        gyro_offset,
        direction
    )

    if not success:
        return False, None

    if check_line_after_turn(calibration):
        return True, left_sign

    print("Anti-Bug: rollback.")

    opposite = (
        "RIGHT" if direction == "LEFT"
        else "LEFT"
    )

    turn_80_degrees(
        bus,
        gyro_offset,
        opposite,
        antibug=True
    )

    return False, None


# ================= VL53L0X =================

def initialize_distance_sensor():

    sensor = VL53L0XSensor()
    sensor.initialize()

    return sensor


def get_distance_cm(sensor):
    return sensor.read_distance_mm() / 10.0


# ================= BUTTON APPROACH =================

def approach_button(calibration, sensor):

    print("\nBUTTON: approaching object.")

    start = time.monotonic()

    detected_frames = 0
    clear_frames = 0
    left_junction = False

    while True:

        elapsed = time.monotonic() - start

        if elapsed > BUTTON_MAX_TIME:
            motors.motor_stop()
            print("\nBUTTON: approach timeout.")
            return False

        normalized = read_line(calibration)
        position = get_line_position(normalized)

        if not junction_raw(normalized):
            clear_frames += 1
        else:
            clear_frames = 0

        if clear_frames >= JUNCTION_CLEAR_FRAMES:
            left_junction = True

        if (
            not left_junction
            and elapsed > BUTTON_CLEAR_TIMEOUT
        ):
            motors.motor_stop()
            print("\nBUTTON: junction not cleared.")
            return False

        distance = get_distance_cm(sensor)

        if (
            distance <= BUTTON_DISTANCE_CM
            and left_junction
        ):
            detected_frames += 1
        else:
            detected_frames = 0

        if (
            detected_frames
            >= BUTTON_DISTANCE_CONFIRM_FRAMES
        ):

            motors.motor_stop()

            print(
                f"\nBUTTON: distance "
                f"{distance:.1f} cm."
            )

            print("BUTTON: waiting 1 second.")
            time.sleep(BUTTON_WAIT_TIME)

            return True

        if (
            position is None
            or not central_line_present(normalized)
        ):
            motors.motor_stop()
            print("\nBUTTON: line lost.")
            return False

        follow_line(
            position,
            BUTTON_SPEED,
            BUTTON_TURN_SPEED
        )

        print(
            f"\rAPPROACH "
            f"| Distance: {distance:.1f} cm"
            f" | Pos: {position:.2f}   ",
            end="",
            flush=True
        )

        time.sleep(UPDATE_INTERVAL)


# ================= REVERSE CONTROLLER =================

def calculate_reverse_correction(
    position,
    heading,
    recovery=False
):

    line_error = position - BACKWARD_CENTER

    if abs(line_error) < BACKWARD_DEADBAND:
        line_error = 0.0

    line_correction = (
        BACKWARD_LINE_KP
        * line_error
        * BACKWARD_STEERING_SIGN
    )

    if recovery:

        gyro_correction = (
            BACKWARD_RECOVERY_KP
            * heading
            * BACKWARD_GYRO_SIGN
        )

        max_correction = (
            BACKWARD_RECOVERY_MAX_CORRECTION
        )

        base_speed = BACKWARD_RECOVERY_SPEED

    else:

        gyro_correction = (
            BACKWARD_GYRO_KP
            * heading
            * BACKWARD_GYRO_SIGN
        )

        max_correction = BACKWARD_MAX_CORRECTION
        base_speed = BACKWARD_SPEED

    total_correction = clamp(
        line_correction + gyro_correction,
        -max_correction,
        max_correction
    )

    left_speed = (
        -base_speed
        + total_correction
        + BACKWARD_TRIM
    )

    right_speed = (
        -base_speed
        - total_correction
        - BACKWARD_TRIM
    )

    left_speed = clamp(
        left_speed, -255, -15
    )

    right_speed = clamp(
        right_speed, -255, -15
    )

    motors.set_speeds(
        clamp_speed(left_speed),
        clamp_speed(right_speed)
    )

    return {
        "line_error": line_error,
        "line_correction": line_correction,
        "gyro_correction": gyro_correction,
        "left": clamp_speed(left_speed),
        "right": clamp_speed(right_speed),
    }


# ================= RETURN TO JUNCTION =================

def return_to_junction(
    calibration,
    bus,
    gyro_offset,
    gyro_left_sign
):

    print("\nRETURN: starting reverse.")
    print("Line feedback: ON")
    print("Gyro feedback: ON")
    print("Junction: S1 AND S8")

    heading_sensor = GyroHeading(
        bus,
        gyro_offset,
        left_sign=gyro_left_sign
    )

    heading_sensor.reset()

    start = time.monotonic()
    last_line_seen = start

    junction_frames = 0
    recovery_mode = False
    recovery_start = None

    normalized = read_line(calibration)

    if junction_raw(normalized):
        motors.motor_stop()
        print("RETURN: edge sensor already active.")
        return False

    try:
        while True:

            now = time.monotonic()
            elapsed = now - start

            if elapsed > BACKWARD_MAX_TIME:
                print("\nRETURN: timeout.")
                return False

            # Read line before applying line-lost logic
            normalized = read_line(calibration)

            position = get_line_position(normalized)

            s1 = is_black(normalized[0])
            s8 = is_black(normalized[7])

            all_black = all(
                is_black(v)
                for v in normalized[:8]
            )

            # ======================================
            # JUNCTION CHECK HAS HIGHEST PRIORITY
            # ======================================

            if (
                elapsed >= BACKWARD_MIN_TIME
                and return_junction_visible(normalized)
            ):
                junction_frames += 1
            else:
                junction_frames = 0

            if (
                junction_frames
                >= RETURN_JUNCTION_CONFIRM_FRAMES
            ):

                motors.motor_stop()

                print("\nRETURN: JUNCTION FOUND!")
                print(f"S1: {int(s1)}")
                print(f"S8: {int(s8)}")
                print(f"All black: {all_black}")

                return True

            # ======================================
            # GYROSCOPE
            # ======================================

            heading = heading_sensor.update()

            if abs(heading) > BACKWARD_ANGLE_MAX:
                print(
                    "\nRETURN: excessive angle "
                    f"{heading:+.1f}"
                )
                return False

            if (
                not recovery_mode
                and abs(heading)
                >= BACKWARD_ANGLE_RECOVERY
            ):
                recovery_mode = True
                recovery_start = now
                print("\nRETURN: angle recovery.")

            if (
                recovery_mode
                and abs(heading)
                <= BACKWARD_ANGLE_SAFE
            ):
                recovery_mode = False
                recovery_start = None
                print("\nRETURN: angle recovered.")

            if (
                recovery_mode
                and recovery_start is not None
                and now - recovery_start
                > BACKWARD_RECOVERY_TIMEOUT
            ):
                print("\nRETURN: recovery timeout.")
                return False

            # ======================================
            # LINE FEEDBACK
            # ======================================

            line_valid = (
                position is not None
                and reverse_line_present(normalized)
            )

            if line_valid:

                last_line_seen = now

                feedback = calculate_reverse_correction(
                    position,
                    heading,
                    recovery=recovery_mode
                )

            else:

                # Stop and keep checking sensors
                # briefly. Never drive blindly.
                motors.motor_stop()

                feedback = {
                    "line_error": 0,
                    "line_correction": 0,
                    "gyro_correction": 0,
                    "left": 0,
                    "right": 0,
                }

                if (
                    now - last_line_seen
                    > BACKWARD_LOST_TIMEOUT
                ):
                    print(
                        "\nRETURN: line lost."
                    )
                    return False

            mode = (
                "RECOVERY"
                if recovery_mode
                else "NORMAL"
            )

            pos_text = (
                f"{position:.2f}"
                if position is not None
                else "---"
            )

            print(
                f"\rRETURN"
                f" | Mode: {mode}"
                f" | Pos: {pos_text}"
                f" | S1: {int(s1)}"
                f" | S8: {int(s8)}"
                f" | Gyro: {heading:+.1f}"
                f" | L: {feedback['left']}"
                f" | R: {feedback['right']}    ",
                end="",
                flush=True
            )

            time.sleep(UPDATE_INTERVAL)

    finally:
        motors.motor_stop()


# ================= FORWARD AFTER RETURN =================

def move_forward_after_return(calibration):

    print(
        "\nBUTTON: moving forward "
        f"{BUTTON_FORWARD_TIME:.2f}s."
    )

    start = time.monotonic()

    try:
        while (
            time.monotonic() - start
            < BUTTON_FORWARD_TIME
        ):

            normalized = read_line(calibration)
            position = get_line_position(normalized)

            if (
                position is None
                or not central_line_present(normalized)
            ):
                print(
                    "\nBUTTON: line lost "
                    "during forward movement."
                )
                return False

            follow_line(
                position,
                BUTTON_FORWARD_SPEED,
                BUTTON_FORWARD_TURN_SPEED
            )

            time.sleep(UPDATE_INTERVAL)

        print("\nForward movement complete.")
        return True

    finally:
        motors.motor_stop()
        time.sleep(0.2)


# ================= BUTTON TASK =================

def execute_button(
    bus,
    gyro_offset,
    calibration,
    distance_sensor
):

    print("\n==========================")
    print("BUTTON TASK START")
    print("==========================")

    motors.motor_stop()

    print("\nSTEP 1: turn LEFT.")

    success, gyro_left_sign = execute_turn(
        bus,
        gyro_offset,
        "LEFT",
        calibration
    )

    if not success:
        return False

    if gyro_left_sign is None:
        print("Gyro polarity not determined.")
        return False

    print("\nSTEP 2: approach button.")

    if not approach_button(
        calibration,
        distance_sensor
    ):
        return False

    print("\nSTEP 3: reverse to junction.")

    if not return_to_junction(
        calibration,
        bus,
        gyro_offset,
        gyro_left_sign
    ):
        return False

    print("\nSTEP 4: forward 0.5 seconds.")

    if not move_forward_after_return(calibration):
        return False

    print("\nSTEP 5: turn RIGHT.")

    success, _ = execute_turn(
        bus,
        gyro_offset,
        "RIGHT",
        calibration
    )

    if not success:
        return False

    print("\nBUTTON TASK COMPLETE.")
    return True


# ================= MAIN =================

def main():

    board = None
    bus = None
    distance_sensor = None

    tracker_ready = False
    motors_ready = False

    junction_count = 0
    route_index = 0

    junction_latched = False
    junction_frames = 0
    clear_frames = 0
    ignore_until = 0.0

    if any(
        item not in {"R", "L", "S", "B"}
        for item in ROUTE
    ):
        print("Invalid ROUTE.")
        return

    try:

        print("Connecting MPU-6050...")

        bus = smbus.SMBus(I2C_BUS_NUMBER)
        initialize_mpu(bus)

        print("Connecting Telemetrix...")

        board = telemetrix.Telemetrix(
            com_port=SERIAL_PORT
        )

        line_tracker.initialize(board)
        tracker_ready = True

        calibration = line_tracker.load_calibration()

        if calibration is None:
            print("Line calibration missing.")
            return

        motors.initialize(board)
        motors_ready = True
        motors.motor_stop()

        print("Connecting VL53L0X...")

        distance_sensor = (
            initialize_distance_sensor()
        )

        distance = get_distance_cm(
            distance_sensor
        )

        print(
            f"Initial distance: {distance:.1f} cm"
        )

        gyro_offset = calibrate_gyro(bus)

        print("\nROBOT READY")
        print("ROUTE:", ROUTE)
        print("Reverse junction: S1 AND S8")
        print("Ctrl+C to stop.")

        time.sleep(0.5)

        while True:

            normalized = read_line(calibration)
            junction = junction_visible(normalized)

            if junction_raw(normalized):
                clear_frames = 0
            else:
                clear_frames += 1

                if (
                    clear_frames
                    >= JUNCTION_CLEAR_FRAMES
                ):
                    junction_latched = False

            if time.monotonic() < ignore_until:

                position = get_line_position(
                    normalized
                )

                if (
                    position is None
                    or not central_line_present(normalized)
                ):
                    motors.motor_stop()
                else:
                    follow_line(position)

                time.sleep(UPDATE_INTERVAL)
                continue

            if junction and not junction_latched:
                junction_frames += 1
            else:
                junction_frames = 0

            if (
                junction_frames >= JUNCTION_CONFIRM_FRAMES
                and not junction_latched
            ):

                junction_latched = True
                junction_frames = 0
                junction_count += 1

                motors.motor_stop()

                print(
                    f"\nJUNCTION #{junction_count}"
                )

                if route_index >= len(ROUTE):
                    print("ROUTE COMPLETE.")
                    break

                direction = ROUTE[route_index]
                print("COMMAND:", direction)

                success = True

                if direction == "B":

                    success = execute_button(
                        bus,
                        gyro_offset,
                        calibration,
                        distance_sensor
                    )

                elif direction == "S":

                    motors.set_speeds(
                        BASE_SPEED,
                        BASE_SPEED
                    )

                    time.sleep(STRAIGHT_TIME)
                    motors.motor_stop()
                    time.sleep(0.2)

                else:

                    turn_direction = (
                        "LEFT"
                        if direction == "L"
                        else "RIGHT"
                    )

                    success, _ = execute_turn(
                        bus,
                        gyro_offset,
                        turn_direction,
                        calibration
                    )

                if not success:
                    motors.motor_stop()
                    print("\nManeuver failed.")
                    break

                route_index += 1

                ignore_until = (
                    time.monotonic()
                    + JUNCTION_IGNORE_TIME
                )

                junction_latched = True
                print("\nFollowing line...")
                continue

            position = get_line_position(normalized)

            if (
                position is None
                or not central_line_present(normalized)
            ):
                motors.motor_stop()
                direction_text = "LINE LOST"
                pos_text = "---"

            else:
                direction_text = follow_line(position)
                pos_text = f"{position:.2f}"

            next_route = (
                ROUTE[route_index]
                if route_index < len(ROUTE)
                else "END"
            )

            print(
                f"\rPos: {pos_text}"
                f" | Direction: {direction_text}"
                f" | S1: {normalized[0]:.2f}"
                f" | S8: {normalized[7]:.2f}"
                f" | Junctions: {junction_count}"
                f" | Next: {next_route}    ",
                end="",
                flush=True
            )

            time.sleep(UPDATE_INTERVAL)

    except KeyboardInterrupt:
        print("\nProgram interrupted.")

    except Exception as error:
        print(f"\nERROR: {error}")

    finally:

        if motors_ready:
            try:
                motors.motor_stop()
            except Exception:
                pass

        if distance_sensor is not None:
            try:
                distance_sensor.close()
            except Exception:
                pass

        if tracker_ready:
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

        print(
            f"\nJunctions detected: {junction_count}"
        )
        print("Robot stopped.")


if __name__ == "__main__":
    main()

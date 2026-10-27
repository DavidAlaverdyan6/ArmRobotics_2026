#!/usr/bin/env python3

import time
import math
import smbus
from telemetrix import telemetrix

import line_tracker
import motors
from sensors.vl53l0x_sensor import VL53L0XSensor


# =============== CONFIGURATION ===============

SERIAL_PORT = "/dev/serial0"
UPDATE_INTERVAL = 0.02

# VL53L0X
OBSTACLE_DISTANCE_MM = 100

# Line sensors
LINE_SENSORS = (1, 2, 3, 4, 5, 6)
S1_INDEX = 0
S8_INDEX = 7
BLACK_THRESHOLD = 0.95

# Line following
BASE_SPEED = 255
TURN_SPEED = 80
LEFT_THRESHOLD = 3.3
RIGHT_THRESHOLD = 3.7

# MPU-6050
I2C_BUS_NUMBER = 1
MPU6050_ADDR = 0x68

ACCEL_X_REGISTER = 0x3B
ACCEL_Y_REGISTER = 0x3D
ACCEL_Z_REGISTER = 0x3F
GYRO_Z_REGISTER = 0x47

ACCEL_SENSITIVITY = 16384.0
GYRO_SENSITIVITY = 131.0

# Route turns
TARGET_ANGLE = 80.0
TURN_FAST_SPEED = 255
TURN_SLOW_SPEED = 90
TURN_TIMEOUT = 5.0

STRAIGHT_TIME = 0.5

# Anti-Bug
ANTIBUG_CHECK_TIME = 0.5

# Line centering before backward
ALIGN_SPEED = 65
ALIGN_TIMEOUT = 4.0
ALIGN_STABLE_READS = 5

ALIGN_CENTER = 3.5
ALIGN_MIN = 3.4
ALIGN_MAX = 3.6

# Backward
BACKWARD_SPEED = 150

# Maximum permitted angle deviation
MAX_DEVIATION_DEG = 3.0

# Heading correction
HEADING_KP = 3.0
MAX_HEADING_CORRECTION = 50

obstacle_count = 1


# =============== ROUTE ===============

ROUTE = [
    "L",
    "R",
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


# =============== HELPERS ===============

def clamp_speed(speed):
    return max(-255, min(255, int(speed)))


def is_black(value):
    return float(value) > BLACK_THRESHOLD


def get_line_position(normalized):
    values = [
        max(0.0, float(normalized[i]))
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


def follow_line(position):
    if position < LEFT_THRESHOLD:
        left_speed = TURN_SPEED
        right_speed = BASE_SPEED
        direction = "LEFT"

    elif position > RIGHT_THRESHOLD:
        left_speed = BASE_SPEED
        right_speed = TURN_SPEED
        direction = "RIGHT"

    else:
        left_speed = BASE_SPEED
        right_speed = BASE_SPEED
        direction = "STRAIGHT"

    motors.set_speeds(
        clamp_speed(left_speed),
        clamp_speed(right_speed)
    )

    return direction


# =============== MPU-6050 ===============

def read_word_2c(bus, register):
    high = bus.read_byte_data(MPU6050_ADDR, register)
    low = bus.read_byte_data(MPU6050_ADDR, register + 1)

    value = (high << 8) | low

    if value >= 0x8000:
        value -= 0x10000

    return value


def initialize_mpu(bus):
    bus.write_byte_data(MPU6050_ADDR, 0x6B, 0x00)

    # Gyroscope range: +/- 250 degrees/sec
    bus.write_byte_data(MPU6050_ADDR, 0x1B, 0x00)

    # Accelerometer range: +/- 2g
    bus.write_byte_data(MPU6050_ADDR, 0x1C, 0x00)

    time.sleep(0.2)
    print("MPU-6050-ը միացված է։")


def read_acceleration(bus):
    ax_raw = read_word_2c(bus, ACCEL_X_REGISTER)
    ay_raw = read_word_2c(bus, ACCEL_Y_REGISTER)
    az_raw = read_word_2c(bus, ACCEL_Z_REGISTER)

    ax = ax_raw / ACCEL_SENSITIVITY
    ay = ay_raw / ACCEL_SENSITIVITY
    az = az_raw / ACCEL_SENSITIVITY

    return ax, ay, az


def calculate_tilt(ax, ay, az):
    pitch = math.degrees(
        math.atan2(
            ax,
            math.sqrt(ay * ay + az * az)
        )
    )

    roll = math.degrees(
        math.atan2(
            ay,
            math.sqrt(ax * ax + az * az)
        )
    )

    return pitch, roll


def calibrate_gyro(bus):
    print("Գիրոսկոպի կալիբրացիա։ Ռոբոտին մի շարժիր...")

    readings = []

    for _ in range(300):
        readings.append(
            read_word_2c(bus, GYRO_Z_REGISTER)
        )
        time.sleep(0.005)

    offset = sum(readings) / len(readings)

    print(f"Գիրոսկոպի Z շեղում՝ {offset:.2f}")

    return offset


def calibrate_tilt(bus):
    print("Աքսելերոմետրի կալիբրացիա...")

    pitch_values = []
    roll_values = []

    for _ in range(50):
        ax, ay, az = read_acceleration(bus)
        pitch, roll = calculate_tilt(ax, ay, az)

        pitch_values.append(pitch)
        roll_values.append(roll)

        time.sleep(0.01)

    initial_pitch = sum(pitch_values) / len(pitch_values)
    initial_roll = sum(roll_values) / len(roll_values)

    print(
        f"Սկզբնական pitch={initial_pitch:.2f}°, "
        f"roll={initial_roll:.2f}°"
    )

    return initial_pitch, initial_roll


# =============== TURN BY GYROSCOPE ===============

def turn_80_degrees(
    bus,
    gyro_offset,
    direction,
    antibug=False
):
    if antibug:
        if direction == "LEFT":
            left_speed = -TURN_FAST_SPEED
            right_speed = TURN_SLOW_SPEED
        else:
            left_speed = TURN_SLOW_SPEED
            right_speed = -TURN_FAST_SPEED

        print(f"\nAnti-Bug՝ հակադարձ պտույտ {direction}...")

    else:
        if direction == "LEFT":
            left_speed = -TURN_SLOW_SPEED
            right_speed = TURN_FAST_SPEED
        else:
            left_speed = TURN_FAST_SPEED
            right_speed = -TURN_SLOW_SPEED

        print(f"\nՊտտվում եմ {direction}՝ {TARGET_ANGLE}°...")

    angle = 0.0
    start_time = time.monotonic()
    previous_time = start_time

    motors.set_speeds(left_speed, right_speed)

    try:
        while angle < TARGET_ANGLE:
            now = time.monotonic()

            if now - start_time > TURN_TIMEOUT:
                print("\nՊտույտի ժամանակը լրացավ։")
                return False

            dt = now - previous_time
            previous_time = now

            gyro_raw = read_word_2c(
                bus,
                GYRO_Z_REGISTER
            )

            gyro_rate = (
                gyro_raw - gyro_offset
            ) / GYRO_SENSITIVITY

            angle += abs(gyro_rate) * dt

            print(
                f"\rԱնկյուն՝ {angle:.1f}° / "
                f"{TARGET_ANGLE:.1f}°",
                end="",
                flush=True
            )

            time.sleep(0.005)

        print("\nՊտույտն ավարտվեց։")
        return True

    finally:
        motors.motor_stop()
        time.sleep(0.2)


# =============== ANTIBUG CHECK ===============

def check_line_after_turn(calibration):
    print("Anti-Bug՝ ստուգում եմ գիծը...")

    start_time = time.monotonic()

    while time.monotonic() - start_time < ANTIBUG_CHECK_TIME:
        result = line_tracker.update(calibration)
        normalized = result["normalized"]

        if len(normalized) < 8:
            raise ValueError("Սպասվում են 8 սենսորների տվյալներ։")

        if any(is_black(normalized[i]) for i in range(8)):
            print("Գիծը հայտնաբերվեց։")
            return True

        time.sleep(0.01)

    print("Գիծը չի հայտնաբերվել։")
    return False


# =============== ALIGN LINE BEFORE BACKWARD ===============

def align_line_before_backward(calibration):
    """
    Տեղում պտտվում է մինչև 3.4 < position < 3.6։
    Պայմանը պետք է հաստատվի 5 անընդմեջ չափման ժամանակ։
    """

    print("\nԳիծը կենտրոնացնում եմ տեղում...")

    start_time = time.monotonic()
    stable_reads = 0

    try:
        while time.monotonic() - start_time < ALIGN_TIMEOUT:
            result = line_tracker.update(calibration)
            normalized = result["normalized"]

            if len(normalized) < 8:
                raise ValueError("Սպասվում են 8 սենսորների տվյալներ։")

            position = get_line_position(normalized)

            if position is None:
                motors.motor_stop()
                stable_reads = 0
                print("\rԳիծը չի հայտնաբերվում։", end="", flush=True)
                time.sleep(0.02)
                continue

            error = position - ALIGN_CENTER

            print(
                f"\rPosition={position:.3f} | "
                f"Կայուն={stable_reads}/{ALIGN_STABLE_READS}",
                end="",
                flush=True
            )

            if ALIGN_MIN < position < ALIGN_MAX:
                motors.motor_stop()
                stable_reads += 1

                if stable_reads >= ALIGN_STABLE_READS:
                    print(
                        f"\nԳիծը կենտրոնացված է՝ {position:.3f}"
                    )
                    return True

            else:
                stable_reads = 0

                turn_speed = min(
                    ALIGN_SPEED,
                    max(35, int(abs(error) * 35))
                )

                if error < 0:
                    motors.set_speeds(
                        -turn_speed,
                        turn_speed
                    )
                else:
                    motors.set_speeds(
                        turn_speed,
                        -turn_speed
                    )

            time.sleep(0.02)

        print("\nՉհաջողվեց գիծը կենտրոնացնել։")
        return False

    finally:
        motors.motor_stop()


# =============== BACKWARD UNTIL JUNCTION ===============

def go_backward_with_angle_limit(
    bus,
    gyro_offset,
    calibration,
    initial_pitch,
    initial_roll
):
    """
    Հետ է գնում այնքան ժամանակ, մինչև S1 կամ S8
    հայտնաբերի խաչմերուկը։

    Ժամանակային սահմանափակում չկա։
    Եթե անկյունային շեղումը գերազանցի 3°-ը,
    ռոբոտը կանգնում է անվտանգության համար։
    """

    print("\nՍկսում եմ հետընթացը՝ մինչև խաչմերուկը։")
    print(f"Թույլատրելի շեղում՝ ±{MAX_DEVIATION_DEG:.1f}°")

    heading_angle = 0.0
    previous_time = time.monotonic()

    try:
        while True:
            now = time.monotonic()

            result = line_tracker.update(calibration)
            normalized = result["normalized"]

            if len(normalized) < 8:
                raise ValueError("Սպասվում են 8 սենսորների տվյալներ։")

            # 1. Նախ ստուգում ենք խաչմերուկը
            if (
                is_black(normalized[S1_INDEX])
                or is_black(normalized[S8_INDEX])
            ):
                motors.motor_stop()
                print("\nԽաչմերուկը հայտնաբերվեց։")
                return True

            # 2. Չափում ենք աքսելերոմետրի թեքությունը
            ax, ay, az = read_acceleration(bus)
            pitch, roll = calculate_tilt(ax, ay, az)

            pitch_deviation = abs(pitch - initial_pitch)
            roll_deviation = abs(roll - initial_roll)

            # 3. Չափում ենք գիրոսկոպի Z անկյունը
            dt = now - previous_time
            previous_time = now

            gyro_raw = read_word_2c(
                bus,
                GYRO_Z_REGISTER
            )

            gyro_rate = (
                gyro_raw - gyro_offset
            ) / GYRO_SENSITIVITY

            heading_angle += gyro_rate * dt
            heading_deviation = abs(heading_angle)

            # 4. 3°-ից մեծ շեղման դեպքում կանգ ենք առնում
            if (
                heading_deviation > MAX_DEVIATION_DEG
                or pitch_deviation > MAX_DEVIATION_DEG
                or roll_deviation > MAX_DEVIATION_DEG
            ):
                motors.motor_stop()

                print("\nՇԵՂՈՒՄԸ ԳԵՐԱԶԱՆՑԵՑ 3°-Ը։")
                print(
                    f"Heading={heading_angle:.2f}°, "
                    f"Pitch Δ={pitch_deviation:.2f}°, "
                    f"Roll Δ={roll_deviation:.2f}°"
                )
                print("Ռոբոտը կանգնեցվել է անվտանգության համար։")

                return False

            # 5. Ուղղության ուղղում գիրոսկոպի տվյալներով
            correction = heading_angle * HEADING_KP

            correction = max(
                -MAX_HEADING_CORRECTION,
                min(MAX_HEADING_CORRECTION, correction)
            )

            left_speed = -BACKWARD_SPEED + correction
            right_speed = -BACKWARD_SPEED - correction

            motors.set_speeds(
                clamp_speed(left_speed),
                clamp_speed(right_speed)
            )

            print(
                f"\rՀետընթաց | Heading={heading_angle:.2f}° | "
                f"Pitch Δ={pitch_deviation:.2f}° | "
                f"Roll Δ={roll_deviation:.2f}° | "
                f"S1={normalized[S1_INDEX]:.2f} | "
                f"S8={normalized[S8_INDEX]:.2f}   ",
                end="",
                flush=True
            )

            time.sleep(UPDATE_INTERVAL)

    finally:
        motors.motor_stop()


# =============== PROCESS ROUTE COMMAND ===============

def process_route_command(
    direction,
    bus,
    gyro_offset,
    calibration
):
    if direction == "S":
        print("Շարժվում եմ ուղիղ...")

        motors.set_speeds(BASE_SPEED, BASE_SPEED)
        time.sleep(STRAIGHT_TIME)
        motors.motor_stop()

        return True

    turn_direction = "LEFT" if direction == "L" else "RIGHT"

    success = turn_80_degrees(
        bus,
        gyro_offset,
        turn_direction
    )

    if not success:
        return False

    if not check_line_after_turn(calibration):
        print("Anti-Bug՝ գիծը չի գտնվել։")

        opposite_direction = (
            "RIGHT" if turn_direction == "LEFT" else "LEFT"
        )

        return turn_80_degrees(
            bus,
            gyro_offset,
            opposite_direction,
            antibug=True
        )

    return True


# =============== MAIN ===============

def main():
    global obstacle_count

    board = None
    bus = None
    distance_sensor = None

    tracker_ready = False
    motors_ready = False
    distance_sensor_active = True

    junction_latched = False
    junction_count = 0
    route_index = 0

    obstacle_count = 1

    if any(command not in {"L", "R", "S"} for command in ROUTE):
        print('ROUTE-ում թույլատրվում են միայն "L", "R", "S"։')
        return

    try:
        # VL53L0X
        print("Միանում եմ VL53L0X-ին...")
        distance_sensor = VL53L0XSensor()
        distance_sensor.initialize()
        print("VL53L0X-ը միացված է։")

        # MPU-6050
        print("Միանում եմ MPU-6050-ին...")
        bus = smbus.SMBus(I2C_BUS_NUMBER)
        initialize_mpu(bus)

        # Telemetrix
        print("Միանում եմ Telemetrix-ին...")
        board = telemetrix.Telemetrix(
            com_port=SERIAL_PORT
        )

        line_tracker.initialize(board)
        tracker_ready = True

        calibration = line_tracker.load_calibration()

        if calibration is None:
            print("Գծի սենսորների կալիբրացիան չի գտնվել։")
            return

        motors.initialize(board)
        motors_ready = True
        motors.motor_stop()

        gyro_offset = calibrate_gyro(bus)
        initial_pitch, initial_roll = calibrate_tilt(bus)

        print("\nՌոբոտը պատրաստ է։")
        print(f"ROUTE={ROUTE}")
        print("Գծի կենտրոնացման պայման՝ 3.4 < position < 3.6")
        print("Հետընթաց՝ մինչև խաչմերուկը հայտնաբերվի")
        print("Անկյունային շեղման սահման՝ 3°")

        while True:

            # =============== DISTANCE SENSOR ===============

            if distance_sensor_active and distance_sensor is not None:
                distance_mm = distance_sensor.read_distance_mm()

                if (
                    distance_mm is not None
                    and 0 < distance_mm < OBSTACLE_DISTANCE_MM
                ):
                    motors.motor_stop()

                    print(f"\nԽոչընդոտ՝ {distance_mm} մմ։")

                    obstacle_count += 1
                    print(f"obstacle_count={obstacle_count}")

                    try:
                        distance_sensor.close()
                    except Exception as error:
                        print(f"Հեռաչափի անջատման սխալ՝ {error}")

                    distance_sensor = None
                    distance_sensor_active = False

                    # 1. Կենտրոնացնել գիծը տեղում
                    aligned = align_line_before_backward(calibration)

                    if not aligned:
                        print("Գիծը չի կենտրոնացվել։ Կանգնում եմ։")
                        break

                    # 2. Չափել սկզբնական թեքությունը
                    initial_pitch, initial_roll = calibrate_tilt(bus)

                    # 3. Հետ գնալ մինչև խաչմերուկը
                    junction_found = go_backward_with_angle_limit(
                        bus,
                        gyro_offset,
                        calibration,
                        initial_pitch,
                        initial_roll
                    )

                    motors.motor_stop()

                    if not junction_found:
                        print(
                            "Հետընթացը կանգնեցվել է։ "
                            "ROUTE հրահանգը չի կատարվում։"
                        )
                        break

                    junction_latched = True
                    junction_count += 1

                    print(f"\nԽաչմերուկ #{junction_count}")

                    if route_index >= len(ROUTE):
                        print("Երթուղին ավարտված է։")
                        break

                    direction = ROUTE[route_index]
                    route_index += 1

                    print(f"ROUTE հրահանգ՝ {direction}")

                    success = process_route_command(
                        direction,
                        bus,
                        gyro_offset,
                        calibration
                    )

                    if not success:
                        route_index = max(0, route_index - 1)
                        print("Anti-Bug՝ ROUTE հրահանգը պահպանված է։")

                    continue

            # =============== NORMAL LINE TRACKING ===============

            result = line_tracker.update(calibration)
            normalized = result["normalized"]

            if len(normalized) < 8:
                raise ValueError("Սպասվում են 8 սենսորների տվյալներ։")

            s1_black = is_black(normalized[S1_INDEX])
            s8_black = is_black(normalized[S8_INDEX])

            if not s1_black and not s8_black:
                junction_latched = False

            # =============== JUNCTION ===============

            if (
                (s1_black or s8_black)
                and not junction_latched
            ):
                junction_latched = True
                junction_count += 1

                motors.motor_stop()
                print(f"\nԽաչմերուկ #{junction_count}")

                if route_index >= len(ROUTE):
                    print("Երթուղին ավարտված է։")
                    break

                direction = ROUTE[route_index]
                route_index += 1

                print(f"ROUTE հրահանգ՝ {direction}")

                success = process_route_command(
                    direction,
                    bus,
                    gyro_offset,
                    calibration
                )

                if not success:
                    route_index = max(0, route_index - 1)
                    print("Anti-Bug՝ ROUTE հրահանգը պահպանված է։")

                continue

            # =============== FOLLOW LINE ===============

            position = get_line_position(normalized)

            if position is None:
                motors.motor_stop()
                direction_text = "LINE LOST"
                position_text = "---"

            else:
                direction_text = follow_line(position)
                position_text = f"{position:.2f}"

            next_route = (
                ROUTE[route_index]
                if route_index < len(ROUTE)
                else "END"
            )

            print(
                f"\rPosition={position_text} | "
                f"Direction={direction_text} | "
                f"S1={normalized[S1_INDEX]:.2f} | "
                f"S8={normalized[S8_INDEX]:.2f} | "
                f"Junctions={junction_count} | "
                f"Next={next_route} | "
                f"obstacle_count={obstacle_count}   ",
                end="",
                flush=True
            )

            time.sleep(UPDATE_INTERVAL)

    except KeyboardInterrupt:
        print("\nԾրագիրը կանգնեցվում է։")

    except Exception as error:
        print(f"\nՍխալ՝ {error}")

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

        print(f"\nԽաչմերուկների քանակը՝ {junction_count}")
        print(f"obstacle_count={obstacle_count}")
        print("Ռոբոտը կանգնեցված է։")


if __name__ == "__main__":
    main()
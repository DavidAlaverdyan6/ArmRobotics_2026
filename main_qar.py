#!/usr/bin/env python3

import time
import smbus
from telemetrix import telemetrix

import line_tracker
import motors


# =============== CONFIGURATION ===============

SERIAL_PORT = "/dev/serial0"
UPDATE_INTERVAL = 0.02

# S2-S7 line sensors
LINE_SENSORS = (1, 2, 3, 4, 5, 6)

S1_INDEX = 0
S8_INDEX = 7

BASE_SPEED = 255
TURN_SPEED = 80

LEFT_THRESHOLD = 3.3
RIGHT_THRESHOLD = 3.7
BLACK_THRESHOLD = 0.95

# MPU-6050
I2C_BUS_NUMBER = 1
MPU6050_ADDR = 0x68
GYRO_Z_REGISTER = 0x47
GYRO_SENSITIVITY = 131.0

# Turning
TARGET_ANGLE = 80.0
TURN_FAST_SPEED = 255
TURN_SLOW_SPEED = 90
TURN_TIMEOUT = 5.0

# Straight movement through junction
STRAIGHT_TIME = 0.5

# Anti-Bug
ANTIBUG_CHECK_TIME = 0.5


# =============== ROUTE ===============
# L = Left, R = Right, S = Straight

ROUTE = [
    "S",
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
    bus.write_byte_data(MPU6050_ADDR, 0x1B, 0x00)

    time.sleep(0.2)

    print("MPU-6050-ը միացված է։")


def calibrate_gyro(bus):
    print("Գիրոսկոպի կալիբրացիա։ Ռոբոտին մի շարժիր...")

    readings = []

    for _ in range(300):
        readings.append(
            read_word_2c(bus, GYRO_Z_REGISTER)
        )

        time.sleep(0.005)

    offset = sum(readings) / len(readings)

    print(f"Գիրոսկոպի շեղում՝ {offset:.2f}")
    print("Կալիբրացիան ավարտվեց։")

    return offset


# =============== TURN 80 DEGREES ===============

def turn_80_degrees(
    bus,
    gyro_offset,
    direction,
    antibug=False
):
    # Anti-Bug-ի հետադարձ պտույտի արագություններ
    if antibug:

        if direction == "LEFT":
            left_speed = -TURN_FAST_SPEED
            right_speed = TURN_SLOW_SPEED

            print(
                "\nAnti-Bug: հետադարձ պտույտ ՁԱԽ՝ 80°..."
            )

        else:
            left_speed = TURN_SLOW_SPEED
            right_speed = -TURN_FAST_SPEED

            print(
                "\nAnti-Bug: հետադարձ պտույտ ԱՋ՝ 80°..."
            )

    # Սովորական պտույտների արագություններ
    else:

        if direction == "LEFT":
            left_speed = -TURN_SLOW_SPEED
            right_speed = TURN_FAST_SPEED

            print("\nՊտտվում եմ ՁԱԽ՝ 80°...")

        else:
            left_speed = TURN_FAST_SPEED
            right_speed = -TURN_SLOW_SPEED

            print("\nՊտտվում եմ ԱՋ՝ 80°...")

    angle = 0.0
    start_time = time.monotonic()
    previous_time = start_time

    motors.set_speeds(left_speed, right_speed)

    try:
        while angle < TARGET_ANGLE:

            current_time = time.monotonic()

            dt = current_time - previous_time
            previous_time = current_time

            if current_time - start_time > TURN_TIMEOUT:
                print("\nՊտույտի ժամանակը լրացավ։")
                return False

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
                f"{TARGET_ANGLE:.1f}°   ",
                end="",
                flush=True
            )

            time.sleep(0.005)

        print("\n80° պտույտն ավարտվեց։")

        return True

    finally:
        motors.motor_stop()
        time.sleep(0.2)


# =============== ANTIBUG CHECK ===============

def check_line_after_turn(calibration):
    print("Anti-Bug: ստուգում եմ գիծը...")

    start_time = time.monotonic()

    while (
        time.monotonic() - start_time
        < ANTIBUG_CHECK_TIME
    ):
        result = line_tracker.update(calibration)
        normalized = result["normalized"]

        if len(normalized) < 8:
            raise ValueError(
                "Anti-Bug-ի համար սպասվում են "
                "8 սենսորների տվյալներ։"
            )

        # Եթե թեկուզ մեկ սենսոր սև է տեսնում
        if any(
            is_black(normalized[i])
            for i in range(8)
        ):
            print("Anti-Bug: գիծը հայտնաբերվեց։")
            return True

        time.sleep(0.01)

    print("Anti-Bug: բոլոր սենսորները սպիտակ են։")

    return False


# =============== MAIN ===============

def main():
    board = None
    bus = None

    tracker_ready = False
    motors_ready = False

    junction_latched = False
    junction_count = 0
    route_index = 0

    valid_directions = {"R", "L", "S"}

    if any(
        direction not in valid_directions
        for direction in ROUTE
    ):
        print(
            'Սխալ ROUTE։ Օգտագործիր միայն "R", "L", "S"։'
        )
        return

    try:
        print("Միանում եմ MPU-6050-ին...")

        bus = smbus.SMBus(I2C_BUS_NUMBER)
        initialize_mpu(bus)

        print("Միանում եմ Telemetrix-ին...")

        board = telemetrix.Telemetrix(
            com_port=SERIAL_PORT
        )

        line_tracker.initialize(board)
        tracker_ready = True

        calibration = line_tracker.load_calibration()

        if calibration is None:
            print("Գծի սենսորների կալիբրացիան չի գտնվել։")
            print("Նախ կատարիր սենսորների կալիբրացիան։")
            return

        motors.initialize(board)
        motors_ready = True

        motors.motor_stop()

        gyro_offset = calibrate_gyro(bus)

        print("\nՌոբոտը պատրաստ է։")
        print("Սկսում եմ գծին հետևելը։")
        print(f"Երթուղի՝ {ROUTE}")
        print("R՝ աջ, L՝ ձախ, S՝ ուղիղ։")
        print("Anti-Bug-ը միացված է։")
        print("Դադարեցնելու համար՝ Ctrl+C։")

        while True:

            result = line_tracker.update(calibration)
            normalized = result["normalized"]

            if len(normalized) < 8:
                raise ValueError(
                    "Սպասվում են 8 սենսորների տվյալներ։"
                )

            s1_black = is_black(normalized[S1_INDEX])
            s8_black = is_black(normalized[S8_INDEX])

            # Խաչմերուկի հայտնաբերման վերագործարկում
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
                    print("Ռոբոտը կանգնած է։")
                    break

                direction = ROUTE[route_index]
                route_index += 1

                print(f"Ընտրված հրահանգ՝ {direction}")

                # =============== STRAIGHT ===============

                if direction == "S":
                    print("Շարժվում եմ ուղիղ...")

                    motors.set_speeds(
                        BASE_SPEED,
                        BASE_SPEED
                    )

                    time.sleep(STRAIGHT_TIME)

                    motors.motor_stop()
                    time.sleep(0.2)

                # =============== LEFT / RIGHT ===============

                else:
                    turn_direction = (
                        "LEFT"
                        if direction == "L"
                        else "RIGHT"
                    )

                    success = turn_80_degrees(
                        bus,
                        gyro_offset,
                        turn_direction
                    )

                    if not success:
                        print(
                            "Պտույտը չավարտվեց։ "
                            "Ռոբոտը կանգնած է։"
                        )
                        break

                    # =============== ANTIBUG ===============

                    line_found = check_line_after_turn(
                        calibration
                    )

                    if not line_found:
                        print(
                            "Anti-Bug: անհաջող պտույտ։ "
                            "Վերադառնում եմ հակառակ ուղղությամբ։"
                        )

                        opposite_direction = (
                            "RIGHT"
                            if turn_direction == "LEFT"
                            else "LEFT"
                        )

                        rollback_success = turn_80_degrees(
                            bus,
                            gyro_offset,
                            opposite_direction,
                            antibug=True
                        )

                        motors.motor_stop()

                        if not rollback_success:
                            print(
                                "Հետադարձ պտույտը չավարտվեց։ "
                                "Ռոբոտը կանգնած է։"
                            )
                            break

                        # Նույն հրամանը կրկին փորձելու համար
                        route_index = max(
                            0,
                            route_index - 1
                        )

                        print(
                            "Հրամանը պահպանվեց՝ "
                            f"{ROUTE[route_index]}"
                        )

                    else:
                        print(
                            "Anti-Bug: պտույտը հաջող է։"
                        )

                print(
                    "Հրամանի մշակումը ավարտվեց։ "
                    "Շարունակում եմ գծին հետևել։"
                )

                continue

            # =============== LINE FOLLOWING ===============

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
                f"\rPosition: {position_text} | "
                f"Direction: {direction_text} | "
                f"S1: {normalized[S1_INDEX]:.2f} | "
                f"S8: {normalized[S8_INDEX]:.2f} | "
                f"Խաչմերուկներ: {junction_count} | "
                f"Հաջորդ ROUTE: {next_route}   ",
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
        print("Ռոբոտը կանգնեցված է։")


if __name__ == "__main__":
    main()
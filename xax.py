
#!/usr/bin/env python3

import time
import sys
import termios
import tty
import select

from telemetrix import telemetrix
import motors


# =============== ԿԱՐԳԱՎՈՐՈՒՄՆԵՐ ===============

SERIAL_PORT = "/dev/serial0"

MAX_SPEED = 255
DRIVE_SPEED = 200
TURN_SPEED = 180

# Թարմացման հաճախականություն
UPDATE_INTERVAL = 0.03

# ==============================================


def clamp_speed(speed):
    return max(-255, min(255, int(speed)))


def set_motors(left, right):
    motors.set_speeds(
        clamp_speed(left),
        clamp_speed(right)
    )


def stop():
    motors.motor_stop()


def read_key():
    """Կարդում է ստեղնաշարի սեղմումը՝ առանց Enter-ի։"""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1).lower()
    return None


def print_controls():
    print("\n====================================")
    print("       ROBOT MANUAL CONTROL")
    print("====================================")
    print(" W  - Առաջ")
    print(" S  - Հետ")
    print(" A  - Ձախ")
    print(" D  - Աջ")
    print(" SPACE - Կանգ")
    print(" Q  - Դուրս գալ")
    print("------------------------------------")
    print("Սեղմիր կոճակը՝ ռոբոտը շարժելու համար։")
    print("Բաց թողնելուց հետո ռոբոտը կկանգնի։")
    print("====================================\n")


def main():
    board = None
    motors_ready = False

    old_settings = None

    # Այս փոփոխականը պահում է վերջին սեղմված կոճակը։
    current_key = None

    try:
        if not sys.stdin.isatty():
            print("Սխալ․ ծրագիրը պետք է գործարկվի տերմինալում։")
            return

        old_settings = termios.tcgetattr(sys.stdin.fileno())

        print("Միանում եմ Telemetrix-ին...")

        board = telemetrix.Telemetrix(
            com_port=SERIAL_PORT
        )

        motors.initialize(board)
        motors_ready = True

        stop()

        tty.setcbreak(sys.stdin.fileno())

        print_controls()

        while True:
            key = read_key()

            if key is not None:
                if key == "q":
                    print("\nԴուրս եմ գալիս...")
                    break

                elif key == "w":
                    current_key = "w"

                elif key == "s":
                    current_key = "s"

                elif key == "a":
                    current_key = "a"

                elif key == "d":
                    current_key = "d"

                elif key == " ":
                    current_key = None
                    stop()
                    print("\rԿԱՆԳ                 ")

                elif key == "\x03":
                    break

            # Եթե նոր կոճակ չի սեղմվել, շարժիչները կանգնեցնում ենք։
            # Սա կանխում է ռոբոտի անվերջ շարժվելը։
            if current_key is None:
                stop()

            elif current_key == "w":
                set_motors(DRIVE_SPEED, DRIVE_SPEED)

            elif current_key == "s":
                set_motors(-DRIVE_SPEED, -DRIVE_SPEED)

            elif current_key == "a":
                # Տեղում ձախ պտույտ
                set_motors(-TURN_SPEED, TURN_SPEED)

            elif current_key == "d":
                # Տեղում աջ պտույտ
                set_motors(TURN_SPEED, -TURN_SPEED)

            time.sleep(UPDATE_INTERVAL)

    except KeyboardInterrupt:
        print("\nԾրագիրը կանգնեցվում է։")

    except Exception as error:
        print(f"\nՍխալ՝ {error}")

    finally:
        if motors_ready:
            try:
                stop()
            except Exception:
                pass

        if old_settings is not None:
            try:
                termios.tcsetattr(
                    sys.stdin.fileno(),
                    termios.TCSADRAIN,
                    old_settings
                )
            except Exception:
                pass

        if board is not None:
            try:
                board.shutdown()
            except Exception:
                pass

        print("Ռոբոտը կանգնեցված է։")


if __name__ == "__main__":
    main()

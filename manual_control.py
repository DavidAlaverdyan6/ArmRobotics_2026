
#!/usr/bin/env python3

import sys
import select
import termios
import tty

from telemetrix import telemetrix
import motors

PORT = "/dev/serial0"

DEFAULT_SPEED = 120
MIN_SPEED = 40
MAX_SPEED = 255
SPEED_STEP = 15


def main():
    board = None
    old_settings = None
    speed = DEFAULT_SPEED

    try:
        if not sys.stdin.isatty():
            raise RuntimeError("Run this program in a terminal.")

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        board = telemetrix.Telemetrix(com_port=PORT)
        motors.initialize(board)
        motors.motor_stop()

        print("\nManual Robot Control")
        print("W - Forward")
        print("S - Backward")
        print("A - Turn left")
        print("D - Turn right")
        print("SPACE - Stop")
        print("+/- - Change speed")
        print("Q - Quit")
        print(f"Speed: {speed}")
        print("Movement continues until another key is pressed.\n")

        running = True

        while running:
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)

            if not ready:
                continue

            key = sys.stdin.read(1).lower()

            if key == "w":
                motors.set_speeds(speed, speed)
                print("Forward")

            elif key == "s":
                motors.set_speeds(-speed, -speed)
                print("Backward")

            elif key == "a":
                motors.set_speeds(-speed, speed)
                print("Left")

            elif key == "d":
                motors.set_speeds(speed, -speed)
                print("Right")

            elif key == " ":
                motors.motor_stop()
                print("STOP")

            elif key in ("+", "="):
                speed = min(MAX_SPEED, speed + SPEED_STEP)
                print(f"Speed: {speed}")

            elif key == "-":
                speed = max(MIN_SPEED, speed - SPEED_STEP)
                print(f"Speed: {speed}")

            elif key == "q":
                motors.motor_stop()
                running = False

    except KeyboardInterrupt:
        pass

    except Exception as error:
        print(f"Error: {error}")

    finally:
        if board is not None:
            try:
                motors.motor_stop()
                board.shutdown()
            except Exception:
                pass

        if old_settings is not None:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                old_settings
            )

        print("\nMotors stopped.")


if __name__ == "__main__":
    main()

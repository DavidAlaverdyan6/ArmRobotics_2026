
#!/usr/bin/env python3

import time
import curses
from telemetrix import telemetrix

import motors

PORT = "/dev/serial0"

# Adjust these to suit your robot.
DRIVE_SPEED = 200
TURN_SPEED = 130
LOOP_DELAY = 0.02

board = None


def stop():
    motors.motor_stop()



def drive(key):
    if key == ord("w"):
        motors.motor_forward(DRIVE_SPEED)

    elif key == ord("s"):
        motors.motor_reverse(DRIVE_SPEED)

    elif key == ord("a"):
        motors.motor_left(TURN_SPEED)

    elif key == ord("d"):
        motors.motor_right(TURN_SPEED)
    else:
        motors.motor_stop()




def controller(stdscr):
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)

    current_key = None

    while True:
        height, width = stdscr.getmaxyx()
        stdscr.erase()

        lines = [
            "ROBOT MANUAL DRIVE",
            "",
            "W - Forward",
            "S - Backward",
            "A - Left",
            "D - Right",
            "SPACE - Stop",
            "Q - Quit",
        ]

        for y, line in enumerate(lines):
            if y < height and width > 1:
                stdscr.addnstr(y, 0, line, width - 1)

        if height > len(lines) + 1:
            stdscr.addnstr(
                len(lines) + 1,
                0,
                f"Active: {chr(current_key).upper() if current_key else 'STOPPED'}",
                width - 1,
            )

        stdscr.refresh()

        key = stdscr.getch()

        if key == ord("q"):
            break

        elif key == ord(" "):
            current_key = None
            stop()

        elif key in (ord("w"), ord("a"), ord("s"), ord("d")):
            if key != current_key:
                current_key = key
                drive(key)

        elif key == -1:
            current_key = None
            stop()

        time.sleep(LOOP_DELAY)


def main():
    global board

    try:
        print("Connecting to STM32...")
        board = telemetrix.Telemetrix(com_port=PORT)

        time.sleep(2)

        motors.initialize(board)

        print("Manual drive ready.")
        curses.wrapper(controller)

    except KeyboardInterrupt:
        print("\nStopping robot.")

    finally:
        if board is not None:
            try:
                stop()
            except Exception:
                pass

            board.shutdown()


if __name__ == "__main__":
    main()

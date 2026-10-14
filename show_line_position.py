#!/usr/bin/env python3

import time
from telemetrix import telemetrix
import line_tracker


SERIAL_PORT = "/dev/serial0"
UPDATE_INTERVAL = 0.1


def show_position(state):
    position = state["position"]

    print("\033[2J\033[H", end="")  # Clear terminal

    print("=" * 48)
    print("       ARMROBOTICS - LINE POSITION")
    print("=" * 48)

    print("\nRAW SENSOR VALUES:")
    print(" ".join(
        f"S{i + 1}:{value:4}"
        for i, value in enumerate(state["raw"])
    ))

    print("\nNORMALIZED SENSOR VALUES:")
    values = state["normalized"]

    for i, value in enumerate(values):
        bar = "#" * round(value * 20)
        print(f"S{i + 1}: [{bar:<20}] {value:.2f}")

    print("\nLINE POSITION:")

    if position is None:
        print("LINE NOT DETECTED")
    else:
        # Position weights in line_tracker.py range from -3 to +4.
        print(f"Position value: {position:+.3f}")

        width = 41
        marker = round((position + 3) / 7 * (width - 1))
        marker = max(0, min(width - 1, marker))

        bar = ["-"] * width
        bar[marker] = "|"

        print("-3 " + "".join(bar) + " +4")
        print("    " + " " * marker + "^")

    print("\nLINE DETECTED:", state["line_detected"])
    print("CROSSING:", state["crossing"] or "None")
    print("\nPress Ctrl+C to exit.")


def main():
    board = None

    try:
        print("Connecting to STM32...")

        board = telemetrix.Telemetrix(
            com_port=SERIAL_PORT
        )

        time.sleep(2)

        print("Initializing line sensors...")
        line_tracker.initialize(board)

        # Allow the first sensor readings to arrive.
        time.sleep(1)

        calibration = line_tracker.load_calibration()

        if calibration is None:
            print("\nCalibration file not found.")
            print("Starting calibration.")

            calibration = line_tracker.calibrate()

            print("Calibration saved.")

        print("\nReading line position...")
        time.sleep(0.5)

        while True:
            state = line_tracker.update(calibration)
            show_position(state)
            time.sleep(UPDATE_INTERVAL)

    except KeyboardInterrupt:
        print("\nStopping...")

    except Exception as error:
        print(f"\nError: {error}")
        raise

    finally:
        line_tracker.shutdown()

        if board is not None:
            try:
                board.shutdown()
            except Exception:
                pass

        print("Disconnected.")


if __name__ == "__main__":
    main()

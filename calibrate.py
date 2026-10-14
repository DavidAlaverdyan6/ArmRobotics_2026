#!/usr/bin/env python3

import time
from telemetrix import telemetrix
import line_tracker


SERIAL_PORT = "/dev/serial0"
CONNECT_DELAY = 4


def main():
    board = None

    try:
        print("=" * 45)
        print("       ARMROBOTICS - CALIBRATION")
        print("=" * 45)

        # Connect to STM32
        print(f"\nConnecting to STM32 on {SERIAL_PORT}...")

        board = telemetrix.Telemetrix(
            com_port=SERIAL_PORT
        )

        time.sleep(CONNECT_DELAY)

        print("STM32 connection initialized.")

        # Initialize line sensors using the shared board
        print("Initializing line sensors...")
        line_tracker.initialize(board)

        # Wait for sensor readings
        time.sleep(1)

        # Check existing calibration
        existing = line_tracker.load_calibration()

        if existing is not None:
            print("\nExisting calibration found.")

            answer = input(
                "Recalibrate sensors? [y/N]: "
            ).strip().lower()

            if answer not in ("y", "yes"):
                print("Keeping existing calibration.")
                return

        print("\nCalibration instructions:")
        print("1. Place ALL 8 sensors over a WHITE surface.")
        print("2. Keep the sensors still and press ENTER.")
        print("3. Place ALL 8 sensors over a BLACK line/surface.")
        print("4. Keep the sensors still and press ENTER.")
        print("\nDo not move the sensors while sampling.")

        # Perform and save calibration
        calibration = line_tracker.calibrate()

        print("\nCalibration completed successfully!")
        print("White values:", calibration["white"])
        print("Black values:", calibration["black"])

        print(
            "\nCalibration file:",
            line_tracker.CALIBRATION_FILE
        )

        print("You can now run line_follow.py.")

    except KeyboardInterrupt:
        print("\nCalibration cancelled.")

    except Exception as error:
        print(f"\nCalibration error: {error}")
        raise

    finally:
        # Release tracker reference and close the board
        line_tracker.shutdown()

        if board is not None:
            try:
                board.shutdown()
            except Exception:
                pass

        print("Disconnected.")


if __name__ == "__main__":
    main()

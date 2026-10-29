
#!/usr/bin/env python3

import time
from telemetrix import telemetrix

import line_tracker
import motors


PORT = "/dev/serial0"

BASE_SPEED = 100
MAX_SPEED = 170
MIN_SPEED = -80

# Start with proportional steering only.
KP = 30.0

# Your existing sensor weights are [-3, -2, -1, 0, 1, 2, 3, 4].
# Tested centered-line position from show_line_position.py: about +0.08.
TARGET_POSITION = 0.08

LOOP_DELAY = 0.04


def clamp(value, low, high):
    return max(low, min(high, value))


def main():
    board = None

    try:
        board = telemetrix.Telemetrix(com_port=PORT)

        motors.initialize(board)
        line_tracker.initialize(board)

        calibration = line_tracker.load_calibration()
        if calibration is None:
            print("No calibration found. Calibrate the sensors first.")
            return

        print("Differential steering active. Ctrl+C to stop.")

        while True:
            result = line_tracker.update(calibration)

            if result is None or result["position"] is None:
                motors.motor_stop()
                time.sleep(LOOP_DELAY)
                continue

            position = result["position"]

            # Positive error means the line is to the right
            # of the target position.
            error = position - TARGET_POSITION
            correction = KP * error

            left_speed = clamp(
                BASE_SPEED + correction,
                MIN_SPEED,
                MAX_SPEED,
            )
            right_speed = clamp(
                BASE_SPEED - correction,
                MIN_SPEED,
                MAX_SPEED,
            )

            motors.set_speeds(
                round(left_speed),
                round(right_speed),
            )

            print(
                f"Position: {position:+.2f} | "
                f"Error: {error:+.2f} | "
                f"L: {left_speed:6.1f} | "
                f"R: {right_speed:6.1f}",
                end="\r",
            )

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nStopping.")

    finally:
        if board is not None:
            try:
                motors.motor_stop()
                line_tracker.shutdown()
            finally:
                board.shutdown()


if __name__ == "__main__":
    main()

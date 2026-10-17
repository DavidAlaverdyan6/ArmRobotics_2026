#!/usr/bin/env python3

import time
import telemetrix
import line_tracker

POLL_INTERVAL = 0.2

def main():
    board = telemetrix.Telemetrix()
    calibration = line_tracker.load_calibration()

    if calibration is None:
        print("Calibration not found.")
        print("Run calibration first.")
        board.shutdown()
        return

    try:
        line_tracker.initialize(board)
        print("Line tracker test started.")
        print("Move the sensor array over the line.")
        print("Press Ctrl+C to exit.\n")

        while True:
            state = line_tracker.update(calibration)

            values = state["normalized"]
            position = state["position"]
            crossing = state["crossing"]

            if position is None:
                direction = "LINE LOST"
            elif position < -0.5:
                direction = "LEFT"
            elif position > 0.5:
                direction = "RIGHT"
            else:
                direction = "STRAIGHT"

            print(
                f"\rSensors: {[round(v, 2) for v in values]} | "
                f"Position: {position if position is not None else 'None'} | "
                f"Direction: {direction} | "
                f"Crossing: {crossing or '-'}",
                end="",
                flush=True,
            )

            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print("\nTest stopped.")

    finally:
        line_tracker.shutdown()
        board.shutdown()


if __name__ == "__main__":
    main()
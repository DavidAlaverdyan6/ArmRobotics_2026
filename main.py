
import time
from telemetrix import telemetrix
import robot_pin_config as robot_pins
import line_tracker
import motors

PORT = "/dev/serial0"

BASE_SPEED = 110
LOOP_DELAY = 0.04
LOST_LINE_LIMIT = 10

# Assumes normalized sensor values are higher when detecting the line.
LINE_THRESHOLD = 0.5

board = None


def main():
    global board
    board = telemetrix.Telemetrix(com_port=PORT)

    try:
        motors.initialize(board)
        line_tracker.initialize(board)

        calibration = line_tracker.load_calibration()
        if calibration is None:
            print("No line-sensor calibration found.")
            print("Calibrate the sensors before running the robot.")
            return

        lost_count = 0
        print("Binary line follower running. Ctrl+C to stop.")

        while True:
            result = line_tracker.update(calibration)

            if result is None:
                motors.motor_stop()
                time.sleep(LOOP_DELAY)
                continue

            normalized = result["normalized"]

            # LINE_SENSORS is physically reversed in the tracker.
            # After reversal, index 0 should be the leftmost sensor.
            left_detected = any(
                value > LINE_THRESHOLD for value in normalized[:4]
            )
            right_detected = any(
                value > LINE_THRESHOLD for value in normalized[4:]
            )

            if left_detected or right_detected:
                lost_count = 0
            else:
                lost_count += 1
                motors.motor_stop()

                if lost_count >= LOST_LINE_LIMIT:
                    print("Line lost. Stopping.")
                    break

                time.sleep(LOOP_DELAY)
                continue

            if left_detected and right_detected:
                # Line under both halves: proceed straight.
                motors.motor_forward(BASE_SPEED)

            elif left_detected:
                # Line detected on the left half.
                motors.motor_left(BASE_SPEED)

            else:
                # Line detected on the right half.
                motors.motor_right(BASE_SPEED)

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nStopping robot.")

    finally:
        if board is not None:
            try:
                motors.motor_stop()
            except Exception:
                pass
            board.shutdown()


if __name__ == "__main__":
    main()

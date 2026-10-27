import time
from telemetrix.telemetrix import Telemetrix

import line_tracker
import robot_pin_config as pins


# ================= CONFIGURATION =================

SERIAL_PORT = "/dev/serial0"
UPDATE_INTERVAL = 0.05

# Sensor positions
LEFT_SENSOR = 0
CENTER_SENSOR = 3
RIGHT_SENSOR = 2

DEADBAND = 0.25
STRONG_TURN = 1.8

# ==================================================

board = None


def motor_setup():
    motor_pins = (
        pins.motorl1,
        pins.motorl2,
        pins.motorr1,
        pins.motorr2,
    )

    for pin in motor_pins:
        board.set_pin_mode_digital_output(pin)

    time.sleep(0.2)
    motors_stop()


def motor_a_forward():
    board.digital_write(pins.motorl1, 1)
    board.digital_write(pins.motorl2, 0)


def motor_b_forward():
    board.digital_write(pins.motorr1, 0)
    board.digital_write(pins.motorr2, 1)

def motor_a_back():
    board.digital_write(pins.motorl1, 0)
    board.digital_write(pins.motorl2, 1)


def motor_b_back():
    board.digital_write(pins.motorr1, 1)
    board.digital_write(pins.motorr2, 0)


def motor_a_stop():
    board.digital_write(pins.motorl1, 0)
    board.digital_write(pins.motorl2, 0)


def motor_b_stop():
    board.digital_write(pins.motorr1, 0)
    board.digital_write(pins.motorr2, 0)


def motors_stop():
    motor_a_stop()
    motor_b_stop()


def follow_line(position):
    if position is None:
        motor_a_back()
        motor_b_back()
        return "LINE LOST - STOP"


    if 2.8 < abs(position) and abs(position) < 3.2:
        motor_a_forward()
        motor_b_forward()
        return "FORWARD"

    if abs(position) <= 2.2:
        motor_b_forward()
        motor_a_back()
        return "STRONG LEFT"



    if abs(position) >= 3.8:
        motor_b_back()
        motor_a_forward()
        return "STRONG RIGHT"




 
    return "FORWARD"


def main():
    global board

    try:
        print("Connecting to STM32...")

        board = Telemetrix(com_port=SERIAL_PORT)
        time.sleep(2)

        print("Initializing 8 line sensors...")
        line_tracker.initialize(board)

        motor_setup()

        calibration = line_tracker.load_calibration()

        if calibration is None:
            print("Calibration not found.")
            motors_stop()
            calibration = line_tracker.calibrate()

        print("\nLine follower started!")
        print("Press Ctrl+C to stop.\n")

        while True:
            state = line_tracker.update(calibration)

            position = state.get("position")
            action = follow_line(position)

            position_text = (
                "NOT DETECTED"
                if position is None
                else f"{position:+.3f}"
            )

            print(
                f"\rPosition: {position_text:>12} | "
                f"Action: {action:<20}",
                end="",
                flush=True,
            )

            time.sleep(UPDATE_INTERVAL)

    except KeyboardInterrupt:
        print("\nStopping robot...")

    finally:
        if board is not None:
            try:
                motors_stop()
            except Exception:
                pass

            try:
                board.shutdown()
            except Exception:
                pass

        try:
            line_tracker.shutdown()
        except Exception:
            pass

        print("Motors stopped. Disconnected.")


if __name__ == "__main__":
    main()
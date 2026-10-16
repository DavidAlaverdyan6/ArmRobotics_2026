
#!/usr/bin/env python3

import time
from telemetrix import telemetrix

import line_tracker
import robot_pin_config as pins


# ================= CONFIGURATION =================

SERIAL_PORT = "/dev/serial0"
UPDATE_INTERVAL = 0.05

# Position deadband
DEADBAND = 0.25

# Strong deviation threshold
STRONG_TURN = 1.8

# ==================================================


board = None


def motor_setup():
    """Configure Motor A and Motor B direction pins."""

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
    """Left motor forward."""
    board.digital_write(pins.motorl1, 1)
    board.digital_write(pins.motorl2, 0)


def motor_b_forward():
    """Right motor forward."""
    board.digital_write(pins.motorr1, 1)
    board.digital_write(pins.motorr2, 0)

def motor_b_back():
    """Right motor forward."""
    board.digital_write(pins.motorr1, 0)
    board.digital_write(pins.motorr2, 1)

def motor_a_back():
    """Right motor forward."""
    board.digital_write(pins.motorr1, 0)
    board.digital_write(pins.motorr2, 1)

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
    """
    Positive position -> steer left.
    Negative position -> steer right.

    This version uses digital ON/OFF steering,
    because the current pin configuration does not
    define separate motor-enable/PWM pins.
    """

    if position is None:
        motor_a_back()
        motor_b_back()
        return "LINE LOST - STOP"

    if abs(position) <= DEADBAND:
        motor_a_forward()
        motor_b_forward()
        return "FORWARD"

    if position >= STRONG_TURN:
        # Strong left correction
        motor_b_stop()
        motor_a_forward()
        return "STRONG LEFT"

    if position > DEADBAND:
        # Mild left correction
        motor_b_stop()
        motor_a_forward()
        return "LEFT"

    if position <= -STRONG_TURN:
        # Strong right correction
        motor_b_forward()
        motor_a_stop()
        return "STRONG RIGHT"

    if position < -DEADBAND:
        # Mild right correction
        motor_b_forward()
        motor_a_stop()
        return "RIGHT"

    motor_a_forward()
    motor_b_forward()
    return "FORWARD"


def main():
    global board

    try:
        print("Connecting to STM32...")

        board = telemetrix.Telemetrix(
            com_port=SERIAL_PORT
        )

        time.sleep(2)

        print("Initializing line sensors...")
        line_tracker.initialize(board)

        print("Initializing Motor A and Motor B...")
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

            position = state["position"]
            action = follow_line(position)

            if position is None:
                position_text = "NOT DETECTED"
            else:
                position_text = f"{position:+.3f}"

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

        line_tracker.shutdown()

        if board is not None:
            try:
                board.shutdown()
            except Exception:
                pass

        print("Motors stopped. Disconnected.")


if __name__ == "__main__":
    main()

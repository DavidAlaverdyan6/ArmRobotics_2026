#!/usr/bin/env python3
"""Standalone 8-channel line-following PID test for Post Integral.

The competition main.py uses forward_line_pid.ForwardLinePID with the same
simple_pid settings. The standalone test remains independently runnable.
Only use this standalone test with the robot in a safe, clear test area.
"""

import math
import time

from simple_pid import PID
from telemetrix import telemetrix

import line_tracker
import motors
from navigation_core import FreshFrames
from stm32_reset import reset_stm32


PORT = "/dev/serial0"

# Initial tuning guesses, NOT measured/validated gains.
BASE_SPEED = 255
MAX_SPEED = 255
MIN_SPEED = 0  # During this standalone forward PID test, never reverse a wheel.
KP = 100     # PWM per normalized sensor-position unit
KI = 0     # PWM per position-unit-second
KD = 5     # PWM-seconds per position unit
MAX_CORRECTION = 255
TARGET_POSITION = line_tracker.CENTER_POSITION  # 0.0 for the 8 symmetric weights
LOOP_DELAY = 0.04
REPORT_INTERVAL = 0.25
SENSOR_STARTUP_TIMEOUT = 6.0


def clamp(value, low, high):
    return max(low, min(high, value))


class LinePID:
    """Translate the library's setpoint-minus-measurement sign to motor PWM."""

    def __init__(self, kp=KP, ki=KI, kd=KD, target=TARGET_POSITION):
        self.pid = PID(
            kp, ki, kd,
            setpoint=target,
            sample_time=None,  # Use actual elapsed monotonic time per NEW sensor frame.
            output_limits=(-MAX_CORRECTION, MAX_CORRECTION),
            differential_on_measurement=True,
        )

    def reset(self):
        self.pid.reset()

    def command(self, position, speed=BASE_SPEED, max_speed=MAX_SPEED):
        if position is None or not math.isfinite(position):
            self.reset()
            return None
        # For line position > target (to robot's right), left motor must be
        # faster. simple-pid gives a negative output; invert it for steering.
        correction = -self.pid(position)
        left = round(clamp(speed + correction, MIN_SPEED, max_speed))
        right = round(clamp(speed - correction, MIN_SPEED, max_speed))
        return left, right, correction

    @property
    def terms(self):
        return self.pid.components


def main():
    board = None
    sensors_ready = False
    motors_ready = False
    controller = LinePID()
    last_report = 0.0
    freshness = FreshFrames()

    try:
        reset_stm32()
        board = telemetrix.Telemetrix(com_port=PORT)

        # Match ADC.py exactly: register analog inputs immediately after
        # connection, before configuring/writing any motor outputs.
        # Only enable the motor driver after all eight inputs report.
        line_tracker.initialize(board)
        sensors_ready = True

        calibration = line_tracker.load_calibration()
        if calibration is None:
            print("No valid 8-channel calibration found. Run python calibrate.py.")
            return

        print("Waiting for all eight analog channels...")
        if not line_tracker.wait_for_callbacks(timeout=SENSOR_STARTUP_TIMEOUT):
            state = line_tracker.update(calibration)
            print("ADC callback timeout. Missing channels:",
                  [i + 1 for i, ready in enumerate(line_tracker.callback_status())
                   if not ready])
            print("Configured pins:", line_tracker.SENSOR_PINS)
            print("Callback counts:", state["sample_ids"])
            print("Latest raw values:", state["raw"])
            print("No motor outputs have been initialized; robot stays stopped.")
            return
        if not line_tracker.all_channels_fresh():
            state = line_tracker.update(calibration)
            print("Sensor readings are stale or unsynchronized. Motors kept stopped.")
            print("Callback counts:", state["sample_ids"])
            print("Latest raw values:", state["raw"])
            return

        # Motor outputs are configured only after successful analog pairing.
        motors.initialize(board)
        motors_ready = True
        motors.motor_stop()

        print(
            f"PID ready: target={TARGET_POSITION:.2f} "
            f"Kp={KP:g} Ki={KI:g} Kd={KD:g} base={BASE_SPEED}. Ctrl+C to stop."
        )
        while True:
            state = line_tracker.update(calibration)
            now = time.monotonic()
            if not freshness.accept(state, now):
                # Repeated snapshot: do not integrate it or derive a false D term.
                # If sensor freshness is actually lost, stop immediately.
                if not line_tracker.all_channels_fresh():
                    motors.motor_stop()
                    controller.reset()
                time.sleep(LOOP_DELAY)
                continue

            position = state["position"]
            command = controller.command(position)
            if command is None:
                motors.motor_stop()
                time.sleep(LOOP_DELAY)
                continue

            left, right, correction = command
            motors.set_speeds(left, right)

            if now - last_report >= REPORT_INTERVAL:
                p, i, d = controller.terms
                print(
                    f"pos={position:+.2f} correction={correction:+.1f} "
                    f"P={p:+.1f} I={i:+.1f} D={d:+.1f} "
                    f"L={left:3d} R={right:3d}"
                )
                last_report = now
            time.sleep(LOOP_DELAY)
    except KeyboardInterrupt:
        print("\\nStopped by user.")
    finally:
        if board is not None:
            try:
                # If ADC pairing failed before motors.initialize(), do not
                # send motor writes to an uninitialized motor driver.
                if motors_ready:
                    motors.motor_stop()
            finally:
                try:
                    if sensors_ready:
                        line_tracker.shutdown()
                finally:
                    board.shutdown()


if __name__ == "__main__":
    main()

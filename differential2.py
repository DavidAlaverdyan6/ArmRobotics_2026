#!/usr/bin/env python3
"""Standalone 8-channel line-following PID test for Post Integral.

The competition main_test.py uses line_tracker.LineTrackingEngine separately.
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
BASE_SPEED = 100
MAX_SPEED = 170
MIN_SPEED = 0  # During this standalone forward PID test, never reverse a wheel.
KP = 30.0     # PWM per normalized sensor-position unit
KI = 0.5      # PWM per position-unit-second
KD = 0.4      # PWM-seconds per position unit
MAX_CORRECTION = 60.0
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
    controller = LinePID()
    last_report = 0.0
    freshness = FreshFrames()

    try:
        reset_stm32()
        board = telemetrix.Telemetrix(com_port=PORT)
        motors.initialize(board)
        motors.motor_stop()
        line_tracker.initialize(board)
        sensors_ready = True

        calibration = line_tracker.load_calibration()
        if calibration is None:
            print("No valid 8-channel calibration found. Run python calibrate.py.")
            return

        print("Waiting for all eight analog channels...")
        if not line_tracker.wait_for_callbacks(timeout=SENSOR_STARTUP_TIMEOUT):
            print("ADC callback timeout. Missing channels:",
                  [i + 1 for i, ready in enumerate(line_tracker.callback_status())
                   if not ready])
            return
        if not line_tracker.all_channels_fresh():
            print("Sensor readings are stale or unsynchronized. Motors kept stopped.")
            return

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
                motors.motor_stop()
            finally:
                try:
                    if sensors_ready:
                        line_tracker.shutdown()
                finally:
                    board.shutdown()


if __name__ == "__main__":
    main()

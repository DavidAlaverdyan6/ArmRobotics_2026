#!/usr/bin/env python3
"""The exact forward-steering PID policy validated in differential2.py.

Uses the same simple_pid settings, signs, correction limits and wheel
clamping, but does not import or execute the standalone differential2 script.
"""
import math

from simple_pid import PID

BASE_SPEED = 255
MAX_SPEED = 255
MIN_SPEED = 0
KP = 100
KI = 0
KD = 5
MAX_CORRECTION = 255


def clamp(value, low, high):
    return max(low, min(high, value))


class ForwardLinePID:
    def __init__(self, motor_driver, target=0.0):
        self.motor_driver = motor_driver
        self.pid = PID(
            KP, KI, KD,
            setpoint=target,
            sample_time=None,
            output_limits=(-MAX_CORRECTION, MAX_CORRECTION),
            differential_on_measurement=True,
        )

    def reset(self):
        self.pid.reset()

    def command(self, position):
        if position is None or not math.isfinite(position):
            self.reset()
            return None
        correction = -self.pid(position)
        left = round(clamp(BASE_SPEED + correction, MIN_SPEED, MAX_SPEED))
        right = round(clamp(BASE_SPEED - correction, MIN_SPEED, MAX_SPEED))
        return left, right, correction

    def drive(self, state):
        position = state.get('position') if state else None
        command = self.command(position)
        if command is None:
            self.motor_driver.motor_stop()
            return False
        left, right, _ = command
        self.motor_driver.set_speeds(left, right)
        return True

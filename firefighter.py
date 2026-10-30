"""Flame input and pump using the robot's existing Telemetrix connection."""

import time

import robot_pin_config as pins

# STM32 firmware uses 12-bit ADC readings. Hardware observation:
# 0 means no flame; readings of 2000-2500 indicate flame (active-high).
FIRE_THRESHOLD = 2000
FIRE_ACTIVE_LOW = False
SENSOR_MAX_AGE = 0.5
WATER_SECONDS = 5.0
FLAME_CONFIRM_SECONDS = 1.0
SENSOR_WAIT_TIMEOUT = 2.0
POLL_SECONDS = 0.02


def firefighting_pose():
    """Photo pose: upper link +90°, forearm +10°, tool -25° to horizontal.

    These are physical link orientations, not raw servo commands. The user
    chose this pose over unreachable tip XYZ=(250, 0, 200), pitch=-45°.
    """
    from arm_ik import (HOME_BASE, HOME_SHOULDER, HOME_ELBOW, HOME_WRIST,
                        HOME_SHOULDER_PITCH, HOME_ELBOW_BEND, HOME_TOOL_PITCH,
                        WRIST_DEGREES_PER_SERVO, checked_angle)
    q1, q2, pitch = 90.0, 10.0, -25.0
    return {
        "base": HOME_BASE,
        "shoulder": checked_angle("Shoulder", HOME_SHOULDER + q1 - HOME_SHOULDER_PITCH),
        "elbow": checked_angle("Elbow", HOME_ELBOW - (q2 - q1 - HOME_ELBOW_BEND)),
        "wrist": checked_angle("Wrist", HOME_WRIST + (
            pitch - q2 - HOME_TOOL_PITCH + HOME_SHOULDER_PITCH + HOME_ELBOW_BEND
        ) / WRIST_DEGREES_PER_SERVO),
    }


class Firefighter:
    def __init__(self, board):
        self.board = board
        self.sample = None

    def initialize(self):
        self.board.set_pin_mode_digital_output(pins.pump)
        self.off()
        # PB0=200 is an ADC pin in this firmware. differential=0 provides
        # continuous reports, including unchanged zero readings.
        self.board.set_pin_mode_analog_input(
            pins.fire_sensor, differential=0, callback=self._callback
        )

    def _callback(self, data):
        try:
            value = int(data[2])
            if data[1] == pins.fire_sensor and 0 <= value <= 4095:
                self.sample = (value, time.monotonic())
        except (IndexError, TypeError, ValueError):
            pass

    def detected(self):
        sample = self.sample
        if sample is None or time.monotonic() - sample[1] > SENSOR_MAX_AGE:
            raise RuntimeError("F: flame sensor has no fresh reading")
        value = sample[0]
        return value <= FIRE_THRESHOLD if FIRE_ACTIVE_LOW else value >= FIRE_THRESHOLD

    def off(self):
        self.board.digital_write(pins.pump, 0)

    def wait_for_sample(self, after=None):
        deadline = time.monotonic() + SENSOR_WAIT_TIMEOUT
        while time.monotonic() < deadline:
            sample = self.sample
            if (sample is not None
                    and (after is None or sample[1] > after)
                    and time.monotonic() - sample[1] <= SENSOR_MAX_AGE):
                return
            time.sleep(POLL_SECONDS)
        raise RuntimeError("F: timed out waiting for a new flame reading")

    def confirm_flame(self):
        """Require one uninterrupted second of flame while the base is stopped."""
        start = time.monotonic()
        while True:
            if not self.detected():
                return False
            if time.monotonic() - start >= FLAME_CONFIRM_SECONDS:
                return True
            time.sleep(POLL_SECONDS)

    def spray(self):
        try:
            self.board.digital_write(pins.pump, 1)
            time.sleep(WATER_SECONDS)
        finally:
            self.off()

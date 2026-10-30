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
SENSOR_WAIT_REPORT_SECONDS = 2.0
POLL_SECONDS = 0.02


def firefighting_pose():
    """Servo angles for the F task, including the closed gripper."""
    return {
        "base": 90,
        "shoulder": 90,
        "elbow": 160,
        "wrist": 70,
        "gripper": 180,
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
            return None
        value = sample[0]
        return value <= FIRE_THRESHOLD if FIRE_ACTIVE_LOW else value >= FIRE_THRESHOLD

    def off(self):
        self.board.digital_write(pins.pump, 0)

    def wait_for_sample(self, after=None):
        """Wait without a deadline; callers keep the drive stopped meanwhile."""
        last_report = time.monotonic()
        while True:
            sample = self.sample
            now = time.monotonic()
            if (sample is not None
                    and (after is None or sample[1] > after)
                    and now - sample[1] <= SENSOR_MAX_AGE):
                return
            if now - last_report >= SENSOR_WAIT_REPORT_SECONDS:
                print("F: waiting for a fresh flame sensor reading; motors stopped.")
                last_report = now
            time.sleep(POLL_SECONDS)

    def confirm_flame(self):
        """Require one uninterrupted second of flame while the base is stopped."""
        start = time.monotonic()
        while True:
            # Missing or stale reports cannot count toward confirmation.
            if self.detected() is not True:
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

"""Flame input and pump using the robot's existing Telemetrix connection."""

import time

import robot_pin_config as pins

# STM32 firmware uses 12-bit ADC readings. Verify these two settings with
# the actual flame sensor: its output type/polarity is not documented in repo.
FIRE_THRESHOLD = 2000
FIRE_ACTIVE_LOW = True
SENSOR_MAX_AGE = 0.5
WATER_SECONDS = 5.0


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

    def spray(self):
        try:
            self.board.digital_write(pins.pump, 1)
            time.sleep(WATER_SECONDS)
        finally:
            self.off()

"""VL53L0X distance sensor over Raspberry Pi I2C."""

import board
import busio
import adafruit_vl53l0x

DISTANCE_OFFSET_MM = 25


class VL53L0XSensor:
    def __init__(self, offset_mm=DISTANCE_OFFSET_MM):
        self.offset_mm = offset_mm
        self.i2c = None
        self.sensor = None

    def initialize(self):
        if self.sensor is not None:
            return

        self.i2c = busio.I2C(board.SCL, board.SDA)
        try:
            self.sensor = adafruit_vl53l0x.VL53L0X(self.i2c)
        except Exception:
            self.close()
            raise

    def read_distance_mm(self):
        """Return corrected distance in mm, or raise if not initialized."""
        if self.sensor is None:
            raise RuntimeError("Call initialize() before reading distance.")

        raw_mm = int(self.sensor.range)
        return max(0, raw_mm - self.offset_mm)

    def read_raw_distance_mm(self):
        """Return the uncorrected sensor reading in mm."""
        if self.sensor is None:
            raise RuntimeError("Call initialize() before reading distance.")

        return int(self.sensor.range)

    def close(self):
        self.sensor = None

        if self.i2c is not None:
            try:
                self.i2c.deinit()
            except (AttributeError, OSError, RuntimeError):
                pass
            self.i2c = None


if __name__ == "__main__":
    # Optional standalone diagnostic; never runs on import.
    import time

    sensor = VL53L0XSensor()
    try:
        sensor.initialize()
        while True:
            raw = sensor.read_raw_distance_mm()
            corrected = max(0, raw - DISTANCE_OFFSET_MM)
            print(f"Raw: {raw:4d} mm | Corrected: {corrected:4d} mm")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        sensor.close()

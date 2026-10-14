#!/usr/bin/env python3
"""MPU6050 orientation sensor for Raspberry Pi I2C."""

import math
import time

import smbus

I2C_BUS_NUMBER = 1
MPU6050_ADDRESS = 0x68

REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_PWR_MGMT_1 = 0x6B
REG_WHO_AM_I = 0x75

GYRO_SCALE = 131.0
ACCEL_SCALE = 16384.0
SAMPLE_RATE_DIVIDER = 4
COMPLEMENTARY_ALPHA = 0.98

CALIBRATION_SAMPLES = 600
CALIBRATION_INTERVAL = 0.005
MAX_DT = 0.1
ACCEL_GRAVITY_MIN = 0.75
ACCEL_GRAVITY_MAX = 1.25


def _signed_16(high, low):
    value = (high << 8) | low
    return value - 65536 if value >= 32768 else value


def accelerometer_angles(ax, ay, az):
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(
        math.atan2(-ax, math.sqrt(ay * ay + az * az))
    )
    return roll, pitch


def normalize_angle(angle):
    return (angle + 180.0) % 360.0 - 180.0


class MPU6050:
    def __init__(self, bus_number=I2C_BUS_NUMBER, address=MPU6050_ADDRESS):
        self.bus_number = bus_number
        self.address = address
        self.bus = None
        self.gyro_offsets = [0.0, 0.0, 0.0]
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.angle_z = 0.0
        self.last_time = None
        self.initialized = False

    def _read_register(self, register):
        return self.bus.read_byte_data(self.address, register)

    def _write_register(self, register, value):
        self.bus.write_byte_data(self.address, register, value)

    def _read_raw_axes(self):
        data = self.bus.read_i2c_block_data(
            self.address, REG_ACCEL_XOUT_H, 14
        )
        return (
            _signed_16(data[0], data[1]),
            _signed_16(data[2], data[3]),
            _signed_16(data[4], data[5]),
            _signed_16(data[8], data[9]),
            _signed_16(data[10], data[11]),
            _signed_16(data[12], data[13]),
        )

    def _calibrate_gyroscope(self):
        """Keep the sensor stationary while this finite calibration runs."""
        time.sleep(0.5)
        sums = [0.0, 0.0, 0.0]

        for _ in range(CALIBRATION_SAMPLES):
            _, _, _, gx, gy, gz = self._read_raw_axes()
            sums[0] += gx
            sums[1] += gy
            sums[2] += gz
            time.sleep(CALIBRATION_INTERVAL)

        self.gyro_offsets = [
            value / CALIBRATION_SAMPLES for value in sums
        ]

    def initialize(self, calibrate=True):
        """Open I2C, configure the chip and establish initial orientation."""
        if self.initialized:
            return

        self.bus = smbus.SMBus(self.bus_number)
        try:
            self._write_register(REG_PWR_MGMT_1, 0x01)
            time.sleep(0.1)

            identity = self._read_register(REG_WHO_AM_I)
            if identity not in (0x68, 0x69):
                raise RuntimeError(
                    f"Unexpected MPU6050 identity: 0x{identity:02X}"
                )

            self._write_register(REG_CONFIG, 0x03)
            self._write_register(REG_SMPLRT_DIV, SAMPLE_RATE_DIVIDER)
            self._write_register(REG_GYRO_CONFIG, 0x00)
            self._write_register(REG_ACCEL_CONFIG, 0x00)
            time.sleep(0.1)

            if calibrate:
                self._calibrate_gyroscope()

            ax, ay, az, _, _, _ = self._read_raw_axes()
            self.angle_x, self.angle_y = accelerometer_angles(ax, ay, az)
            self.angle_z = 0.0
            self.last_time = time.monotonic()
            self.initialized = True

        except Exception:
            self.close()
            raise

    def update(self):
        """
        Read one sample and return current orientation.

        Returns a dict containing roll, pitch, yaw and dt.
        If called too long after the previous sample, it re-baselines timing
        rather than integrating a large erroneous interval.
        """
        if not self.initialized:
            raise RuntimeError("Call initialize() before update().")

        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = (
            self._read_raw_axes()
        )

        if dt <= 0.0 or dt > MAX_DT:
            return {
                "roll": self.angle_x,
                "pitch": self.angle_y,
                "yaw": self.angle_z,
                "dt": dt,
            }

        gx = (gx_raw - self.gyro_offsets[0]) / GYRO_SCALE
        gy = (gy_raw - self.gyro_offsets[1]) / GYRO_SCALE
        gz = (gz_raw - self.gyro_offsets[2]) / GYRO_SCALE

        gyro_roll = self.angle_x + gx * dt
        gyro_pitch = self.angle_y + gy * dt
        self.angle_z = normalize_angle(self.angle_z + gz * dt)

        magnitude_g = (
            math.sqrt(ax_raw**2 + ay_raw**2 + az_raw**2) / ACCEL_SCALE
        )
        accel_usable = ACCEL_GRAVITY_MIN <= magnitude_g <= ACCEL_GRAVITY_MAX

        if accel_usable:
            accel_roll, accel_pitch = accelerometer_angles(
                ax_raw, ay_raw, az_raw
            )
            alpha = COMPLEMENTARY_ALPHA
            self.angle_x = alpha * gyro_roll + (1 - alpha) * accel_roll
            self.angle_y = alpha * gyro_pitch + (1 - alpha) * accel_pitch
        else:
            self.angle_x = gyro_roll
            self.angle_y = gyro_pitch

        return {
            "roll": self.angle_x,
            "pitch": self.angle_y,
            "yaw": self.angle_z,
            "dt": dt,
        }

    def close(self):
        if self.bus is not None:
            try:
                self.bus.close()
            finally:
                self.bus = None
        self.initialized = False


if __name__ == "__main__":
    # Standalone diagnostic mode only; importing this module starts nothing.
    sensor = MPU6050()
    try:
        sensor.initialize()
        while True:
            orientation = sensor.update()
            print(
                f"\rRoll: {orientation['roll']:7.2f}° | "
                f"Pitch: {orientation['pitch']:7.2f}° | "
                f"Yaw: {orientation['yaw']:7.2f}°",
                end="",
                flush=True,
            )
            time.sleep(0.02)
    except KeyboardInterrupt:
        print()
    finally:
        sensor.close()

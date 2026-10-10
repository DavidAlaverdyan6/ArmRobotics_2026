#!/usr/bin/env python3

"""
MPU6050 orientation monitor for Raspberry Pi.

Features:
- Gyroscope bias calibration at startup.
- Accelerometer-based pitch and roll correction.
- Complementary filter to reduce pitch/roll drift.
- Gyroscope integration for yaw (Z).
- Monotonic timing for stable integration.
- Configurable sampling and calibration.
- Clean shutdown on Ctrl+C.

Important:
The MPU6050 has no magnetometer. Yaw is relative and will
eventually drift; pitch and roll are stabilized by gravity.
"""

import math
import sys
import time

import smbus


# ============================================================
# Configuration
# ============================================================

I2C_BUS_NUMBER = 1
MPU6050_ADDRESS = 0x68

# MPU6050 registers
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_PWR_MGMT_1 = 0x6B
REG_WHO_AM_I = 0x75

# Default sensor scales:
# Gyroscope: +/-250 degrees/second -> 131 LSB/(degree/second)
# Accelerometer: +/-2g -> 16384 LSB/g
GYRO_SCALE = 131.0
ACCEL_SCALE = 16384.0

# Configure a 200 Hz sensor output rate (1 kHz / (1 + 4))
SAMPLE_RATE_DIVIDER = 4

# Complementary filter:
# Higher alpha trusts the gyroscope more.
# Lower alpha trusts the accelerometer more.
COMPLEMENTARY_ALPHA = 0.98

# Calibration settings
CALIBRATION_SAMPLES = 600
CALIBRATION_INTERVAL = 0.005

# Main loop
UPDATE_INTERVAL = 0.02  # Target 50 Hz

# Reject unrealistically long time intervals after pauses.
MAX_DT = 0.1

# Accelerometer angle correction is unreliable under strong
# linear acceleration. Skip correction when measured magnitude
# is substantially different from gravity.
ACCEL_GRAVITY_MIN = 0.75
ACCEL_GRAVITY_MAX = 1.25


# ============================================================
# I2C and sensor access
# ============================================================

bus = None


def read_register(register):
    """Read one byte from an MPU6050 register."""
    return bus.read_byte_data(MPU6050_ADDRESS, register)


def write_register(register, value):
    """Write one byte to an MPU6050 register."""
    bus.write_byte_data(MPU6050_ADDRESS, register, value)


def read_raw_axes():
    """
    Read accelerometer and gyroscope data.

    Returns:
        (ax, ay, az, gx, gy, gz) as signed 16-bit raw values.
    """
    data = bus.read_i2c_block_data(
        MPU6050_ADDRESS,
        REG_ACCEL_XOUT_H,
        14,
    )

    def signed_16(high, low):
        value = (high << 8) | low
        return value - 65536 if value >= 32768 else value

    ax = signed_16(data[0], data[1])
    ay = signed_16(data[2], data[3])
    az = signed_16(data[4], data[5])

    # Bytes 6-7 contain temperature.
    gx = signed_16(data[8], data[9])
    gy = signed_16(data[10], data[11])
    gz = signed_16(data[12], data[13])

    return ax, ay, az, gx, gy, gz


def initialize_sensor():
    """Wake the sensor and configure its filters and ranges."""
    write_register(REG_PWR_MGMT_1, 0x01)  # Wake; use X gyro PLL
    time.sleep(0.1)

    identity = read_register(REG_WHO_AM_I)
    if identity not in (0x68, 0x69):
        raise RuntimeError(
            f"Unexpected MPU6050 identity: 0x{identity:02X}"
        )

    # Digital low-pass filter: approximately 42 Hz accel,
    # 44 Hz gyro. Helps reduce noise and vibration.
    write_register(REG_CONFIG, 0x03)

    # Sample rate = 1 kHz / (1 + divider) when DLPF is enabled.
    write_register(REG_SMPLRT_DIV, SAMPLE_RATE_DIVIDER)

    # Gyroscope range: +/-250 degrees/second.
    write_register(REG_GYRO_CONFIG, 0x00)

    # Accelerometer range: +/-2g.
    write_register(REG_ACCEL_CONFIG, 0x00)

    time.sleep(0.1)


# ============================================================
# Calibration
# ============================================================

def calibrate_gyroscope():
    """
    Estimate gyro bias while the sensor is stationary.

    Keep the sensor still throughout calibration. The returned
    offsets are in raw sensor units.
    """
    print()
    print("Gyroscope calibration")
    print("---------------------")
    print("Keep the MPU6050 completely still.")

    sums = [0.0, 0.0, 0.0]

    # Allow the sensor to settle before collecting samples.
    time.sleep(0.5)

    for sample in range(CALIBRATION_SAMPLES):
        _, _, _, gx, gy, gz = read_raw_axes()

        sums[0] += gx
        sums[1] += gy
        sums[2] += gz

        time.sleep(CALIBRATION_INTERVAL)

        if (sample + 1) % 100 == 0:
            print(
                f"\rCalibrating: "
                f"{sample + 1}/{CALIBRATION_SAMPLES}",
                end="",
                flush=True,
            )

    offsets = [
        total / CALIBRATION_SAMPLES
        for total in sums
    ]

    print("\rCalibration complete.                    ")
    print(
        "Gyro offsets (raw): "
        f"X={offsets[0]:.2f}, "
        f"Y={offsets[1]:.2f}, "
        f"Z={offsets[2]:.2f}"
    )
    print(
        "Gyro offsets (deg/s): "
        f"X={offsets[0] / GYRO_SCALE:+.4f}, "
        f"Y={offsets[1] / GYRO_SCALE:+.4f}, "
        f"Z={offsets[2] / GYRO_SCALE:+.4f}"
    )
    print()

    return offsets


# ============================================================
# Orientation calculations
# ============================================================

def accelerometer_angles(ax, ay, az):
    """
    Calculate roll and pitch from gravity.

    Returns:
        (roll_degrees, pitch_degrees)
    """
    roll = math.degrees(math.atan2(ay, az))

    pitch = math.degrees(
        math.atan2(
            -ax,
            math.sqrt(ay * ay + az * az),
        )
    )

    return roll, pitch


def accelerometer_is_usable(ax, ay, az):
    """
    Check whether acceleration magnitude is close to 1g.

    Gravity-based orientation correction is less reliable while
    the sensor is undergoing strong linear acceleration.
    """
    magnitude_g = math.sqrt(ax * ax + ay * ay + az * az) / ACCEL_SCALE

    return ACCEL_GRAVITY_MIN <= magnitude_g <= ACCEL_GRAVITY_MAX


def normalize_angle(angle):
    """Wrap an angle to the interval [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def run():
    global bus

    bus = smbus.SMBus(I2C_BUS_NUMBER)

    try:
        initialize_sensor()

        gyro_offsets = calibrate_gyroscope()

        # Establish initial pitch and roll from gravity.
        ax_raw, ay_raw, az_raw, _, _, _ = read_raw_axes()

        angle_x, angle_y = accelerometer_angles(
            ax_raw,
            ay_raw,
            az_raw,
        )

        # Yaw has no absolute reference on this sensor.
        angle_z = 0.0

        # Use monotonic time so clock adjustments cannot corrupt dt.
        last_time = time.monotonic()

        print("Orientation monitor running. Press Ctrl+C to stop.")
        print("Angles are relative to the initial sensor orientation.")
        print()

        while True:
            loop_start = time.monotonic()
            dt = loop_start - last_time
            last_time = loop_start

            # Avoid integrating an unusually long interval after
            # a pause, debugger stop, or system scheduling delay.
            if dt <= 0.0 or dt > MAX_DT:
                _, _, _, _, _, _ = read_raw_axes()
                time.sleep(UPDATE_INTERVAL)
                continue

            ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = (
                read_raw_axes()
            )

            # Convert accelerometer values to g.
            ax = ax_raw / ACCEL_SCALE
            ay = ay_raw / ACCEL_SCALE
            az = az_raw / ACCEL_SCALE

            # Correct gyro bias and convert to degrees/second.
            gx = (gx_raw - gyro_offsets[0]) / GYRO_SCALE
            gy = (gy_raw - gyro_offsets[1]) / GYRO_SCALE
            gz = (gz_raw - gyro_offsets[2]) / GYRO_SCALE

            # Predict angles by integrating angular velocity.
            gyro_roll = angle_x + gx * dt
            gyro_pitch = angle_y + gy * dt
            angle_z += gz * dt

            # Correct roll and pitch using gravity when the
            # accelerometer magnitude looks reasonable.
            if accelerometer_is_usable(ax_raw, ay_raw, az_raw):
                accel_roll, accel_pitch = accelerometer_angles(
                    ax_raw,
                    ay_raw,
                    az_raw,
                )

                angle_x = (
                    COMPLEMENTARY_ALPHA * gyro_roll
                    + (1.0 - COMPLEMENTARY_ALPHA) * accel_roll
                )

                angle_y = (
                    COMPLEMENTARY_ALPHA * gyro_pitch
                    + (1.0 - COMPLEMENTARY_ALPHA) * accel_pitch
                )
            else:
                # During strong acceleration, temporarily trust
                # the gyro rather than a misleading gravity vector.
                angle_x = gyro_roll
                angle_y = gyro_pitch

            angle_z = normalize_angle(angle_z)

            print(
                f"\rX: {angle_x:7.2f}° | "
                f"Y: {angle_y:7.2f}° | "
                f"Z: {angle_z:7.2f}°",
                end="",
                flush=True,
            )

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, UPDATE_INTERVAL - elapsed))

    except KeyboardInterrupt:
        print("\nStopped by user.")

    except OSError as exc:
        print(
            f"\nI²C communication error: {exc}",
            file=sys.stderr,
        )
        print(
            "Check the SDA/SCL wiring, power, and I²C address.",
            file=sys.stderr,
        )
        raise

    finally:
        if bus is not None:
            bus.close()
            bus = None

        print("MPU6050 connection closed.")


if __name__ == "__main__":
    run()

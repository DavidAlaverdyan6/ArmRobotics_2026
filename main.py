
#!/usr/bin/env python3

import time

from telemetrix import telemetrix
from smbus2 import SMBus

import line_tracker
import motors
import led
import robot_pin_config as pins

rgb = led.RGBLED(
    pins.LED_RED,
    pins.LED_GREEN,
    pins.LED_BLUE,
)
# ── Existing hardware configuration ─────────────────────────────

PORT = "/dev/serial0"

BASE_SPEED = 150
MAX_SPEED = 180
MIN_SPEED = -60

KP = 30.0
TARGET_POSITION = 0.5
LOOP_DELAY = 0.04

# ── Intersection handling ───────────────────────────────────────

CROSSING_CONFIRM_FRAMES = 3
WHITE_THRESHOLD = 0.15
WHITE_CONFIRM_FRAMES = 3

CLEAR_SPEED = 100
CLEAR_MIN_TIME = 0.25
CLEAR_TIMEOUT = 3.0

REACQUIRE_TIMEOUT = 2.5

# ── Gyro-controlled turns ───────────────────────────────────────

I2C_BUS = 1
MPU_ADDRESS = 0x68

TURN_SPEED = 130
TURN_ANGLE = 88.0
TURN_TIMEOUT = 3.0

# Standard MPU6050 Z-axis convention:
# positive rotation is counter-clockwise when viewed from above.
LEFT_TURN_SIGN = 1
RIGHT_TURN_SIGN = -1

GYRO_CALIBRATION_SAMPLES = 300
GYRO_CALIBRATION_DELAY = 0.005
GYRO_DEADBAND_DPS = 1.0


def clamp(value, low, high):
    return max(low, min(high, value))


class MPU6050:
    """Minimal MPU6050 gyro reader using Raspberry Pi I2C."""

    PWR_MGMT_1 = 0x6B
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    GYRO_ZOUT_H = 0x47

    def __init__(self, bus_number=I2C_BUS, address=MPU_ADDRESS):
        self.bus = SMBus(bus_number)
        self.address = address
        self.bias_z = 0.0
        self.angle = 0.0
        self.last_time = None

        # Wake the MPU6050.
        self.bus.write_byte_data(
            self.address, self.PWR_MGMT_1, 0x00
        )

        # Enable a little low-pass filtering.
        self.bus.write_byte_data(
            self.address, self.CONFIG, 0x03
        )

        # Gyroscope range: +/-250 degrees/second.
        self.bus.write_byte_data(
            self.address, self.GYRO_CONFIG, 0x00
        )

        time.sleep(0.1)

    def _read_signed_word(self, register):
        high, low = self.bus.read_i2c_block_data(
            self.address, register, 2
        )
        value = (high << 8) | low

        if value >= 32768:
            value -= 65536

        return value

    def read_z_dps(self):
        return self._read_signed_word(self.GYRO_ZOUT_H) / 131.0

    def calibrate(self):
        """Keep the robot completely stationary during calibration."""
        print("Calibrating gyro. Keep the robot stationary...")

        readings = []

        for _ in range(GYRO_CALIBRATION_SAMPLES):
            readings.append(self.read_z_dps())
            time.sleep(GYRO_CALIBRATION_DELAY)

        self.bias_z = sum(readings) / len(readings)
        self.last_time = time.monotonic()

        print(f"Gyro Z bias: {self.bias_z:+.3f} deg/s")

    def reset_angle(self):
        self.angle = 0.0
        self.last_time = time.monotonic()

    def update_angle(self):
        now = time.monotonic()

        if self.last_time is None:
            self.last_time = now
            return self.angle

        dt = now - self.last_time
        self.last_time = now

        # Ignore unusually long intervals rather than integrating
        # a large angle jump after a delayed loop.
        if dt <= 0 or dt > 0.2:
            return self.angle

        rate = self.read_z_dps() - self.bias_z

        if abs(rate) < GYRO_DEADBAND_DPS:
            rate = 0.0

        self.angle += rate * dt
        return self.angle

    def close(self):
        self.bus.close()


def is_all_white(result):
    values = result["normalized"]
    return all(value < WHITE_THRESHOLD for value in values)


def main():
    board = None
    gyro = None

    state = "FOLLOW"

    crossing_candidate = None
    crossing_frames = 0
    white_frames = 0

    pending_turn = None
    state_started = 0.0
    turn_started = 0.0

    try:
        board = telemetrix.Telemetrix(com_port=PORT)

        motors.initialize(board)
        line_tracker.initialize(board)

        calibration = line_tracker.load_calibration()

        if calibration is None:
            print("No calibration found. Calibrate the sensors first.")
            return

        gyro = MPU6050()
        motors.motor_stop()
        gyro.calibrate()

        print("Line follower ready. Ctrl+C to stop.")

        while True:
            now = time.monotonic()
            result = line_tracker.update(calibration)

            # ── FOLLOW: ordinary proportional line following ────
            if state == "FOLLOW":
                crossing = result["crossing"]

                if crossing is not None:
                    if crossing == crossing_candidate:
                        crossing_frames += 1
                    else:
                        crossing_candidate = crossing
                        crossing_frames = 1
                else:
                    crossing_candidate = None
                    crossing_frames = 0

                if crossing_frames >= CROSSING_CONFIRM_FRAMES:
                    # Latch the decision so sensor changes while
                    # crossing cannot change the chosen direction.
                    if crossing_candidate == "LEFT":
                        pending_turn = "LEFT"
                    elif crossing_candidate == "RIGHT":
                        pending_turn = "RIGHT"
                    else:
                        pending_turn = "STRAIGHT"

                    motors.motor_stop()
                    state = "CLEAR"
                    state_started = now
                    white_frames = 0
                    crossing_candidate = None
                    crossing_frames = 0

                    print(f"\nIntersection: {pending_turn}")
                    time.sleep(LOOP_DELAY)
                    continue

                position = result["position"]

                if position is None:
                    # Preserve the existing lost-line behavior.
                    motors.motor_stop()
                else:
                    error = position - TARGET_POSITION
                    correction = KP * error

                    left_speed = clamp(
                        BASE_SPEED + correction,
                        MIN_SPEED,
                        MAX_SPEED,
                    )
                    right_speed = clamp(
                        BASE_SPEED - correction,
                        MIN_SPEED,
                        MAX_SPEED,
                    )

                    motors.set_speeds(
                        round(left_speed),
                        round(right_speed),
                    )

            # ── CLEAR: advance through the intersection ──────────
            elif state == "CLEAR":
                motors.set_speeds(CLEAR_SPEED, CLEAR_SPEED)

                if is_all_white(result):
                    white_frames += 1
                else:
                    white_frames = 0

                elapsed = now - state_started

                if (
                    elapsed >= CLEAR_MIN_TIME
                    and white_frames >= WHITE_CONFIRM_FRAMES
                ):
                    motors.motor_stop()

                    if pending_turn == "STRAIGHT":
                        # Drive forward through the crossing until
                        # the line is detected again.
                        state = "REACQUIRE"
                        state_started = now
                    else:
                        gyro.reset_angle()
                        turn_started = now
                        state = "TURN"

                elif elapsed > CLEAR_TIMEOUT:
                    motors.motor_stop()
                    raise RuntimeError(
                        "Timed out clearing intersection. "
                        "Check sensor calibration and crossing thresholds."
                    )

            # ── TURN: closed-loop gyro turn ──────────────────────
            elif state == "TURN":
                angle = gyro.update_angle()

                sign = (
                    LEFT_TURN_SIGN
                    if pending_turn == "LEFT"
                    else RIGHT_TURN_SIGN
                )

                progress = sign * angle

                if progress >= TURN_ANGLE:
                    motors.motor_stop()
                    state = "REACQUIRE"
                    state_started = now

                elif now - turn_started > TURN_TIMEOUT:
                    motors.motor_stop()
                    raise RuntimeError(
                        f"{pending_turn} turn timed out at "
                        f"{progress:.1f} degrees. "
                        "Check gyro direction, motor polarity and speed."
                    )
                else:
                    # Pivot in place.
                    motors.set_speeds(
                        -sign * TURN_SPEED,
                        sign * TURN_SPEED,
                    )

            # ── REACQUIRE: find the line after turning/straight ──
            elif state == "REACQUIRE":
                if result["position"] is not None:
                    motors.motor_stop()
                    state = "FOLLOW"
                    pending_turn = None
                    crossing_candidate = None
                    crossing_frames = 0
                    white_frames = 0
                    print("\nLine reacquired.")
                elif now - state_started > REACQUIRE_TIMEOUT:
                    motors.motor_stop()
                    raise RuntimeError(
                        "Could not reacquire the line after maneuver."
                    )
                else:
                    # Straight crossing: continue forward.
                    # After a turn, make a slow forward search.
                    search_speed = (
                        CLEAR_SPEED
                        if pending_turn == "STRAIGHT"
                        else 55
                    )
                    motors.set_speeds(search_speed, search_speed)

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nStopping.")

    finally:
        if board is not None:
            try:
                motors.motor_stop()
                line_tracker.shutdown()
            finally:
                board.shutdown()

        if gyro is not None:
            gyro.close()


if __name__ == "__main__":
    main()


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

# Hardware configuration

PORT = "/dev/serial0"

BASE_SPEED = 150
MAX_SPEED = 180
MIN_SPEED = -60

KP = 30.0
TARGET_POSITION = 0.5
LOOP_DELAY = 0.04

# Intersection handling

STARTUP_INTERSECTION_IGNORE = 0.5
CROSSING_CONFIRM_FRAMES = 3

JUNCTION_EDGE_THRESHOLD = 0.20
EXIT_CONFIRM_FRAMES = 3

CLEAR_SPEED = 100
CLEAR_TIMEOUT = 3.0

# Movement before turning

APPROACH_TIME = 0.5

# Gyro-controlled turns

I2C_BUS = 1
MPU_ADDRESS = 0x68

TURN_SPEED = 130
TURN_ANGLE = 88.0
TURN_TIMEOUT = 3.0

LEFT_TURN_SIGN = 1
RIGHT_TURN_SIGN = -1

GYRO_CALIBRATION_SAMPLES = 300
GYRO_CALIBRATION_DELAY = 0.005
GYRO_DEADBAND_DPS = 1.0

# Post-turn line correction

POST_TURN_TIME = 1.0

# Sensor indices: S1..S8, ordered left to right

LEFT_SENSORS = (0, 1, 2)
CENTER_SENSORS = (3, 4)
RIGHT_SENSORS = (5, 6, 7)


def clamp(value, low, high):
    return max(low, min(high, value))


class MPU6050:
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

        self.bus.write_byte_data(
            self.address, self.PWR_MGMT_1, 0x00
        )
        self.bus.write_byte_data(
            self.address, self.CONFIG, 0x03
        )
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

        if dt <= 0 or dt > 0.2:
            return self.angle

        rate = self.read_z_dps() - self.bias_z

        if abs(rate) < GYRO_DEADBAND_DPS:
            rate = 0.0

        self.angle += rate * dt
        return self.angle

    def close(self):
        self.bus.close()


def junction_exit_detected(result):
    values = result["normalized"]

    left_clear = all(
        values[i] < JUNCTION_EDGE_THRESHOLD
        for i in LEFT_SENSORS
    )

    right_clear = all(
        values[i] < JUNCTION_EDGE_THRESHOLD
        for i in RIGHT_SENSORS
    )

    center_detected = any(
        values[i] >= JUNCTION_EDGE_THRESHOLD
        for i in CENTER_SENSORS
    )

    return center_detected and (left_clear or right_clear)


def follow_line(result):
    position = result["position"]

    if position is None:
        motors.motor_stop()
        return

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


def main():
    board = None
    gyro = None

    state = "FOLLOW"

    crossing_candidate = None
    crossing_frames = 0
    exit_frames = 0

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

        robot_start_time = time.monotonic()

        while True:
            now = time.monotonic()
            result = line_tracker.update(calibration)

            # FOLLOW: normal line tracking and crossing detection

            if state == "FOLLOW":
                elapsed_since_start = now - robot_start_time

                if elapsed_since_start < STARTUP_INTERSECTION_IGNORE:
                    crossing = None
                else:
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
                    if crossing_candidate == "LEFT":
                        pending_turn = "LEFT"
                    elif crossing_candidate == "RIGHT":
                        pending_turn = "RIGHT"
                    else:
                        pending_turn = "STRAIGHT"

                    motors.motor_stop()

                    state = "CLEAR"
                    state_started = now
                    exit_frames = 0

                    crossing_candidate = None
                    crossing_frames = 0

                    print(f"\nIntersection detected: {pending_turn}")

                    time.sleep(LOOP_DELAY)
                    continue

                follow_line(result)

            # CLEAR: move through the junction until its exit is detected

            elif state == "CLEAR":
                crossing = result["crossing"]

                if crossing is None:
                    exit_frames += 1
                else:
                    exit_frames = 0

                if exit_frames >= EXIT_CONFIRM_FRAMES:
                    motors.motor_stop()

                    if pending_turn == "STRAIGHT":
                        state = "FOLLOW"
                        pending_turn = None

                        crossing_candidate = None
                        crossing_frames = 0
                        exit_frames = 0

                        print("Crossing cleared. Resuming line following.")

                    else:
                        state = "APPROACH"
                        state_started = now

                        print(
                            f"Crossing cleared. Moving forward for "
                            f"{APPROACH_TIME:.1f} seconds before turning."
                        )

                elif now - state_started > CLEAR_TIMEOUT:
                    motors.motor_stop()

                    raise RuntimeError(
                        "Crossing pattern did not clear. "
                        "Check line_tracker crossing classification."
                    )

                else:
                    motors.set_speeds(CLEAR_SPEED, CLEAR_SPEED)

            # APPROACH: move forward before starting the pivot

            elif state == "APPROACH":
                if now - state_started >= APPROACH_TIME:
                    motors.motor_stop()

                    gyro.reset_angle()
                    turn_started = time.monotonic()
                    state = "TURN"

                    print(f"Starting {pending_turn} gyro turn.")

                else:
                    motors.set_speeds(CLEAR_SPEED, CLEAR_SPEED)

            # TURN: gyro-controlled pivot

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

                    state = "POST_TURN"
                    state_started = time.monotonic()

                    crossing_candidate = None
                    crossing_frames = 0
                    exit_frames = 0

                    print(
                        "Turn complete. Starting "
                        f"{POST_TURN_TIME:.1f}-second line correction."
                    )

                elif now - turn_started > TURN_TIMEOUT:
                    motors.motor_stop()

                    raise RuntimeError(
                        f"{pending_turn} turn timed out at "
                        f"{progress:.1f} degrees. "
                        "Check gyro direction and motor polarity."
                    )

                else:
                    motors.set_speeds(
                        -sign * TURN_SPEED,
                        sign * TURN_SPEED,
                    )

            # POST_TURN: follow the line, but ignore crossings temporarily

            elif state == "POST_TURN":
                follow_line(result)

                if now - state_started >= POST_TURN_TIME:
                    state = "FOLLOW"
                    pending_turn = None

                    crossing_candidate = None
                    crossing_frames = 0
                    exit_frames = 0

                    print(
                        "Post-turn correction complete. "
                        "Resuming normal line following."
                    )

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

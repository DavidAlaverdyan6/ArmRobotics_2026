
#!/usr/bin/env python3

import time

from telemetrix import telemetrix
from smbus2 import SMBus

import line_tracker
import motors


# ── Hardware configuration ──────────────────────────────────────

PORT = "/dev/serial0"

# ── Line following ──────────────────────────────────────────────

BASE_SPEED = 150
MAX_SPEED = 170
MIN_SPEED = -50

KP = 40.0
TARGET_POSITION = 0.5
LOOP_DELAY = 0.04

# ── Live debugging display ──────────────────────────────────────

DISPLAY_INTERVAL = 0.1

# ── Intersection handling ───────────────────────────────────────

CROSSING_CONFIRM_FRAMES = 3

WHITE_THRESHOLD = 0.15
WHITE_CONFIRM_FRAMES = 3

CLEAR_SPEED = 120
CLEAR_MIN_TIME = 0.45
CLEAR_TIMEOUT = 4.5

REACQUIRE_TIMEOUT = 3.5

# ── Gyro-controlled turns ──────────────────────────────────────

I2C_BUS = 1
MPU_ADDRESS = 0x68

TURN_SPEED = 155
TURN_ANGLE = 95
TURN_TIMEOUT = 4.5

LEFT_TURN_SIGN = 1
RIGHT_TURN_SIGN = -1

GYRO_CALIBRATION_SAMPLES = 300
GYRO_CALIBRATION_DELAY = 0.005
GYRO_DEADBAND_DPS = 1.0


def clamp(value, low, high):
    return max(low, min(high, value))


def show_position(state, mode, pending_turn=None):
    """Display live sensor values without opening another connection."""

    position = state["position"]

    # Clear the terminal and redraw the diagnostic screen.
    print("\033[2J\033[H", end="")

    print("=" * 52)
    print("          ARMROBOTICS - LINE DEBUGGER")
    print("=" * 52)

    print(f"\nMODE: {mode}")
    print(f"PENDING TURN: {pending_turn or 'None'}")

    print("\nRAW SENSOR VALUES:")
    print(" ".join(
        f"S{i + 1}:{value:4}"
        for i, value in enumerate(state["raw"])
    ))

    print("\nNORMALIZED SENSOR VALUES:")

    for i, value in enumerate(state["normalized"]):
        bar = "#" * round(value * 20)
        print(f"S{i + 1}: [{bar:<20}] {value:.2f}")

    print("\nLINE POSITION:")

    if position is None:
        print("LINE NOT DETECTED")
    else:
        print(f"Position value: {position:+.3f}")

        # Display scale matches show_line_position.py: -3 to +4.
        width = 41
        marker = round((position + 3) / 7 * (width - 1))
        marker = max(0, min(width - 1, marker))

        bar = ["-"] * width
        bar[marker] = "|"

        print("-3 " + "".join(bar) + " +4")
        print("    " + " " * marker + "^")

        error = position - TARGET_POSITION
        print(f"Target position: {TARGET_POSITION:+.3f}")
        print(f"Position error:  {error:+.3f}")

    print(f"\nLINE DETECTED: {state['line_detected']}")
    print(f"CROSSING: {state['crossing'] or 'None'}")
    print("\nPress Ctrl+C to stop.", flush=True)


class MPU6050:
    """Minimal MPU6050 Z-axis gyro reader over Raspberry Pi I2C."""

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

        try:
            # Wake the MPU6050.
            self.bus.write_byte_data(
                self.address, self.PWR_MGMT_1, 0x00
            )

            # Configure the digital low-pass filter.
            self.bus.write_byte_data(
                self.address, self.CONFIG, 0x03
            )

            # Gyro full-scale range: +/-250 degrees/second.
            self.bus.write_byte_data(
                self.address, self.GYRO_CONFIG, 0x00
            )

            time.sleep(0.1)

        except Exception:
            self.bus.close()
            raise

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
        """Calibrate with the robot completely stationary."""

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

        # Avoid integrating large jumps after a delayed iteration.
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
    return all(
        value < WHITE_THRESHOLD
        for value in result["normalized"]
    )


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
    last_display_time = 0.0

    try:
        # This is the ONLY Telemetrix session.
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
        time.sleep(0.5)

        while True:
            now = time.monotonic()

            # One sensor update per loop. Both debugging and control
            # use this exact same result.
            result = line_tracker.update(calibration)

            # ── Debug display ───────────────────────────────────
            if now - last_display_time >= DISPLAY_INTERVAL:
                show_position(result, state, pending_turn)
                last_display_time = now

            # ── FOLLOW: proportional line following ─────────────
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

                    continue

                position = result["position"]

                if position is None:
                    motors.motor_stop()
                else:
                    error = position - TARGET_POSITION

                    # Slow down on sharp bends while retaining
                    # proportional differential steering.
                    base_speed = max(
                        55,
                        BASE_SPEED - 12 * abs(error),
                    )

                    correction = KP * error

                    left_speed = clamp(
                        base_speed + correction,
                        MIN_SPEED,
                        MAX_SPEED,
                    )

                    right_speed = clamp(
                        base_speed - correction,
                        MIN_SPEED,
                        MAX_SPEED,
                    )

                    motors.set_speeds(
                        round(left_speed),
                        round(right_speed),
                    )

            # ── CLEAR: move through the intersection ────────────
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
                        "Check sensor calibration and thresholds."
                    )

            # ── TURN: gyro-controlled pivot ─────────────────────
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
                        f"{progress:.1f} degrees. Check gyro direction, "
                        "motor polarity, battery and turning speed."
                    )

                else:
                    motors.set_speeds(
                        -sign * TURN_SPEED,
                        sign * TURN_SPEED,
                    )

            # ── REACQUIRE: find the line again ──────────────────
            elif state == "REACQUIRE":
                if result["position"] is not None:
                    motors.motor_stop()

                    state = "FOLLOW"
                    pending_turn = None

                    crossing_candidate = None
                    crossing_frames = 0
                    white_frames = 0

                elif now - state_started > REACQUIRE_TIMEOUT:
                    motors.motor_stop()

                    raise RuntimeError(
                        "Could not reacquire the line after maneuver."
                    )

                else:
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
            except Exception:
                pass

            try:
                line_tracker.shutdown()
            finally:
                board.shutdown()

        if gyro is not None:
            gyro.close()

        print("Disconnected.")


if __name__ == "__main__":
    main()


#!/usr/bin/env python3

import time
import curses
import math
import smbus
from telemetrix import telemetrix

import motors
from stm32_reset import STM32_RESET_GPIO, reset_stm32

PORT = "/dev/serial0"

# Adjust these to suit your robot.
DRIVE_SPEED = 190
MAX_DRIVE_SPEED = 255
FULL_SPEED_HOLD_SECONDS = 1.0
TURN_SPEED = 130
CURVE_INNER_SPEED = 90
LOOP_DELAY = 0.01
DISPLAY_INTERVAL = 0.05
PWM_SLEW_PER_TICK = 90
KEY_TIMEOUT_SECONDS = 0.12
CHORD_WINDOW_SECONDS = 0.75

I2C_BUS = 1
MPU_ADDR = 0x68
GYRO_Z_REG = 0x47
GYRO_SCALE = 131.0
GYRO_CALIBRATION_SAMPLES = 200
GYRO_HEADING_KP = 4.0
GYRO_RATE_KD = 0.8
GYRO_MAX_CORRECTION = 90
# MPU Z is normally positive for a physical left turn when mounted face-up.
# Change to -1 only if a straight-drive test shows correction getting worse.
GYRO_LEFT_POSITIVE = 1

board = None


def stop():
    motors.motor_stop()



def gyro_raw(bus):
    hi = bus.read_byte_data(MPU_ADDR, GYRO_Z_REG)
    lo = bus.read_byte_data(MPU_ADDR, GYRO_Z_REG + 1)
    value = (hi << 8) | lo
    return value - 65536 if value >= 32768 else value


def initialize_gyro(bus):
    bus.write_byte_data(MPU_ADDR, 0x6B, 0)
    bus.write_byte_data(MPU_ADDR, 0x1B, 0)
    time.sleep(0.2)
    print("Keep the robot still: calibrating MPU6050...")
    samples = []
    for _ in range(GYRO_CALIBRATION_SAMPLES):
        samples.append(gyro_raw(bus))
        time.sleep(0.002)
    offset = sum(samples) / len(samples)
    print(f"Gyro ready; Z offset={offset:.1f}.")
    return offset


def approach(current, target, step=PWM_SLEW_PER_TICK):
    if current < target:
        return min(target, current + step)
    if current > target:
        return max(target, current - step)
    return target


class ManualDrive:
    def __init__(self, gyro_bus, gyro_offset):
        self.bus = gyro_bus
        self.offset = gyro_offset
        self.mode = None
        self.left_pwm = 0
        self.right_pwm = 0
        self.heading = 0.0
        self.rate = 0.0
        self.correction = 0.0
        self.previous_time = time.monotonic()
        self.mode_started = self.previous_time

    def set_mode(self, mode):
        if mode != self.mode:
            # Straight mode holds the heading present when W takes control;
            # it does not try to undo an intentional curved turn.
            if mode == "W":
                self.heading = 0.0
                self.rate = 0.0
                self.correction = 0.0
            self.previous_time = time.monotonic()
            self.mode_started = self.previous_time
        self.mode = mode

    def _forward_speed(self, now):
        if now - self.mode_started >= FULL_SPEED_HOLD_SECONDS:
            return MAX_DRIVE_SPEED
        return DRIVE_SPEED

    def _read_heading(self, now):
        dt = max(0.0, min(now - self.previous_time, 0.15))
        self.previous_time = now
        self.rate = (
            GYRO_LEFT_POSITIVE
            * (gyro_raw(self.bus) - self.offset)
            / GYRO_SCALE
        )
        if not math.isfinite(self.rate) or abs(self.rate) > 1000:
            raise RuntimeError("Invalid MPU6050 angular rate")
        self.heading += self.rate * dt
        self.correction = max(
            -GYRO_MAX_CORRECTION,
            min(
                GYRO_MAX_CORRECTION,
                GYRO_HEADING_KP * self.heading + GYRO_RATE_KD * self.rate,
            ),
        )

    def _target_speeds(self, now):
        if self.mode == "W":
            self._read_heading(now)
            forward_speed = self._forward_speed(now)
            # A positive (left) yaw is corrected by slowing the right wheel.
            if self.correction >= 0:
                return forward_speed, forward_speed - self.correction
            return forward_speed + self.correction, forward_speed
        if self.mode == "WA":
            return CURVE_INNER_SPEED, self._forward_speed(now)
        if self.mode == "WD":
            return self._forward_speed(now), CURVE_INNER_SPEED
        if self.mode == "S":
            return -DRIVE_SPEED, -DRIVE_SPEED
        if self.mode == "A":
            return -TURN_SPEED, TURN_SPEED
        if self.mode == "D":
            return TURN_SPEED, -TURN_SPEED
        return 0, 0

    def update(self, now):
        target_left, target_right = self._target_speeds(now)
        self.left_pwm = approach(self.left_pwm, target_left)
        self.right_pwm = approach(self.right_pwm, target_right)
        motors.set_speeds(round(self.left_pwm), round(self.right_pwm))

    def emergency_stop(self):
        self.mode = None
        self.left_pwm = 0
        self.right_pwm = 0
        self.heading = 0.0
        self.rate = 0.0
        self.correction = 0.0
        motors.motor_stop()


def resolve_mode(current_mode, last_key, last_key_time, letter, now):
    """Translate terminal key events into straight, pivot, or curve mode."""
    paired = now - last_key_time <= CHORD_WINDOW_SECONDS
    if paired and {last_key, letter} == {"W", "A"}:
        return "WA"
    if paired and {last_key, letter} == {"W", "D"}:
        return "WD"
    if current_mode == "WA" and letter == "A":
        return "WA"
    if current_mode == "WD" and letter == "D":
        return "WD"
    return letter




def controller(stdscr, manual_drive):
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)

    current_mode = None
    last_key = None
    last_key_time = 0.0
    last_command_time = 0.0
    last_display_time = 0.0

    while True:
        cycle_started = time.monotonic()
        if cycle_started - last_display_time >= DISPLAY_INTERVAL:
            height, width = stdscr.getmaxyx()
            stdscr.erase()

            lines = [
                "ROBOT MANUAL DRIVE",
                "",
                "W - Forward",
                "S - Backward",
                "A - Left",
                "D - Right",
                "W+A - Smooth curve left",
                "W+D - Smooth curve right",
                "SPACE - Stop",
                "Q - Quit",
            ]

            for y, line in enumerate(lines):
                if y < height and width > 1:
                    stdscr.addnstr(y, 0, line, width - 1)

            if height > len(lines) + 1:
                stdscr.addnstr(
                    len(lines) + 1,
                    0,
                    f"Active: {current_mode or 'STOPPED'}",
                    width - 1,
                )

            if height > len(lines) + 2:
                stdscr.addnstr(
                    len(lines) + 2,
                    0,
                    f"Gyro heading={manual_drive.heading:+.1f} deg "
                    f"rate={manual_drive.rate:+.1f} deg/s "
                    f"correction={manual_drive.correction:+.0f}",
                    width - 1,
                )

            stdscr.refresh()
            last_display_time = cycle_started

        keys = []
        key = stdscr.getch()
        while key != -1:
            keys.append(key)
            key = stdscr.getch()

        quit_requested = False
        for key in keys:
            now = time.monotonic()
            if key == ord("q"):
                quit_requested = True
                break
            if key == ord(" "):
                current_mode = None
                last_key = None
                manual_drive.emergency_stop()
                continue
            if key not in (ord("w"), ord("a"), ord("s"), ord("d")):
                continue

            letter = chr(key).upper()
            current_mode = resolve_mode(
                current_mode, last_key, last_key_time, letter, now
            )

            last_key = letter
            last_key_time = now
            last_command_time = now
            manual_drive.set_mode(current_mode)

        if quit_requested:
            break

        now = time.monotonic()
        if current_mode is not None and (
            now - last_command_time > KEY_TIMEOUT_SECONDS
        ):
            current_mode = None
            manual_drive.emergency_stop()
        elif current_mode is not None:
            manual_drive.update(now)

        time.sleep(max(0.0, LOOP_DELAY - (time.monotonic() - cycle_started)))


def main():
    global board
    gyro_bus = None
    manual_drive = None

    try:
        print("Connecting to STM32...")
        print(f"Resetting STM32 through Raspberry Pi BCM{STM32_RESET_GPIO}...")
        reset_stm32()
        board = telemetrix.Telemetrix(com_port=PORT)

        time.sleep(2)

        motors.initialize(board)
        stop()
        gyro_bus = smbus.SMBus(I2C_BUS)
        gyro_offset = initialize_gyro(gyro_bus)
        manual_drive = ManualDrive(gyro_bus, gyro_offset)

        print("Manual drive ready.")
        curses.wrapper(controller, manual_drive)

    except KeyboardInterrupt:
        print("\nStopping robot.")

    finally:
        if manual_drive is not None:
            try:
                manual_drive.emergency_stop()
            except Exception:
                pass
        if board is not None:
            try:
                stop()
            except Exception:
                pass

            board.shutdown()
        if gyro_bus is not None:
            try:
                gyro_bus.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()

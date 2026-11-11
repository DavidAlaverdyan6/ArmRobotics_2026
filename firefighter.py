"""Flame input and pump using the robot's existing Telemetrix connection."""

import time

import robot_pin_config as pins
from arm_ik import inverse_kinematics

# STM32 firmware uses 12-bit ADC readings. Hardware observation:
# 0 means no flame; readings of 2000-2500 indicate flame (active-high).
FIRE_THRESHOLD = 2000
FIRE_ACTIVE_LOW = False
SENSOR_MAX_AGE = 0.5
WATER_SECONDS = 5.0
FLAME_CONFIRM_SECONDS = 0.5
FIRE_SCAN_DEGREES = 20
FIRE_SCAN_STEP_DEGREES = 2
FIRE_SCAN_STEP_DELAY = 0.08
SENSOR_WAIT_REPORT_SECONDS = 2.0
SENSOR_WAIT_TIMEOUT_SECONDS = 4.0
POLL_SECONDS = 0.02

FIRE_X_MM = 300.0
FIRE_Y_MM = 0.0
FIRE_Z_MM = 50.0
FIRE_TOOL_PITCH_DEG = -30.0


def firefighting_pose():
    """Solve the configured XYZ/tool-pitch pose for the F task."""
    solution = inverse_kinematics(
        FIRE_X_MM,
        FIRE_Y_MM,
        FIRE_Z_MM,
        tool_pitch=FIRE_TOOL_PITCH_DEG,
    )
    return {
        "base": solution.base,
        "shoulder": solution.shoulder,
        "elbow": solution.elbow,
        "wrist": solution.wrist,
        "gripper": 0,
    }


def scan_for_confirmed_flame(fire, arm, center_base):
    """Sweep the arm left then right while the chassis remains stopped."""
    step = FIRE_SCAN_STEP_DEGREES
    offsets = (
        list(range(-step, -FIRE_SCAN_DEGREES - 1, -step))
        + list(range(-FIRE_SCAN_DEGREES + step, FIRE_SCAN_DEGREES + 1, step))
    )
    print(
        f"\nF: signal was not stable for {FLAME_CONFIRM_SECONDS:g} s; "
        f"scanning arm from -{FIRE_SCAN_DEGREES} to "
        f"+{FIRE_SCAN_DEGREES} degrees."
    )
    for offset in offsets:
        arm.write(base=center_base + offset)
        time.sleep(FIRE_SCAN_STEP_DELAY)
        detected = fire.detected()
        if detected is None:
            fire.wait_for_sample()
            detected = fire.detected()
        if detected is True and fire.confirm_flame():
            print(
                f"\nF: flame confirmed at horizontal offset "
                f"{offset:+d} degrees."
            )
            return True

    arm.write(base=center_base)
    print("\nF: scan found no stable flame; resuming slow approach.")
    return False


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
        """Wait for a new report with a bounded deadline while stopped."""
        last_report = time.monotonic()
        deadline = last_report + SENSOR_WAIT_TIMEOUT_SECONDS
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
            if now >= deadline:
                raise TimeoutError("F: flame sensor report timeout")
            time.sleep(POLL_SECONDS)

    def confirm_flame(self):
        """Require an uninterrupted flame signal while the base is stopped."""
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

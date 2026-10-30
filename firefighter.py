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
SENSOR_WAIT_TIMEOUT = 2.0
POLL_SECONDS = 0.02


# Gripper-tip target in the robot's arm_web coordinate system (mm).
FIRE_TARGET_X_MM = 300.0
FIRE_TARGET_Y_MM = 0.0
FIRE_TARGET_Z_MM = 130.0
FIRE_TOOL_PITCH_DEG = -15.0


def firefighting_pose():
    """Solve the F position with the tip pointing 15 degrees below horizontal.

    The tool direction is fixed at the target and while the robot approaches
    the flame. The movement from home to this pose is joint-interpolated;
    it does not keep a constant tool pitch during that transition.
    """
    from arm_ik import inverse_kinematics

    solution = inverse_kinematics(
        FIRE_TARGET_X_MM,
        FIRE_TARGET_Y_MM,
        FIRE_TARGET_Z_MM,
        tool_pitch=FIRE_TOOL_PITCH_DEG,
        elbow_up=False,
    )
    return {
        name: getattr(solution, name)
        for name in ("base", "shoulder", "elbow", "wrist")
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
            raise RuntimeError("F: flame sensor has no fresh reading")
        value = sample[0]
        return value <= FIRE_THRESHOLD if FIRE_ACTIVE_LOW else value >= FIRE_THRESHOLD

    def off(self):
        self.board.digital_write(pins.pump, 0)

    def wait_for_sample(self, after=None):
        deadline = time.monotonic() + SENSOR_WAIT_TIMEOUT
        while time.monotonic() < deadline:
            sample = self.sample
            if (sample is not None
                    and (after is None or sample[1] > after)
                    and time.monotonic() - sample[1] <= SENSOR_MAX_AGE):
                return
            time.sleep(POLL_SECONDS)
        raise RuntimeError("F: timed out waiting for a new flame reading")

    def confirm_flame(self):
        """Require one uninterrupted second of flame while the base is stopped."""
        start = time.monotonic()
        while True:
            if not self.detected():
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

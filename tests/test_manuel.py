import unittest
from unittest.mock import patch

import manuel


class FakeGyroBus:
    def __init__(self, raw=0):
        self.raw = raw & 0xFFFF

    def read_byte_data(self, _address, register):
        if register == manuel.GYRO_Z_REG:
            return self.raw >> 8
        if register == manuel.GYRO_Z_REG + 1:
            return self.raw & 0xFF
        raise AssertionError(f"unexpected register {register:#x}")


class ManualDriveTests(unittest.TestCase):
    def test_terminal_w_then_a_resolves_to_curve_after_tap_stops(self):
        mode = manuel.resolve_mode(None, "W", 1.0, "A", 1.5)
        self.assertEqual(mode, "WA")
        self.assertEqual(
            manuel.resolve_mode(mode, "A", 1.5, "A", 1.6),
            "WA",
        )

    def test_forward_left_and_right_are_smooth_curves(self):
        drive = manuel.ManualDrive(FakeGyroBus(), 0)

        drive.set_mode("WA")
        self.assertEqual(
            drive._target_speeds(1.0),
            (manuel.CURVE_INNER_SPEED, manuel.DRIVE_SPEED),
        )

        drive.set_mode("WD")
        self.assertEqual(
            drive._target_speeds(1.0),
            (manuel.DRIVE_SPEED, manuel.CURVE_INNER_SPEED),
        )

    def test_positive_left_yaw_generates_right_correction(self):
        # 1310 raw units above offset is +10 degrees/second.
        drive = manuel.ManualDrive(FakeGyroBus(raw=1310), 0)
        drive.set_mode("W")
        left, right = drive._target_speeds(drive.previous_time + 0.1)

        self.assertEqual(left, manuel.DRIVE_SPEED)
        self.assertLess(right, left)
        self.assertGreater(drive.heading, 0)

    def test_forward_reaches_maximum_after_one_second_hold(self):
        drive = manuel.ManualDrive(FakeGyroBus(), 0)
        drive.set_mode("W")

        before, _ = drive._target_speeds(
            drive.mode_started + manuel.FULL_SPEED_HOLD_SECONDS - 0.01
        )
        after, _ = drive._target_speeds(
            drive.mode_started + manuel.FULL_SPEED_HOLD_SECONDS + 0.01
        )

        self.assertEqual(before, manuel.DRIVE_SPEED)
        self.assertEqual(after, manuel.MAX_DRIVE_SPEED)

    def test_motor_commands_ramp_but_emergency_stop_is_immediate(self):
        drive = manuel.ManualDrive(FakeGyroBus(), 0)
        drive.set_mode("WA")

        with patch.object(manuel.motors, "set_speeds") as set_speeds, \
             patch.object(manuel.motors, "motor_stop") as motor_stop:
            drive.update(1.0)
            set_speeds.assert_called_once_with(
                manuel.PWM_SLEW_PER_TICK,
                manuel.PWM_SLEW_PER_TICK,
            )

            drive.emergency_stop()
            motor_stop.assert_called_once_with()
            self.assertEqual((drive.left_pwm, drive.right_pwm), (0, 0))


if __name__ == "__main__":
    unittest.main()

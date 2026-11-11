"""Hardware-free tests for the extracted line-following engine."""

import unittest
from unittest.mock import patch

from line_tracker import LineTrackingEngine


class FakeMotors:
    def __init__(self):
        self.commands = []
        self.stopped = False

    def set_speeds(self, left, right):
        self.commands.append((left, right))

    def motor_stop(self):
        self.stopped = True


class LineFollowerTests(unittest.TestCase):
    def setUp(self):
        self.motors = FakeMotors()
        self.follower = LineTrackingEngine(motor_driver=self.motors)

    def test_centered_line_drives_both_motors_equally(self):
        self.assertTrue(self.follower.drive({"position": 0.0}))
        self.assertEqual(self.motors.commands[-1], (110, 110))

    def test_positive_error_applies_differential_correction(self):
        self.follower.drive({"position": 1.0})
        self.assertEqual(self.motors.commands[-1], (140, 80))

    def test_missing_line_stops(self):
        self.assertFalse(self.follower.drive({"position": None}))
        self.assertTrue(self.motors.stopped)

    def test_damping_reduces_correction_when_approaching_center(self):
        with patch('line_tracker.time.monotonic', side_effect=[1.0, 1.04]):
            self.follower.drive({'position': 0.5})
            self.follower.drive({'position': 0.3})
        left, right = self.motors.commands[-1]
        self.assertLess(left - right, 2 * 30 * 0.3)
        self.assertGreater(left - right, 0)

    def test_stop_discards_derivative_before_reacquisition(self):
        with patch('line_tracker.time.monotonic', side_effect=[1.0, 1.04]):
            self.follower.drive({'position': -1.0})
            self.follower.stop()
            self.follower.drive({'position': 1.0})
        self.assertEqual(self.motors.commands[-1], (140, 80))

    def test_open_loop_fallback_continues_straight(self):
        self.follower.drive_straight(speed=90, max_speed=120)
        self.assertEqual(self.motors.commands[-1], (90, 90))

    def test_reverse_keeps_both_wheels_in_reverse(self):
        self.assertTrue(
            self.follower.reverse(
                {"position": 0.5},
                speed=100,
                heading_correction=5,
            )
        )
        left, right = self.motors.commands[-1]
        self.assertLess(left, 0)
        self.assertLess(right, 0)

    def test_reverse_never_commands_below_hardware_minimum(self):
        self.follower.reverse({"position": 1.0}, speed=130)
        left, right = self.motors.commands[-1]
        self.assertLessEqual(left, -80)
        self.assertLessEqual(right, -80)
        self.assertGreaterEqual(left, -130)
        self.assertGreaterEqual(right, -130)

    def test_reverse_fallback_uses_heading_correction(self):
        self.follower.reverse_straight(
            speed=100,
            heading_correction=10,
        )
        self.assertEqual(self.motors.commands[-1], (-100, -90))


if __name__ == "__main__":
    unittest.main()

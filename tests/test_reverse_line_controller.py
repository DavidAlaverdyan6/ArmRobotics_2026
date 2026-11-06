"""Checks for geometry-aware reverse line following without hardware."""

import unittest
from unittest.mock import Mock, patch

from test_cube_task import load_main_test


class StaticHeading:
    def __init__(self, angle=0.0, rate=0.0):
        self.angle = angle
        self.last_rate = rate

    def read(self):
        return self.angle


class ReverseLineControllerTests(unittest.TestCase):
    def setUp(self):
        self.nav = load_main_test()
        self.nav.motors.set_speeds = Mock()

    def test_line_correction_brakes_only_one_reverse_wheel(self):
        controller = self.nav.ReverseLineController(StaticHeading())
        with patch.object(self.nav.time, "monotonic", return_value=1.0):
            self.assertTrue(controller.drive({"position": 2.0}, 130))
        left, right = self.nav.motors.set_speeds.call_args.args
        self.assertLessEqual(left, -self.nav.REVERSE_MIN_PWM)
        self.assertLessEqual(right, -self.nav.REVERSE_MIN_PWM)
        self.assertGreaterEqual(left, -self.nav.REVERSE_MAX_PWM)
        self.assertGreaterEqual(right, -self.nav.REVERSE_MAX_PWM)
        self.assertNotEqual(left, right)
        self.assertGreater(abs(left), abs(right))

    def test_prolonged_line_loss_fails_safely(self):
        controller = self.nav.ReverseLineController(StaticHeading())
        with patch.object(
            self.nav.time, "monotonic", side_effect=[0.0, 1.0]
        ):
            self.assertTrue(controller.drive({"position": None}, 130))
            self.assertFalse(controller.drive({"position": None}, 130))


if __name__ == "__main__":
    unittest.main()

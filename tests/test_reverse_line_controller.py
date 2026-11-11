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

    def test_unbounded_line_loss_keeps_driving_for_cube1(self):
        controller = self.nav.ReverseLineController(
            StaticHeading(), line_loss_timeout=None
        )
        with patch.object(
            self.nav.time, "monotonic", side_effect=[0.0, 100.0]
        ):
            self.assertTrue(controller.drive({"position": None}, 130))
            self.assertTrue(controller.drive({"position": None}, 130))
        self.assertEqual(self.nav.motors.set_speeds.call_count, 2)

    def test_button_timeout_allows_longer_gyro_reacquisition(self):
        controller = self.nav.ReverseLineController(
            StaticHeading(),
            line_loss_timeout=self.nav.BUTTON_REVERSE_LINE_LOSS_TIMEOUT,
        )
        with patch.object(
            self.nav.time, "monotonic", side_effect=[0.0, 4.9, 5.1]
        ):
            self.assertTrue(controller.drive({"position": None}, 130))
            self.assertTrue(controller.drive({"position": None}, 130))
            self.assertFalse(controller.drive({"position": None}, 130))

    def test_button_edge_recovery_turns_toward_right_line_while_reversing(self):
        controller = self.nav.ReverseLineController(
            StaticHeading(), edge_recovery=True
        )
        with patch.object(self.nav.time, "monotonic", return_value=1.0):
            self.assertTrue(controller.drive({"position": 2.5}, 115))

        left, right = self.nav.motors.set_speeds.call_args.args
        self.assertEqual((left, right), (-150, -80))

    def test_button_edge_recovery_turns_toward_left_line_while_reversing(self):
        controller = self.nav.ReverseLineController(
            StaticHeading(), edge_recovery=True
        )
        with patch.object(self.nav.time, "monotonic", return_value=1.0):
            self.assertTrue(controller.drive({"position": -2.5}, 115))

        self.assertEqual(self.nav.motors.set_speeds.call_args.args, (-80, -150))


if __name__ == "__main__":
    unittest.main()

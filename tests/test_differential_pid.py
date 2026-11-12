"""Hardware-free unit tests for standalone simple-pid steering behavior."""

import unittest

from differential2 import LinePID, BASE_SPEED, MAX_CORRECTION


class PIDSteeringTests(unittest.TestCase):
    def test_center_outputs_equal_speeds(self):
        pid = LinePID()
        left, right, _ = pid.command(0.0)
        self.assertEqual((left, right), (BASE_SPEED, BASE_SPEED))

    def test_positive_position_accelerates_left(self):
        pid = LinePID()
        left, right, correction = pid.command(1.0)
        self.assertGreater(correction, 0)
        self.assertGreater(left, right)

    def test_negative_position_accelerates_right(self):
        pid = LinePID()
        left, right, correction = pid.command(-1.0)
        self.assertLess(correction, 0)
        self.assertLess(left, right)

    def test_missing_position_cancels_output_and_resets(self):
        pid = LinePID()
        pid.command(2.0)
        self.assertIsNone(pid.command(None))
        self.assertEqual(pid.command(0.0)[:2], (BASE_SPEED, BASE_SPEED))

    def test_limits_keep_both_wheels_nonnegative(self):
        pid = LinePID(kp=500, ki=0, kd=0)
        left, right, correction = pid.command(3.5, speed=100, max_speed=170)
        self.assertLessEqual(abs(correction), MAX_CORRECTION)
        self.assertTrue(0 <= left <= 170)
        self.assertTrue(0 <= right <= 170)


if __name__ == "__main__":
    unittest.main()

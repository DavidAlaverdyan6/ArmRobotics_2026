"""Hardware-independent checks for main's shared 255-PWM PID."""
import unittest

from forward_line_pid import ForwardLinePID
from differential2 import LinePID


class DummyMotor:
    def __init__(self):
        self.values = []
        self.stopped = False

    def set_speeds(self, left, right):
        self.values.append((left, right))

    def motor_stop(self):
        self.stopped = True


class ForwardPIDTests(unittest.TestCase):
    def test_matches_standalone_at_constant_position(self):
        for position in (-3.5, -1.0, 0.0, 0.6, 2.0, 3.5):
            motor = DummyMotor()
            shared = ForwardLinePID(motor)
            standalone = LinePID()
            expected = standalone.command(position)
            actual = shared.command(position)
            self.assertEqual(actual[:2], expected[:2])

    def test_zero_error_full_speed(self):
        motor = DummyMotor()
        shared = ForwardLinePID(motor)
        self.assertTrue(shared.drive({'position': 0.0}))
        self.assertEqual(motor.values[-1], (255, 255))

    def test_steering_saturates_without_reverse(self):
        motor = DummyMotor()
        shared = ForwardLinePID(motor)
        self.assertTrue(shared.drive({'position': 3.5}))
        left, right = motor.values[-1]
        self.assertTrue(0 <= left <= 255 and 0 <= right <= 255)
        self.assertGreater(left, right)

    def test_no_line_stops_and_resets(self):
        motor = DummyMotor()
        shared = ForwardLinePID(motor)
        shared.drive({'position': 1.0})
        self.assertFalse(shared.drive({'position': None}))
        self.assertTrue(motor.stopped)
        self.assertTrue(shared.drive({'position': 0.0}))
        self.assertEqual(motor.values[-1], (255,255))


if __name__=='__main__':
    unittest.main()

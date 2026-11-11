"""Hardware-independent line tracker and motor safety checks."""

import threading
import unittest
from unittest.mock import Mock

import line_tracker
import motors


class MainSafetyTests(unittest.TestCase):
    def test_zero_adc_values_are_valid_callbacks(self):
        board = Mock()
        callbacks = {}
        board.set_pin_mode_analog_input.side_effect = (
            lambda pin, callback, differential=0:
            callbacks.__setitem__(pin, callback)
        )
        line_tracker.initialize(board)
        try:
            for pin in line_tracker.SENSOR_PINS:
                callbacks[pin]([2, pin, 0])
            self.assertTrue(line_tracker.all_channels_fresh())
            self.assertEqual(line_tracker.sensor_values, [0] * 8)
            self.assertEqual(line_tracker.sample_ids, [1] * 8)
        finally:
            line_tracker.shutdown()

    def test_unarmed_and_stale_motor_commands_rejected(self):
        board = Mock()
        motors.initialize(board)
        motors.enable_guard(lambda: False)
        with self.assertRaises(RuntimeError):
            motors.set_speeds(100, 100)
        motors.arm()
        with self.assertRaises(RuntimeError):
            motors.set_speeds(100, 100)
        self.assertFalse(motors._armed)
        self.assertEqual(board.analog_write.call_args_list[-1].args[1], 0)

    def test_other_thread_cannot_command_drive(self):
        board = Mock()
        motors.initialize(board)
        motors.enable_guard(lambda: True)
        motors.arm()
        errors = []

        def command_from_other_thread():
            try:
                motors.set_speeds(100, 100)
            except Exception as exc:
                errors.append(exc)

        thread = threading.Thread(target=command_from_other_thread)
        thread.start()
        thread.join()
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], RuntimeError)
        motors.disarm()


if __name__ == "__main__":
    unittest.main()

"""Verify the line tracker uses ADC.py-compatible Telemetrix callbacks."""

import unittest

import line_tracker


class FakeBoard:
    def __init__(self):
        self.inputs = []

    # Deliberately omit a differential argument: ADC.py does the same.
    def set_pin_mode_analog_input(self, pin, callback):
        self.inputs.append((pin, callback))


class LineTrackerCallbackTests(unittest.TestCase):
    def tearDown(self):
        line_tracker.shutdown()

    def test_all_eight_callbacks_are_registered_and_zero_is_valid(self):
        board = FakeBoard()
        line_tracker.initialize(board)
        self.assertEqual(len(board.inputs), 8)
        self.assertFalse(line_tracker.all_callbacks_ready())

        for pin, callback in board.inputs:
            callback([0, pin, 0])

        self.assertTrue(line_tracker.all_callbacks_ready())
        self.assertEqual(line_tracker.sensor_values, [0] * 8)

    def test_initialize_resets_previous_callback_status(self):
        first = FakeBoard()
        line_tracker.initialize(first)
        for pin, callback in first.inputs:
            callback([0, pin, 1234])
        self.assertTrue(line_tracker.all_callbacks_ready())

        line_tracker.initialize(FakeBoard())
        self.assertEqual(line_tracker.callback_status(), [False] * 8)

    def test_symmetric_weights_use_zero_as_the_center(self):
        centered = [0, 0, 0, 1, 1, 0, 0, 0]
        self.assertEqual(line_tracker.CENTER_POSITION, 0.0)
        self.assertEqual(line_tracker.calculate_position(centered), 0.0)
        self.assertEqual(
            line_tracker.calculate_position([1, 0, 0, 0, 0, 0, 0, 0]),
            -3.5,
        )
        self.assertEqual(
            line_tracker.calculate_position([0, 0, 0, 0, 0, 0, 0, 1]),
            3.5,
        )

    def test_isolated_floor_patch_is_not_a_crossing(self):
        uneven_floor = [0.92, 0.25, 0, 0.8, 0.8, 0, 0, 0]
        real_left_branch = [0.88, 0.70, 0, 0.8, 0.8, 0, 0, 0]
        self.assertIsNone(line_tracker.detect_crossing(uneven_floor))
        self.assertEqual(
            line_tracker.detect_crossing(real_left_branch),
            "LEFT",
        )


if __name__ == "__main__":
    unittest.main()

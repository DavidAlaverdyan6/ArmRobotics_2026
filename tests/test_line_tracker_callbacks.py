"""Verify the line tracker uses ADC.py-compatible Telemetrix callbacks."""

import unittest
import tempfile
from pathlib import Path

import line_tracker
from pins import PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0


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
        self.assertEqual(
            [pin for pin, _ in board.inputs],
            [PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0],
        )
        self.assertFalse(line_tracker.all_callbacks_ready())

        for pin, callback in board.inputs:
            callback([0, pin, 0])

        self.assertTrue(line_tracker.all_callbacks_ready())
        self.assertEqual(line_tracker.sensor_values, [0] * 8)

    def test_adc_style_callback_pairing_ignores_reported_pin_field(self):
        # ADC.py uses the callback registration index, not data[1], to
        # identify each channel. Some STM32 Telemetrix pin identifiers may
        # differ from the project's internal PA0-PA7 constants.
        board = FakeBoard()
        line_tracker.initialize(board)
        for index, (_, callback) in enumerate(board.inputs):
            callback([0, index, 1000 + index])
        self.assertEqual(line_tracker.callback_status(), [True] * 8)
        self.assertEqual(
            line_tracker.sensor_values, [1000 + i for i in range(8)]
        )

    def test_invalid_adc_value_does_not_mark_callback_ready(self):
        board = FakeBoard()
        line_tracker.initialize(board)
        board.inputs[0][1]([0, 0, 5000])
        self.assertFalse(line_tracker.callback_status()[0])

    def test_calibration_for_old_pin_map_is_rejected(self):
        original_path = line_tracker.CALIBRATION_FILE
        with tempfile.TemporaryDirectory() as directory:
            line_tracker.CALIBRATION_FILE = str(Path(directory) / "calibration.json")
            try:
                old = {
                    "white": [3000] * 8,
                    "gray": [3300] * 8,
                    "black": [3900] * 8,
                }
                line_tracker.save_calibration(old)
                self.assertIsNone(line_tracker.load_calibration())
                old["pins"] = [200, PA6, PA5, PA4, PA3, PA2, PA1, PA0]
                line_tracker.save_calibration(old)
                self.assertIsNone(line_tracker.load_calibration())
                old["pins"] = line_tracker.SENSOR_PINS.copy()
                line_tracker.save_calibration(old)
                self.assertEqual(line_tracker.load_calibration(), old)
            finally:
                line_tracker.CALIBRATION_FILE = original_path

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

    def test_faint_gray_marker_residue_is_not_a_line(self):
        faint_residue = [0, 0.08, 0.18, 0.28, 0.31, 0.20, 0.07, 0]
        self.assertIsNone(line_tracker.calculate_position(faint_residue))

    def test_small_medium_dark_patch_is_not_a_line(self):
        self.assertIsNone(
            line_tracker.calculate_position([0, 0, 0, 0.50, 0, 0, 0, 0])
        )

    def test_narrow_but_truly_black_line_is_still_detected(self):
        self.assertEqual(
            line_tracker.calculate_position([0, 0, 0, 0.90, 0, 0, 0, 0]),
            -0.5,
        )

    def test_real_measured_line_profile_is_still_detected(self):
        measured = [0, 0.10, 0.36, 0.51, 0.53, 0.56, 0.53, 0.41]
        self.assertIsNotNone(line_tracker.calculate_position(measured))

    def test_gray_branch_does_not_pull_position_off_black_tape(self):
        tape = [0, 0, 0, 0.95, 0.95, 0, 0, 0]
        tape_with_gray_branch = [0, 0, 0, 0.95, 0.95, 0, 0.55, 0.55]
        self.assertEqual(
            line_tracker.calculate_position(tape_with_gray_branch),
            line_tracker.calculate_position(tape),
        )

    def test_general_normalization_uses_white_and_black_only(self):
        calibration = {
            "white": [3500] * 8,
            "gray": [3200] * 8,
            "black": [3900] * 8,
        }
        self.assertEqual(
            line_tracker.normalize_sensor(0, 3200, calibration), 0.0
        )
        self.assertEqual(
            line_tracker.normalize_sensor(0, 3500, calibration), 0.0
        )
        self.assertAlmostEqual(
            line_tracker.normalize_sensor(0, 3900, calibration), 1.0
        )
        self.assertAlmostEqual(
            line_tracker.normalize_sensor(0, 3700, calibration), 0.5
        )

    def test_update_does_not_apply_a_global_gray_filter(self):
        board = FakeBoard()
        line_tracker.initialize(board)
        calibration = {
            "white": [3100] * 8,
            "gray": [3650] * 8,
            "black": [3900] * 8,
        }
        for pin, callback in board.inputs:
            callback([0, pin, 3650])

        state = line_tracker.update(calibration)

        self.assertTrue(all(value > 0 for value in state["normalized"]))
        self.assertIsNotNone(state["position"])


if __name__ == "__main__":
    unittest.main()

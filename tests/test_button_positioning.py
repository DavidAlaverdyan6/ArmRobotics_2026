"""Offline checks for the B task's final position before servo motion."""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch


def load_navigation():
    stubs = {name: types.ModuleType(name) for name in
             ("smbus", "telemetrix", "motors", "line_tracker",
              "sensors.vl53l0x_sensor")}
    stubs["telemetrix"].telemetrix = Mock()
    stubs["sensors.vl53l0x_sensor"].VL53L0XSensor = Mock()
    with patch.dict(sys.modules, stubs):
        spec = importlib.util.spec_from_file_location(
            "button_position_navigation",
            Path(__file__).resolve().parents[1] / "main_ggs_interactive.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class ButtonPositionTests(unittest.TestCase):
    def setUp(self):
        self.nav = load_navigation()
        self.centered = {
            "position": 0.08,
            "normalized": [0, 0, 0.2, 0.8, 0.8, 0.2, 0, 0],
        }

    def test_distance_and_line_must_both_match(self):
        nav = self.nav
        self.assertTrue(nav.button_position_ready(self.centered, 21.5))
        self.assertTrue(nav.button_position_ready(self.centered, 21.0))
        self.assertTrue(nav.button_position_ready(self.centered, 22.0))
        self.assertFalse(nav.button_position_ready(self.centered, 20.9))
        self.assertFalse(nav.button_position_ready(self.centered, 22.1))
        off_center = {**self.centered, "position": 0.6}
        self.assertFalse(nav.button_position_ready(off_center, 21.5))

    def test_too_close_reverses_then_confirms_while_stopped(self):
        nav = self.nav
        clock = [0.0]
        nav.stop = Mock()
        nav.differential_drive = Mock(return_value=True)
        nav.differential_reverse = Mock(return_value=True)
        nav.line_tracker.update = Mock(return_value=self.centered)
        detector = Mock()
        detector.read.return_value = (False, False, False)
        sensor = Mock()
        sensor.read_distance_mm.side_effect = (
            [350] * 3 + [200] * 2 + [215] * 6
        )

        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep",
                          side_effect=lambda seconds: clock.__setitem__(
                              0, clock[0] + seconds)):
            self.assertTrue(nav.button_approach(None, detector, sensor))

        nav.differential_reverse.assert_called_once_with(
            self.centered, nav.BUTTON_ADJUST_SPEED
        )
        self.assertGreaterEqual(nav.stop.call_count, 4)
        self.assertEqual(sensor.read_distance_mm.call_count, 11)

    def test_invalid_distance_stops_without_moving(self):
        nav = self.nav
        nav.stop = Mock()
        nav.differential_drive = Mock()
        nav.line_tracker.update = Mock(return_value=self.centered)
        detector = Mock()
        detector.read.return_value = (False, False, False)
        sensor = Mock()
        sensor.read_distance_mm.return_value = 0

        self.assertFalse(nav.button_approach(None, detector, sensor))
        nav.differential_drive.assert_not_called()
        nav.stop.assert_called()

    def test_button_visible_before_junction_clears(self):
        nav = self.nav
        clock = [0.0]
        nav.stop = Mock()
        nav.differential_drive = Mock(return_value=True)
        nav.line_tracker.update = Mock(return_value=self.centered)
        detector = Mock()
        detector.read.side_effect = (
            [(True, False, False)] * 2 + [(False, False, False)] * 7
        )
        sensor = Mock()
        sensor.read_distance_mm.return_value = 215

        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep",
                          side_effect=lambda seconds: clock.__setitem__(
                              0, clock[0] + seconds)):
            self.assertTrue(nav.button_approach(None, detector, sensor))

        self.assertEqual(nav.differential_drive.call_count, 5)
        self.assertTrue(all(
            call.kwargs["speed"] == nav.BUTTON_ADJUST_SPEED
            for call in nav.differential_drive.call_args_list
        ))

    def test_too_close_on_junction_stops(self):
        nav = self.nav
        nav.stop = Mock()
        nav.differential_drive = Mock()
        nav.line_tracker.update = Mock(return_value=self.centered)
        detector = Mock()
        detector.read.return_value = (True, False, False)
        sensor = Mock()
        sensor.read_distance_mm.return_value = 170

        self.assertFalse(nav.button_approach(None, detector, sensor))
        nav.differential_drive.assert_not_called()
        nav.stop.assert_called()


if __name__ == "__main__":
    unittest.main()

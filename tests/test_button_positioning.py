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
        self.assertTrue(nav.button_position_ready(self.centered, 21.0))
        self.assertTrue(nav.button_position_ready(self.centered, 21.0))
        self.assertTrue(nav.button_position_ready(self.centered, 20.0))
        self.assertTrue(nav.button_position_ready(self.centered, 22.0))
        self.assertFalse(nav.button_position_ready(self.centered, 19.9))
        self.assertFalse(nav.button_position_ready(self.centered, 22.1))
        self.assertTrue(nav.button_position_ready(
            {**self.centered, "position": 0.65}, 21.0
        ))
        off_center = {**self.centered, "position": 0.8}
        self.assertFalse(nav.button_position_ready(off_center, 21.0))

    def test_reverse_steers_toward_line_and_stops_if_lost(self):
        nav = self.nav
        nav.motors.set_speeds = Mock()
        nav.stop = Mock()
        nav.differential_reverse({**self.centered, "position": 1.08}, 70)
        left, right = nav.motors.set_speeds.call_args.args
        self.assertTrue(right < left < 0)
        nav.differential_reverse({**self.centered, "position": -0.92}, 70)
        left, right = nav.motors.set_speeds.call_args.args
        self.assertTrue(left < right < 0)
        self.assertFalse(nav.differential_reverse({"position": None}, 70))
        nav.stop.assert_called_once()

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
            [350] * 3 + [180] + [210] * 6
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
        self.assertEqual(sensor.read_distance_mm.call_count, 10)

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
        sensor.read_distance_mm.return_value = 210

        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep",
                          side_effect=lambda seconds: clock.__setitem__(
                              0, clock[0] + seconds)):
            self.assertTrue(nav.button_approach(None, detector, sensor))

        nav.differential_drive.assert_not_called()

    def test_too_close_on_junction_reverses_then_confirms(self):
        nav = self.nav
        clock = [0.0]
        nav.stop = Mock()
        nav.differential_drive = Mock()
        nav.differential_reverse = Mock(return_value=True)
        junction = {**self.centered,
                    "normalized": [1, 0, 0.2, 0.8, 0.8, 0.2, 0, 0]}
        nav.line_tracker.update = Mock(
            side_effect=[junction] + [self.centered] * 6
        )
        detector = Mock()
        detector.read.return_value = (True, False, False)
        sensor = Mock()
        sensor.read_distance_mm.side_effect = [178] + [210] * 6

        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep",
                          side_effect=lambda seconds: clock.__setitem__(
                              0, clock[0] + seconds)):
            self.assertTrue(nav.button_approach(None, detector, sensor))

        nav.differential_reverse.assert_called_once_with(
            junction, nav.BUTTON_ADJUST_SPEED
        )
        nav.differential_drive.assert_not_called()
        nav.stop.assert_called()


if __name__ == "__main__":
    unittest.main()

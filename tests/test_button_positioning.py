"""Offline checks for the bumper-based BUTTON task."""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch


def load_navigation():
    stubs = {name: types.ModuleType(name) for name in (
        "smbus", "telemetrix", "motors", "line_tracker",
        "sensors.vl53l0x_sensor", "led", "robot_pin_config",
    )}
    stubs["telemetrix"].telemetrix = Mock()
    stubs["line_tracker"].LineTrackingEngine = Mock(return_value=Mock())
    stubs["line_tracker"].CENTER_POSITION = 0.0
    stubs["sensors.vl53l0x_sensor"].VL53L0XSensor = Mock()
    with patch.dict(sys.modules, stubs):
        spec = importlib.util.spec_from_file_location(
            "button_bumper_navigation",
            Path(__file__).resolve().parents[1] / "main_test.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class ButtonBumperTests(unittest.TestCase):
    def setUp(self):
        self.nav = load_navigation()
        self.centered = {"position": 0.08}

    def test_full_speed_runs_for_two_seconds_and_reads_distance_continuously(self):
        nav = self.nav
        clock = [0.0]
        nav.stop = Mock()
        nav.differential_drive = Mock()
        nav.line_tracker.update = Mock(return_value=self.centered)
        sensor = Mock()
        # 30 cm approach, 20 cm bumper-run trigger, then many live readings.
        sensor.read_distance_mm.side_effect = [300, 200] + [50] * 110

        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds:
                          clock.__setitem__(0, clock[0] + seconds)):
            self.assertTrue(nav.button_bumper_approach(None, sensor))

        self.assertEqual(
            nav.differential_drive.call_args_list[0].kwargs["speed"],
            nav.MANEUVER_SPEED,
        )
        full_speed_calls = nav.differential_drive.call_args_list[1:]
        self.assertTrue(full_speed_calls)
        self.assertTrue(all(
            call.kwargs["speed"] == nav.BUTTON_BUMPER_SPEED
            and call.kwargs["max_speed"] == nav.BUTTON_BUMPER_SPEED
            for call in full_speed_calls
        ))
        self.assertGreaterEqual(sensor.read_distance_mm.call_count, 100)

    def test_zero_distance_is_read_during_the_timed_bumper_run(self):
        nav = self.nav
        clock = [0.0]
        nav.stop = Mock()
        nav.differential_drive = Mock()
        nav.line_tracker.update = Mock(return_value=self.centered)
        sensor = Mock()
        sensor.read_distance_mm.side_effect = [200] + [0] * 110

        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds:
                          clock.__setitem__(0, clock[0] + seconds)):
            self.assertTrue(nav.button_bumper_approach(None, sensor))
        self.assertGreaterEqual(sensor.read_distance_mm.call_count, 100)

    def test_arm_is_activated_in_base_pose(self):
        arm = Mock()
        home = {
            "base": 90, "shoulder": 170, "elbow": 180,
            "wrist": 0, "gripper": 180,
        }
        with patch.dict(sys.modules, {
            "arm_web": types.SimpleNamespace(START_ANGLES=home),
            "roboarm": types.SimpleNamespace(
                get_shared_arm=Mock(return_value=(arm, True))
            ),
        }):
            self.nav.activate_arm_home(Mock())
        arm.write.assert_called_once_with(**home)

    def test_junction_detection_still_rejects_isolated_outer_patch(self):
        detector = self.nav.JunctionDetector()
        patch_state = {
            "position": -1.4,
            "normalized": [0.9, 0.2, 0, 0.8, 0.8, 0, 0, 0],
        }
        self.assertFalse(detector.read(patch_state)[2])


if __name__ == "__main__":
    unittest.main()

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
            "wrist": 0, "gripper": 170,
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

    def test_junction_rearms_only_after_stable_centered_clear_line(self):
        gate = self.nav.JunctionRearmGate()
        gate.mark_junction()

        gate.update(False, False, 0.8)
        gate.update(False, False, 0.0)
        gate.update(False, False, 0.0)
        self.assertFalse(gate.armed)

        gate.update(False, False, 0.0)
        self.assertTrue(gate.armed)

    def test_reacquire_uses_aggressive_low_speed_steering(self):
        self.nav.line_follower.steering_correction = Mock(return_value=40)
        self.nav.motors.set_speeds = Mock()

        self.assertTrue(self.nav.aggressive_reacquire_drive({"position": 1.0}))

        self.nav.motors.set_speeds.assert_called_once_with(144, 16)

    def test_button_finishes_with_the_tested_r90_turn(self):
        self.nav.pivot = Mock(return_value=(True, 1))
        self.nav.pivot_90 = Mock(return_value=(True, None))
        self.nav.reacquire_line = Mock(return_value=True)
        self.nav.button_bumper_approach = Mock(return_value=True)
        self.nav.button_return = Mock(return_value=True)
        self.nav.follow_for = Mock(return_value=True)

        bus = Mock()
        self.assertTrue(self.nav.button_task(
            None, Mock(), Mock(), bus, 12.0, Mock()
        ))

        self.nav.pivot.assert_called_once_with(bus, 12.0, "L")
        self.nav.button_return.assert_called_once_with(
            None,
            unittest.mock.ANY,
            bus,
            12.0,
            1,
            line_loss_timeout=self.nav.BUTTON_REVERSE_LINE_LOSS_TIMEOUT,
        )
        self.nav.follow_for.assert_called_once_with(
            None, self.nav.RETURN_FORWARD_TIME
        )
        self.nav.pivot_90.assert_called_once_with(
            unittest.mock.ANY,
            12.0,
            "R",
            angle_target=self.nav.TURN_90_ANGLE,
        )


if __name__ == "__main__":
    unittest.main()

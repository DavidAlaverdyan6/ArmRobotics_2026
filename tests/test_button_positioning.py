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

    def test_full_speed_runs_for_one_second_and_reads_distance_continuously(self):
        nav = self.nav
        clock = [0.0]
        nav.stop = Mock()
        nav.differential_drive = Mock()
        nav.line_follower.steering_correction = Mock(return_value=0)
        nav.motors.set_speeds = Mock()
        nav.line_tracker.update = Mock(return_value=self.centered)
        sensor = Mock()
        # 30 cm approach, 20 cm bumper-run trigger, then many live readings.
        sensor.read_distance_mm.side_effect = [300, 200] + [50] * 110

        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds:
                          clock.__setitem__(0, clock[0] + seconds)):
            self.assertTrue(nav.button_bumper_approach(None, sensor))

        nav.motors.set_speeds.assert_called_once_with(
            nav.MANEUVER_SPEED, nav.MANEUVER_SPEED
        )
        full_speed_calls = nav.differential_drive.call_args_list
        self.assertTrue(full_speed_calls)
        self.assertTrue(all(
            call.kwargs["speed"] == nav.BUTTON_BUMPER_SPEED
            and call.kwargs["max_speed"] == nav.BUTTON_BUMPER_SPEED
            for call in full_speed_calls
        ))
        self.assertGreaterEqual(sensor.read_distance_mm.call_count, 50)

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
        self.assertGreaterEqual(sensor.read_distance_mm.call_count, 50)

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

    def test_gray_side_mark_does_not_count_as_a_junction(self):
        detector = self.nav.JunctionDetector()
        gray_side = {
            "position": 0.0,
            "normalized": [0, 0, 0, 0.95, 0.95, 0, 0.66, 0.88],
        }
        self.assertFalse(detector.read(gray_side)[2])

        black_side = {
            "position": 0.0,
            "normalized": [0, 0, 0, 0.95, 0.95, 0, 0.90, 0.98],
        }
        self.assertTrue(detector.read(black_side)[2])

    def test_reacquire_matches_mainv2_differential_centering(self):
        nav = self.nav
        clock = [0.0]
        nav.stop = Mock()
        nav.differential_drive = Mock()
        nav.line_tracker.update = Mock(return_value={"position": 0.4})

        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds:
                          clock.__setitem__(0, clock[0] + seconds)):
            self.assertTrue(nav.reacquire_line(None))

        self.assertEqual(nav.differential_drive.call_count, 4)
        self.assertTrue(all(
            call.kwargs["speed"] == nav.MANEUVER_SPEED
            and call.kwargs["max_speed"] == nav.MANEUVER_MAX_SPEED
            for call in nav.differential_drive.call_args_list
        ))

    def test_button_return_accepts_confirmed_t_junction_on_one_side(self):
        nav = self.nav
        clock = [0.0]
        narrow = {"position": 0.0,
                  "normalized": [0, 0, 0, 0.9, 0.9, 0, 0, 0]}
        edge_only = {"position": 2.5,
                     "normalized": [0, 0, 0, 0, 0, 0, 0.9, 0.9]}
        junction = {"position": 0.0,
                    "normalized": [0, 0, 0, 0.9, 0.9, 0, 0.9, 0.9]}
        nav.line_tracker.update = Mock(side_effect=[
            narrow, narrow, narrow, edge_only, edge_only,
            junction, junction,
        ])
        nav.differential_reverse = Mock(return_value=True)
        detector = Mock()
        detector.read.side_effect = [
            (False, False, False),
            (False, False, False),
            (False, False, False),
            (False, True, False),  # Outer patch alone must not stop it.
            (False, True, False),
            (False, True, True),   # A real branch needs two frames.
            (False, True, True),
        ]

        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds:
                          clock.__setitem__(0, clock[0] + seconds)):
            self.assertTrue(nav.button_return(
                None, detector, Mock(), 0.0, 1,
            ))

        self.assertEqual(detector.read.call_count, 7)

    def test_button_return_arms_with_only_right_sensors_on_line(self):
        nav = self.nav
        clock = [0.0]
        edge = {"position": 2.5,
                "normalized": [0, 0, 0, 0, 0, 0, 0.95, 0.95]}
        wide = {"position": 2.0,
                "normalized": [0, 0, 0, 0.9, 0.9, 0, 0.95, 0.95]}
        nav.line_tracker.update = Mock(side_effect=[
            edge, edge, edge, wide, wide,
        ])
        nav.differential_reverse = Mock(return_value=True)
        detector = Mock()
        detector.read.return_value = (False, True, False)

        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds:
                          clock.__setitem__(0, clock[0] + seconds)):
            self.assertTrue(nav.button_return(
                None, detector, Mock(), 0.0, 1,
            ))

        self.assertEqual(detector.read.call_count, 5)

    def test_button_departure_is_straight_then_checks_line(self):
        nav = self.nav
        clock = [0.0]
        nav.stop = Mock()
        nav.line_follower.drive_straight = Mock()
        nav.line_tracker.update = Mock(return_value={"position": 0.4})

        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds:
                          clock.__setitem__(0, clock[0] + seconds)):
            self.assertTrue(nav.button_depart_and_check_line(None))

        self.assertGreaterEqual(clock[0], nav.BUTTON_DEPART_SECONDS)
        self.assertTrue(all(
            call.kwargs == {
                "speed": nav.MANEUVER_SPEED,
                "max_speed": nav.MANEUVER_MAX_SPEED,
            }
            for call in nav.line_follower.drive_straight.call_args_list
        ))
        nav.line_tracker.update.assert_called_once_with(None)

    def test_button_approach_keeps_both_wheels_forward_on_edge_line(self):
        nav = self.nav
        nav.line_follower.steering_correction = Mock(return_value=200)
        nav.motors.set_speeds = Mock()

        nav.button_safe_forward_drive({"position": 2.5})

        nav.motors.set_speeds.assert_called_once_with(160, 40)

    def test_button_finishes_with_the_tested_r90_turn(self):
        self.nav.pivot = Mock(return_value=(True, 1))
        self.nav.pivot_90 = Mock(return_value=(True, None))
        self.nav.reacquire_line = Mock(return_value=True)
        self.nav.button_depart_and_check_line = Mock(return_value=True)
        self.nav.button_bumper_approach = Mock(return_value=True)
        self.nav.button_return = Mock(return_value=True)
        self.nav.follow_for = Mock(return_value=True)

        bus = Mock()
        self.assertTrue(self.nav.button_task(
            None, Mock(), Mock(), bus, 12.0, Mock()
        ))

        self.nav.pivot.assert_called_once_with(bus, 12.0, "L")
        self.nav.button_depart_and_check_line.assert_called_once_with(None)
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

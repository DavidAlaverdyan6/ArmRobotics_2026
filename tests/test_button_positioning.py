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
            Path(__file__).resolve().parents[1] / "main.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class ButtonBumperTests(unittest.TestCase):
    def setUp(self):
        self.nav = load_navigation()
        self.centered = {"position": 0.08}

    def test_bumper_speed_runs_for_one_second_and_reads_distance_continuously(self):
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
        bumper_speed_calls = nav.differential_drive.call_args_list
        self.assertTrue(bumper_speed_calls)
        self.assertTrue(all(
            call.kwargs["speed"] == nav.BUTTON_BUMPER_SPEED
            and call.kwargs["max_speed"] == nav.BUTTON_BUMPER_SPEED
            for call in bumper_speed_calls
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

    def test_fast_route_junction_requires_full_black_branch_and_center(self):
        nav = self.nav
        complete = {
            "position": 0.0,
            "normalized": [0, 0, 0, 0.9, 0.9, 0, 0.8, 0.96],
        }
        incomplete = {
            "position": 0.0,
            "normalized": [0, 0, 0, 0.9, 0.9, 0, 0.55, 0.96],
        }
        self.assertTrue(nav.strong_route_junction(complete))
        self.assertFalse(nav.strong_route_junction(incomplete))

    def test_route_recovery_requires_three_fresh_line_frames(self):
        nav = self.nav
        nav.stop = Mock()
        clock = [0.0]
        positions_and_ids = [
            (None, 1), (0.1, 2), (0.1, 2), (0.1, 3), (0.1, 4),
        ]
        states = [
            {
                "position": position,
                "sample_ids": [sample_id] * 8,
                "sample_times": [0.0] * 8,
            }
            for position, sample_id in positions_and_ids
        ]
        nav.line_tracker.update = Mock(side_effect=states)
        nav.line_tracker.all_channels_fresh = Mock(return_value=True)
        nav.line_follower.reverse_straight = Mock()
        with patch.object(
            nav.time, "monotonic", side_effect=lambda: clock[0]
        ), patch.object(
            nav.time, "sleep",
            side_effect=lambda seconds: clock.__setitem__(
                0, clock[0] + seconds
            ),
        ):
            self.assertTrue(
                nav.reverse_to_line_on_route(None, nav.FreshFrames())
            )
        self.assertEqual(nav.line_tracker.update.call_count, 5)
        self.assertEqual(
            nav.line_follower.reverse_straight.call_count, 4
        )
        nav.line_follower.reverse_straight.assert_any_call(80)

    def test_route_recovery_stops_after_five_seconds(self):
        nav = self.nav
        nav.stop = Mock()
        clock = [0.0]
        sample_id = [0]

        def missing_state(_calibration):
            sample_id[0] += 1
            return {
                "position": None,
                "sample_ids": [sample_id[0]] * 8,
                "sample_times": [clock[0]] * 8,
            }

        nav.line_tracker.update = Mock(side_effect=missing_state)
        nav.line_tracker.all_channels_fresh = Mock(return_value=True)
        nav.line_follower.reverse_straight = Mock()
        with patch.object(
            nav.time, "monotonic", side_effect=lambda: clock[0]
        ), patch.object(
            nav.time, "sleep",
            side_effect=lambda seconds: clock.__setitem__(
                0, clock[0] + seconds
            ),
        ):
            self.assertFalse(
                nav.reverse_to_line_on_route(None, nav.FreshFrames())
            )
        self.assertGreaterEqual(clock[0], 5.0)
        nav.stop.assert_called()

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

    def test_reacquire_matches_main_differential_centering(self):
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

    def test_route_turn_reverses_only_after_one_second_without_line(self):
        nav = self.nav
        clock = [0.0]
        missing = {"position": None}
        centered = {"position": 0.0}
        nav.line_tracker.update = Mock(side_effect=(
            [missing] * 26 + [missing] + [centered] * 8
        ))
        nav.line_follower.reverse_straight = Mock()
        nav.differential_drive = Mock()
        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds:
                          clock.__setitem__(0, clock[0] + seconds)):
            self.assertTrue(nav.reacquire_line(
                None, recover_after_line_loss=True
            ))
        self.assertGreaterEqual(clock[0], nav.ROUTE_TURN_LINE_LOSS_SECONDS)
        nav.line_follower.drive_straight.assert_not_called()
        self.assertGreater(nav.line_follower.reverse_straight.call_count, 0)
        nav.line_follower.drive_straight.assert_not_called()
        self.assertTrue(
            all(
                drive_call.args == (nav.ROUTE_TURN_REVERSE_SPEED,)
                for drive_call in nav.line_follower.reverse_straight.call_args_list
            )
        )
        self.assertGreaterEqual(nav.differential_drive.call_count, 4)

    def test_route_turn_stops_when_reverse_cannot_find_line(self):
        nav = self.nav
        clock = [0.0]
        nav.line_tracker.update = Mock(return_value={"position": None})
        nav.line_follower.reverse_straight = Mock()
        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds:
                          clock.__setitem__(0, clock[0] + seconds)):
            self.assertFalse(nav.reacquire_line(
                None, recover_after_line_loss=True
            ))
        self.assertGreater(nav.line_follower.reverse_straight.call_count, 0)
        self.assertGreaterEqual(
            clock[0], nav.ROUTE_TURN_LINE_LOSS_SECONDS
            + nav.ROUTE_TURN_REVERSE_TIMEOUT
        )

    def test_button_return_accepts_confirmed_t_junction_on_one_side(self):
        nav = self.nav
        clock = [0.0]
        narrow = {"position": 0.0,
                  "normalized": [0, 0, 0, 0.9, 0.9, 0, 0, 0]}
        edge_only = {"position": 2.5,
                     "normalized": [0, 0, 0, 0, 0, 0, 0.9, 0.9]}
        junction = {"position": 0.0,
                    "normalized": [0, 0, 0, 0.9, 0.9, 0, 0.9, 0.9]}
        nav.line_tracker.update = Mock(side_effect=(
            [narrow] * 3 + [edge_only] * 2 + [junction] * 30
        ))
        nav.differential_reverse = Mock(return_value=True)
        detector = Mock()
        detector.read.side_effect = lambda state: (
            False, state is not narrow, state is junction
        )

        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds:
                          clock.__setitem__(0, clock[0] + seconds)):
            self.assertTrue(nav.button_return(
                None, detector, Mock(), 0.0, 1,
            ))

        self.assertGreaterEqual(clock[0], nav.BUTTON_RETURN_JUNCTION_DELAY)
        self.assertGreater(detector.read.call_count, 7)

    def test_button_return_arms_with_only_right_sensors_on_line(self):
        nav = self.nav
        clock = [0.0]
        edge = {"position": 2.5,
                "normalized": [0, 0, 0, 0, 0, 0, 0.95, 0.95]}
        wide = {"position": 2.0,
                "normalized": [0, 0, 0, 0.9, 0.9, 0, 0.95, 0.95]}
        nav.line_tracker.update = Mock(side_effect=[edge] * 3 + [wide] * 30)
        nav.differential_reverse = Mock(return_value=True)
        detector = Mock()
        detector.read.side_effect = lambda state: (
            False, True, state is wide
        )

        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds:
                          clock.__setitem__(0, clock[0] + seconds)):
            self.assertTrue(nav.button_return(
                None, detector, Mock(), 0.0, 1,
            ))

        self.assertGreaterEqual(clock[0], nav.BUTTON_RETURN_JUNCTION_DELAY)
        self.assertGreater(detector.read.call_count, 5)

    def test_button_return_does_not_accept_wide_straight_line_as_junction(self):
        nav = self.nav
        clock = [0.0]
        wide_straight = {
            "position": 0.0,
            "normalized": [0, 0, 0.9, 0.9, 0.9, 0.9, 0, 0],
        }
        nav.line_tracker.update = Mock(return_value=wide_straight)
        nav.differential_reverse = Mock(return_value=True)
        detector = nav.JunctionDetector()

        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds:
                          clock.__setitem__(0, clock[0] + seconds)):
            self.assertFalse(nav.button_return(
                None, detector, Mock(), 0.0, 1,
                max_return_time=nav.BUTTON_RETURN_JUNCTION_DELAY + 0.2,
            ))

        self.assertGreater(clock[0], nav.BUTTON_RETURN_JUNCTION_DELAY)

    def test_button_return_rejects_small_early_black_patch(self):
        nav = self.nav
        clock = [0.0]
        narrow = {
            "position": 0.0,
            "normalized": [0, 0, 0, 0.9, 0.9, 0, 0, 0],
        }
        patch_state = {
            "position": 1.0,
            # Outer branch plus only one central sensor: not the main T-line.
            "normalized": [0, 0, 0, 0, 0.9, 0, 0.95, 0.95],
        }
        nav.line_tracker.update = Mock(
            side_effect=[narrow] * 4 + [patch_state] * 30
        )
        nav.differential_reverse = Mock(return_value=True)
        detector = Mock()
        detector.read.side_effect = lambda state: (
            False, state is patch_state, state is patch_state
        )
        with patch.object(
            nav.time, "monotonic", side_effect=lambda: clock[0]
        ), patch.object(
            nav.time, "sleep",
            side_effect=lambda seconds: clock.__setitem__(
                0, clock[0] + seconds
            ),
        ):
            self.assertFalse(nav.button_return(
                None, detector, Mock(), 0.0, 1, max_return_time=0.6
            ))

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
        self.nav.pivot_90 = Mock(side_effect=[(True, 1), (True, None)])
        self.nav.reacquire_line = Mock(return_value=True)
        self.nav.button_bumper_approach = Mock(return_value=True)
        self.nav.button_return = Mock(return_value=True)
        self.nav.follow_for = Mock(return_value=True)

        bus = Mock()
        self.assertTrue(self.nav.button_task(
            None, Mock(), Mock(), bus, 12.0, Mock()
        ))

        self.assertEqual(
            self.nav.pivot_90.call_args_list[0],
            unittest.mock.call(
                bus, 12.0, "L", angle_target=self.nav.TURN_90_ANGLE
            ),
        )
        self.nav.button_return.assert_called_once_with(
            None,
            unittest.mock.ANY,
            bus,
            12.0,
            1,
            line_loss_timeout=self.nav.BUTTON_REVERSE_LINE_LOSS_TIMEOUT,
        )
        self.assertEqual(
            self.nav.follow_for.call_args_list,
            [
                unittest.mock.call(
                    None,
                    self.nav.TURN_90_FORWARD_TIME,
                    speed=self.nav.TURN_90_FORWARD_SPEED,
                    max_speed=self.nav.TURN_90_FORWARD_MAX_SPEED,
                ),
                unittest.mock.call(None, self.nav.RETURN_FORWARD_TIME),
            ],
        )
        self.assertEqual(
            self.nav.pivot_90.call_args_list[1],
            unittest.mock.call(
                bus, 12.0, "R", angle_target=self.nav.TURN_90_ANGLE
            ),
        )


if __name__ == "__main__":
    unittest.main()

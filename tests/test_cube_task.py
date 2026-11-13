"""Offline checks for the complete CUBE1 state machine."""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, call, patch


ROOT = Path(__file__).resolve().parents[1]


def load_main():
    stubs = {name: types.ModuleType(name) for name in (
        "smbus", "telemetrix", "motors", "line_tracker",
        "sensors.vl53l0x_sensor",
    )}
    stubs["telemetrix"].telemetrix = Mock()
    stubs["line_tracker"].LineTrackingEngine = Mock(return_value=Mock())
    stubs["line_tracker"].CENTER_POSITION = 0.0
    stubs["sensors.vl53l0x_sensor"].VL53L0XSensor = Mock()
    with patch.dict(sys.modules, stubs):
        spec = importlib.util.spec_from_file_location(
            "cube1_main", ROOT / "main.py"
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class Cube1TaskTests(unittest.TestCase):
    def setUp(self):
        self.nav = load_main()
        self.nav.stop = Mock()

    def test_arm_uses_requested_xyz_pitch_and_open_gripper(self):
        move = Mock(return_value=True)
        with patch.dict(sys.modules, {
            "button_arm": types.SimpleNamespace(move_to_xyz_and_hold=move),
        }):
            self.assertTrue(self.nav.cube1_arm_action(Mock()))
        move.assert_called_once_with(
            unittest.mock.ANY,
            350.0,
            0.0,
            -140.0,
            tool_pitch=-10.0,
            gripper=0.0,
            open_before_move=True,
        )

    def test_arm_folds_home_before_final_turn(self):
        arm = Mock()
        move_home = Mock()
        home = {
            "base": 90, "shoulder": 170, "elbow": 180,
            "wrist": 0, "gripper": 180,
        }
        with patch.dict(sys.modules, {
            "arm_web": types.SimpleNamespace(
                START_ANGLES=home, AUTO_MAX_STEP_DEG=2, STEP_DELAY=0.03
            ),
            "button_arm": types.SimpleNamespace(_return_home=move_home),
            "roboarm": types.SimpleNamespace(
                get_shared_arm=Mock(return_value=(arm, False))
            ),
        }):
            self.assertTrue(self.nav.cube1_fold_arm(Mock()))
        move_home.assert_called_once_with(arm, home, 2, 0.03)

    def test_right_turn_stops_when_center_sensors_find_new_line(self):
        clock = [0.0]
        old_line = {
            "normalized": [0, 0, 0.9, 0.9, 0.9, 0.9, 0, 0],
        }
        clear = {"normalized": [0] * 8}
        partial_center = {
            "normalized": [0, 0, 0, 0.9, 0.9, 0, 0, 0],
        }
        new_line = {
            "normalized": [0, 0, 0.9, 0.9, 0.9, 0.9, 0, 0],
        }
        states = (
            [old_line]
            + [clear] * self.nav.CUBE1_TURN_CLEAR_CONFIRM
            + [partial_center] * self.nav.CUBE1_TURN_LINE_CONFIRM
            + [new_line] * self.nav.CUBE1_TURN_LINE_CONFIRM
        )
        distance_sensor = Mock()
        # Out-of-range laser data is logged but must not delay the line stop.
        distance_sensor.read_distance_mm.return_value = 800.0
        self.nav.line_tracker.update = Mock(side_effect=states)
        self.nav.gyro_rate = Mock(return_value=-500.0)
        self.nav.motors.set_speeds = Mock()
        with patch.object(
            self.nav.time, "monotonic", side_effect=lambda: clock[0]
        ), patch.object(
            self.nav.time, "sleep",
            side_effect=lambda seconds: clock.__setitem__(
                0, clock[0] + seconds
            ),
        ):
            self.assertEqual(
                self.nav.cube1_turn_to_center_line(
                    None, Mock(), 12.0, distance_sensor
                ),
                (True, 1),
            )
        self.assertEqual(
            self.nav.line_tracker.update.call_count,
            len(states),
        )
        self.nav.motors.set_speeds.assert_called_with(
            self.nav.CUBE1_TURN_SPEED,
            -self.nav.CUBE1_TURN_SPEED,
        )

    def test_turn_stops_at_90_even_without_line_or_laser(self):
        clock = [0.0]
        self.nav.line_tracker.update = Mock(
            return_value={"normalized": [0] * 8}
        )
        self.nav.gyro_rate = Mock(return_value=-500.0)
        self.nav.motors.set_speeds = Mock()
        distance_sensor = Mock()
        distance_sensor.read_distance_mm.return_value = 800.0
        with patch.object(
            self.nav.time, "monotonic", side_effect=lambda: clock[0]
        ), patch.object(
            self.nav.time, "sleep",
            side_effect=lambda seconds: clock.__setitem__(
                0, clock[0] + seconds
            ),
        ):
            self.assertEqual(
                self.nav.cube1_turn_to_center_line(
                    None, Mock(), 12.0, distance_sensor
                ),
                (True, 1),
            )
        self.assertLessEqual(clock[0], 0.20)

    def test_laser_requires_consecutive_hits_inside_point_two_seconds(self):
        samples = []
        record = self.nav.cube1_record_laser_sample
        self.assertFalse(record(samples, 0.00, 150.0))
        self.assertFalse(record(samples, 0.03, 150.0))
        # One random outside value cancels the partial confirmation.
        self.assertFalse(record(samples, 0.05, 900.0))
        self.assertFalse(record(samples, 0.07, 150.0))
        self.assertFalse(record(samples, 0.10, 150.0))
        self.assertTrue(record(samples, 0.13, 150.0))
        # Old hits outside the 0.2-second window cannot count.
        self.assertFalse(record(samples, 0.40, 150.0))

    def test_cube_detection_pause_then_junction_stops_before_crossing(self):
        centered = {"position": 0.0}
        self.nav.line_tracker.update = Mock(return_value=centered)
        detector = Mock()
        detector.read.side_effect = [
            (False, False, False),
            (False, False, False),
            (False, False, False),
            (True, True, True),
            (True, True, True),
        ]
        sensor = Mock()
        sensor.detected.return_value = True
        self.nav.differential_drive = Mock()
        with patch.object(self.nav.time, "sleep"):
            self.assertEqual(
                self.nav.cube1_carry_until_drop(None, detector, sensor),
                "junction",
            )
        self.nav.differential_drive.assert_called_with(
            centered,
            speed=self.nav.CUBE1_APPROACH_SPEED,
            max_speed=self.nav.CUBE1_APPROACH_MAX_SPEED,
        )

    def test_junction_before_cube_is_ignored(self):
        centered = {"position": 0.0}
        self.nav.line_tracker.update = Mock(return_value=centered)
        detector = Mock()
        detector.read.side_effect = [
            (False, False, False),
            (True, True, True),
            (True, True, True),
            (False, False, False),
            (False, False, False),
            (False, False, False),
            (True, True, True),
            (True, True, True),
        ]
        sensor = Mock()
        sensor.detected.side_effect = [
            False, False, False, True, True, True,
        ]
        self.nav.differential_drive = Mock()
        with patch.object(self.nav.time, "sleep"):
            self.assertEqual(
                self.nav.cube1_carry_until_drop(None, detector, sensor),
                "junction",
            )
        self.assertEqual(sensor.detected.call_count, 6)

    def test_ir_sensor_is_ignored_after_cube_confirmation_until_junction(self):
        centered = {"position": 0.0}
        self.nav.line_tracker.update = Mock(return_value=centered)
        detector = Mock()
        detector.read.side_effect = [
            (False, False, False),
            (False, False, False),
            (False, False, False),
            (False, False, False),
            (True, True, True),
            (True, True, True),
        ]
        sensor = Mock()
        sensor.detected.side_effect = [True, True, True]
        self.nav.differential_drive = Mock()
        with patch.object(self.nav.time, "sleep"):
            self.assertEqual(
                self.nav.cube1_carry_until_drop(None, detector, sensor),
                "junction",
            )
        self.assertEqual(sensor.detected.call_count, 3)

    def test_brief_line_dropout_does_not_abort_cube_search(self):
        centered = {"position": 0.0}
        missing = {"position": None}
        self.nav.line_tracker.update = Mock(side_effect=[
            centered, centered, centered, missing, missing, centered, centered,
        ])
        detector = Mock()
        detector.read.side_effect = [
            (False, False, False),
            (False, False, False),
            (False, False, False),
            (False, False, False),
            (False, False, False),
            (True, True, True),
            (True, True, True),
        ]
        sensor = Mock()
        sensor.detected.side_effect = [True, True, True]
        self.nav.differential_drive = Mock()
        with patch.object(self.nav.time, "sleep"):
            self.assertEqual(
                self.nav.cube1_carry_until_drop(None, detector, sensor),
                "junction",
            )
        self.assertEqual(self.nav.line_follower.drive_straight.call_count, 2)

    def test_sustained_line_loss_stops_cube_search(self):
        missing = {"position": None}
        self.nav.line_tracker.update = Mock(return_value=missing)
        detector = Mock()
        detector.read.return_value = (False, False, False)
        sensor = Mock()
        sensor.detected.return_value = False
        with patch.object(self.nav.time, "sleep"):
            self.assertIsNone(
                self.nav.cube1_carry_until_drop(None, detector, sensor)
            )
        self.assertEqual(
            self.nav.line_follower.drive_straight.call_count,
            self.nav.CUBE1_LINE_LOSS_CONFIRM - 1,
        )

    def test_final_push_reports_cube_still_present_after_timeout(self):
        clock = [0.0]
        self.nav.line_tracker.update = Mock(return_value={"position": 0.0})
        self.nav.differential_drive = Mock()
        sensor = Mock()
        sensor.detected.return_value = True
        indicator = Mock()
        with patch.object(
            self.nav.time, "monotonic", side_effect=lambda: clock[0]
        ), patch.object(
            self.nav.time, "sleep",
            side_effect=lambda seconds: clock.__setitem__(
                0, clock[0] + seconds
            ),
        ):
            self.assertTrue(
                self.nav.cube1_push_past_junction(
                    None, sensor, indicator=indicator
                )
            )
        self.assertTrue(self.nav.differential_drive.called)
        self.assertGreaterEqual(clock[0], self.nav.CUBE1_DROP_PUSH_TIMEOUT)
        self.assertGreater(indicator.orange.call_count, 1)
        self.assertGreater(indicator.off.call_count, 1)

    def test_final_push_stops_early_when_cube_drops(self):
        clock = [0.0]
        self.nav.line_tracker.update = Mock(return_value={"position": 0.0})
        self.nav.differential_drive = Mock()
        sensor = Mock()
        sensor.detected.side_effect = [False, False, False]
        with patch.object(
            self.nav.time, "monotonic", side_effect=lambda: clock[0]
        ), patch.object(
            self.nav.time, "sleep",
            side_effect=lambda seconds: clock.__setitem__(
                0, clock[0] + seconds
            ),
        ):
            self.assertFalse(
                self.nav.cube1_push_past_junction(None, sensor)
            )
        self.assertLess(clock[0], self.nav.CUBE1_DROP_PUSH_TIMEOUT)
        self.assertEqual(sensor.detected.call_count, 3)
        self.assertTrue(self.nav.differential_drive.called)
        for drive_call in self.nav.differential_drive.call_args_list:
            self.assertEqual(
                drive_call.kwargs["speed"], self.nav.CUBE1_DROP_PUSH_SPEED
            )
            self.assertEqual(
                drive_call.kwargs["max_speed"],
                self.nav.CUBE1_DROP_PUSH_MAX_SPEED,
            )

    def test_return_accepts_only_s8_for_cube1(self):
        clock = [0.0]
        self.nav.line_tracker.update = Mock(return_value={"position": 0.0})
        self.nav.differential_reverse = Mock()
        heading = Mock()
        heading.read.return_value = 0.0
        self.nav.Heading = Mock(return_value=heading)
        detector = Mock()
        detector.read.side_effect = [
            (False, False, False),
            (False, False, False),
            (False, False, False),
            # S1 alone must be ignored.
            (True, False, True),
            (True, False, True),
            (False, False, False),
            # Two stable S8 frames complete the CUBE1 return.
            (False, True, True),
            (False, True, True),
        ]
        with patch.object(
            self.nav.time, "monotonic", side_effect=lambda: clock[0]
        ), patch.object(
            self.nav.time,
            "sleep",
            side_effect=lambda seconds: clock.__setitem__(
                0, clock[0] + seconds
            ),
        ):
            self.assertTrue(self.nav.button_return(
                None,
                detector,
                Mock(),
                0.0,
                1,
                label="CUBE1",
                accept_s8_only=True,
            ))

    def test_return_uses_line_not_gyro_when_line_is_visible(self):
        clock = [0.0]
        state = {
            "position": 0.4,
            "normalized": [0, 0, 0, 0.9, 0.9, 0, 0.9, 0.9],
        }
        self.nav.line_tracker.update = Mock(return_value=state)
        self.nav.differential_reverse = Mock()
        heading = Mock()
        # A large value used to trip the 30-degree gyro abort.
        heading.read.return_value = 45.0
        self.nav.Heading = Mock(return_value=heading)
        detector = Mock()
        detector.read.side_effect = (
            [(False, False, False)] * 3
            + [(True, True, True)] * 30
        )
        with patch.object(
            self.nav.time, "monotonic", side_effect=lambda: clock[0]
        ), patch.object(
            self.nav.time,
            "sleep",
            side_effect=lambda seconds: clock.__setitem__(
                0, clock[0] + seconds
            ),
        ):
            self.assertTrue(self.nav.button_return(
                None, detector, Mock(), 0.0, 1
            ))
        for called_state, called_speed, controller in (
            call.args for call in self.nav.differential_reverse.call_args_list
        ):
            self.assertIs(called_state, state)
            self.assertEqual(called_speed, self.nav.RETURN_SPEED)
            self.assertIsInstance(controller, self.nav.ReverseLineController)

    def test_junction_endpoint_crosses_then_uses_final_r90(self):
        events = []
        self.nav.cube1_turn_to_center_line = Mock(
            side_effect=lambda *args: events.append("turn_to_line") or (True, -1)
        )
        self.nav.cube1_arm_action = Mock(
            side_effect=lambda *args: events.append("arm") or True
        )
        self.nav.cube1_carry_until_drop = Mock(
            side_effect=lambda *args, **kwargs: (
                kwargs["on_cube_found"](), events.append("drop"), "junction"
            )[-1]
        )
        self.nav.cube1_push_past_junction = Mock(
            side_effect=lambda *args, **kwargs: events.append("push") or False
        )
        self.nav.button_return = Mock(
            side_effect=lambda *args, **kwargs:
            events.append("reverse") or True
        )
        self.nav.cube1_fold_arm = Mock(
            side_effect=lambda *args, **kwargs: events.append("fold") or True
        )
        self.nav.reacquire_line = Mock(
            side_effect=lambda *args, **kwargs:
            events.append("reacquire") or True
        )
        self.nav.follow_for = Mock(
            side_effect=lambda *args, **kwargs:
            events.append("forward") or True
        )
        self.nav.pivot_90 = Mock(
            side_effect=lambda *args, **kwargs:
            events.append("r90") or (True, -1)
        )
        self.nav.pivot = Mock(return_value=(True, None))
        rgb = Mock()
        rgb.orange.side_effect = lambda: events.append("orange")
        self.nav.cube1_create_indicator = Mock(return_value=rgb)

        cube_sensor = Mock()
        with patch.object(self.nav.time, "sleep"), patch.dict(sys.modules, {
            "firefighter": types.SimpleNamespace(
                Firefighter=Mock(return_value=cube_sensor)
            ),
        }):
            self.assertTrue(self.nav.cube1_task(
                None, Mock(), Mock(), 12.0, Mock(), Mock()
            ))
        cube_sensor.initialize.assert_called_once_with()
        cube_sensor.wait_for_sample.assert_called_once()
        self.assertIn("after", cube_sensor.wait_for_sample.call_args.kwargs)
        self.assertLess(events.index("arm"), events.index("drop"))

        self.assertEqual(self.nav.pivot_90.call_count, 1)
        for pivot_call in self.nav.pivot_90.call_args_list:
            self.assertEqual(pivot_call, call(
                unittest.mock.ANY,
                12.0,
                "R",
                angle_target=self.nav.TURN_90_ANGLE,
            ))
        self.nav.pivot.assert_not_called()
        initial_forward = call(
            None,
            self.nav.TURN_90_FORWARD_TIME,
            speed=self.nav.TURN_90_FORWARD_SPEED,
            max_speed=self.nav.TURN_90_FORWARD_MAX_SPEED,
        )
        self.assertEqual(
            self.nav.follow_for.call_args_list,
            [
                initial_forward,
                call(None, self.nav.RETURN_FORWARD_TIME),
            ],
        )
        self.assertLess(events.index("forward"), events.index("turn_to_line"))
        self.assertLess(events.index("turn_to_line"), events.index("arm"))
        self.assertLess(events.index("arm"), events.index("drop"))
        self.assertLess(events.index("orange"), events.index("drop"))
        self.assertLess(events.index("drop"), events.index("push"))
        self.assertLess(events.index("push"), events.index("fold"))
        self.assertLess(events.index("fold"), events.index("reverse"))
        self.assertLess(events.index("fold"), events.index("forward", 1))
        self.nav.cube1_fold_arm.assert_called_once_with(
            unittest.mock.ANY, reason="junction"
        )
        self.nav.button_return.assert_called_once_with(
            None,
            unittest.mock.ANY,
            unittest.mock.ANY,
            12.0,
            -1,
            label="CUBE1",
            accept_s8_only=True,
            line_loss_timeout=None,
        )
        rgb.orange.assert_called_once_with()
        rgb.off.assert_called()
        rgb.close.assert_called_once_with()

    def test_task_stops_when_sensor_guided_turn_fails(self):
        self.nav.cube1_arm_action = Mock()
        self.nav.cube1_turn_to_center_line = Mock(
            return_value=(False, None)
        )
        self.nav.cube1_carry_until_drop = Mock()
        self.nav.follow_for = Mock(return_value=True)

        with patch.object(self.nav.time, "sleep"):
            self.assertFalse(self.nav.cube1_task(
                None, Mock(), Mock(), 12.0, Mock(), Mock()
            ))
        self.nav.cube1_arm_action.assert_not_called()
        self.nav.cube1_carry_until_drop.assert_not_called()


if __name__ == "__main__":
    unittest.main()

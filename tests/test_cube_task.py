"""Offline checks for the complete CUBE1 state machine."""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, call, patch


ROOT = Path(__file__).resolve().parents[1]


def load_main_test():
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
            "cube1_main_test", ROOT / "main_test.py"
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class Cube1TaskTests(unittest.TestCase):
    def setUp(self):
        self.nav = load_main_test()
        self.nav.stop = Mock()

    def test_arm_uses_requested_xyz_pitch_and_open_gripper(self):
        move = Mock(return_value=True)
        with patch.dict(sys.modules, {
            "button_arm": types.SimpleNamespace(move_to_xyz_and_hold=move),
        }):
            self.assertTrue(self.nav.cube1_arm_action(Mock()))
        move.assert_called_once_with(
            unittest.mock.ANY,
            360.0,
            0.0,
            -150.0,
            tool_pitch=-20.0,
            gripper=0.0,
        )

    def test_arm_folds_home_before_reverse(self):
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

    def test_crosses_junction_before_accepting_line_loss(self):
        centered = {"position": 0.0}
        no_line = {"position": None}
        # Line loss before the junction is ignored; loss after it is the drop.
        self.nav.line_tracker.update = Mock(side_effect=[
            no_line, centered, centered, centered, centered, no_line,
        ])
        detector = Mock()
        detector.read.side_effect = [
            (False, False, False),
            (False, False, False),
            (False, False, False),
            (True, True, True),
            (True, True, True),
            (False, False, False),
        ]
        self.nav.follow_for = Mock(return_value=True)
        self.nav.differential_drive = Mock()
        with patch.object(self.nav.time, "sleep"):
            self.assertTrue(self.nav.cube1_cross_junction_until_drop(
                None, detector
            ))
        self.nav.follow_for.assert_called_once_with(
            None,
            self.nav.CUBE1_JUNCTION_CROSS_SECONDS,
            speed=self.nav.MANEUVER_SPEED,
            max_speed=self.nav.MANEUVER_MAX_SPEED,
        )

    def test_line_loss_after_second_junction_is_virtual_cube_drop(self):
        centered = {"position": 0.0}
        no_line = {"position": None}
        self.nav.line_tracker.update = Mock(side_effect=[
            centered, centered, centered, centered, centered, no_line,
        ])
        detector = Mock()
        detector.read.side_effect = [
            (False, False, False),
            (False, False, False),
            (False, False, False),
            (True, True, True),
            (True, True, True),
        ]
        self.nav.follow_for = Mock(return_value=True)
        self.nav.differential_drive = Mock()
        with patch.object(self.nav.time, "sleep"):
            self.assertTrue(self.nav.cube1_cross_junction_until_drop(
                None, detector
            ))

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
        state = {"position": 0.4}
        self.nav.line_tracker.update = Mock(return_value=state)
        self.nav.differential_reverse = Mock()
        heading = Mock()
        # A large value used to trip the 30-degree gyro abort.
        heading.read.return_value = 45.0
        self.nav.Heading = Mock(return_value=heading)
        detector = Mock()
        detector.read.side_effect = [
            (False, False, False),
            (False, False, False),
            (False, False, False),
            (True, True, True),
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
                None, detector, Mock(), 0.0, 1
            ))
        for called_state, called_speed, controller in (
            call.args for call in self.nav.differential_reverse.call_args_list
        ):
            self.assertIs(called_state, state)
            self.assertEqual(called_speed, self.nav.RETURN_SPEED)
            self.assertIsInstance(controller, self.nav.ReverseLineController)

    def test_complete_task_uses_r90_then_button_style_final_turn(self):
        events = []
        self.nav.cube1_arm_action = Mock(
            side_effect=lambda *args: events.append("arm") or True
        )
        self.nav.cube1_cross_junction_until_drop = Mock(
            side_effect=lambda *args: events.append("drop") or True
        )
        self.nav.cube1_fold_arm = Mock(
            side_effect=lambda *args: events.append("fold") or True
        )
        self.nav.reacquire_line = Mock(
            side_effect=lambda *args: events.append("reacquire") or True
        )
        self.nav.button_return = Mock(
            side_effect=lambda *args, **kwargs:
            events.append("reverse") or True
        )
        self.nav.follow_for = Mock(return_value=True)
        self.nav.pivot_90 = Mock(
            side_effect=lambda *args, **kwargs:
            events.append("r90") or (True, -1)
        )
        self.nav.pivot = Mock(return_value=(True, None))

        with patch.object(self.nav.time, "sleep"):
            self.assertTrue(self.nav.cube1_task(
                None, Mock(), Mock(), 12.0, Mock()
            ))

        self.nav.pivot_90.assert_called_once_with(
            unittest.mock.ANY,
            12.0,
            "R",
            angle_target=self.nav.TURN_90_ANGLE,
        )
        self.nav.pivot.assert_called_once_with(
            unittest.mock.ANY,
            12.0,
            "R",
            angle_target=self.nav.BUTTON_FINAL_RIGHT_ANGLE,
        )
        self.assertEqual(self.nav.follow_for.call_args_list[0], call(
            None,
            self.nav.TURN_90_FORWARD_TIME,
            speed=self.nav.TURN_90_FORWARD_SPEED,
            max_speed=self.nav.TURN_90_FORWARD_MAX_SPEED,
        ))
        self.assertLess(events.index("r90"), events.index("reacquire"))
        self.assertLess(events.index("reacquire"), events.index("arm"))
        self.nav.button_return.assert_called_once()
        self.assertLess(events.index("drop"), events.index("fold"))
        self.assertLess(events.index("fold"), events.index("reverse"))
        self.assertEqual(
            self.nav.button_return.call_args.kwargs["label"], "CUBE1"
        )
        self.assertTrue(self.nav.button_return.call_args.kwargs["accept_s8_only"])


if __name__ == "__main__":
    unittest.main()

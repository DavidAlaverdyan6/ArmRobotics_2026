"""CUBE1 end-of-track completion in main_test.py (offline hardware mocks)."""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch


def load_navigation():
    stubs = {
        name: types.ModuleType(name)
        for name in (
            "smbus", "telemetrix", "motors", "line_tracker",
            "sensors.vl53l0x_sensor", "stm32_reset",
        )
    }
    stubs["telemetrix"].telemetrix = Mock()
    stubs["line_tracker"].LineTrackingEngine = Mock(return_value=Mock())
    stubs["line_tracker"].CENTER_POSITION = 0.0
    stubs["sensors.vl53l0x_sensor"].VL53L0XSensor = Mock()
    stubs["stm32_reset"].STM32_RESET_GPIO = 23
    stubs["stm32_reset"].reset_stm32 = Mock()
    with patch.dict(sys.modules, stubs):
        spec = importlib.util.spec_from_file_location(
            "cube_line_loss_test_main",
            Path(__file__).resolve().parents[1] / "main_test.py",
        )
        navigation = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(navigation)
    return navigation


class CubeLineLossTests(unittest.TestCase):
    def setUp(self):
        self.nav = load_navigation()
        self.nav.stop = Mock()
        self.nav.differential_drive = Mock()
        self.nav.line_tracker.all_channels_fresh = Mock(return_value=True)
        self.nav.line_follower.drive_straight = Mock()
        self.detector = Mock()
        self.sensor = Mock()
        self.sensor.detected.return_value = True
        self.indicator = Mock()

    def feed(self, states, junction_flags):
        self.nav.line_tracker.update = Mock(side_effect=states)
        self.detector.read = Mock(side_effect=junction_flags)

    def test_line_loss_after_second_crossing_counts_as_drop(self):
        clear = {"position": 0.0, "normalized": [0, 0, 0, 1, 1, 0, 0, 0]}
        cross = {"position": 0.0, "normalized": [1] * 8}
        lost = {"position": None, "normalized": [0] * 8}
        frames = [clear] * 3 + [cross] * 2 + [lost] * 5
        flags = [(False, False, False)] * 3 + [(True, True, True)] * 2
        flags += [(False, False, False)] * 5
        self.feed(frames, flags)
        with patch.object(self.nav.time, "sleep"):
            result = self.nav.cube1_transport(
                None, self.detector, self.sensor, self.indicator,
            )
        self.assertEqual(result, "drop_line_lost")
        self.assertEqual(self.nav.line_tracker.update.call_count, len(frames))
        self.nav.line_follower.drive_straight.assert_not_called()
        self.nav.stop.assert_called()

    def test_line_loss_before_second_crossing_is_not_success(self):
        lost = {"position": None, "normalized": [0] * 8}
        self.feed([lost], [(False, False, False)])
        with patch.object(self.nav.time, "sleep"):
            result = self.nav.cube1_transport(
                None, self.detector, self.sensor, self.indicator,
            )
        self.assertIsNone(result)

    def test_successful_drop_with_line_still_visible_is_unchanged(self):
        clear = {"position": 0.0, "normalized": [0, 0, 0, 1, 1, 0, 0, 0]}
        cross = {"position": 0.0, "normalized": [1] * 8}
        frames = [clear] * 3 + [cross] * 2 + [clear] * 3
        flags = [(False, False, False)] * 3 + [(True, True, True)] * 2
        flags += [(False, False, False)] * 3
        self.feed(frames, flags)
        self.sensor.detected.side_effect = [True] * 5 + [False] * 3
        with patch.object(self.nav.time, "sleep"):
            result = self.nav.cube1_transport(
                None, self.detector, self.sensor, self.indicator,
            )
        self.assertEqual(result, "drop")

    def test_completed_line_loss_reverses_then_finishes_existing_r90(self):
        nav = self.nav
        order = []
        nav.follow_for = Mock(return_value=True)
        nav.pivot_90 = Mock(side_effect=lambda *args, **kwargs: order.append("pivot") or (True, 1))
        nav.reacquire_line = Mock(return_value=True)
        nav.cube1_arm_action = Mock(return_value=True)
        nav.cube1_transport = Mock(return_value="drop_line_lost")
        nav.cube1_fold_arm = Mock(side_effect=lambda *args, **kwargs: order.append("fold") or True)
        nav.cube1_emergency_reverse_to_line = Mock(
            side_effect=lambda *args: order.append("find_line") or True
        )
        nav.button_return = Mock(
            side_effect=lambda *args, **kwargs: order.append("return") or True
        )
        nav.cube1_create_indicator = Mock(return_value=Mock())
        with patch.dict(sys.modules, {
            "firefighter": types.SimpleNamespace(
                Firefighter=Mock(return_value=Mock())
            )
        }):
            self.assertTrue(nav.cube1_task(None, Mock(), Mock(), 0, Mock()))
        self.assertEqual(
            order, ["pivot", "fold", "find_line", "return", "pivot"],
        )
        self.assertEqual(nav.pivot_90.call_count, 2)
        nav.button_return.assert_called_once()
        self.assertTrue(nav.button_return.call_args.kwargs["accept_s8_only"])


if __name__ == "__main__":
    unittest.main()

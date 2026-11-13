"""CUBE1 offline integration tests; no hardware is connected."""
import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]


def load_main():
    stubs = {k: types.ModuleType(k) for k in (
        "smbus", "telemetrix", "motors", "line_tracker", "sensors.vl53l0x_sensor"
    )}
    stubs["telemetrix"].telemetrix = Mock()
    stubs["line_tracker"].LineTrackingEngine = Mock(return_value=Mock())
    stubs["line_tracker"].CENTER_POSITION = 0.0
    stubs["sensors.vl53l0x_sensor"].VL53L0XSensor = Mock()
    with patch.dict(sys.modules, stubs):
        spec = importlib.util.spec_from_file_location("cube1_main", ROOT/"main.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class CubeTests(unittest.TestCase):
    def setUp(self):
        self.nav = load_main()
        self.nav.stop = Mock()

    def test_first_r90_then_arm_without_recenter(self):
        n = self.nav
        events = []
        n.align_to_crossbar_center = Mock(side_effect=lambda *_: events.append("align") or True)
        n.pivot_90 = Mock(side_effect=lambda *args, **kw: events.append("R90") or (True,-1))
        n.cube1_arm_action = Mock(side_effect=lambda *_: events.append("arm") or True)
        n.reacquire_line = Mock(side_effect=lambda *args, **kw: events.append("reacquire") or True)
        n.cube1_transport = Mock(side_effect=lambda *args: events.append("transport") or "drop")
        n.cube1_fold_arm = Mock(side_effect=lambda *args, **kw: events.append("fold") or True)
        n.button_return = Mock(side_effect=lambda *args, **kw: events.append("reverse") or True)
        n.cube1_create_indicator = Mock(return_value=Mock())
        sensor = Mock()
        with patch.dict(sys.modules, {"firefighter":types.SimpleNamespace(Firefighter=Mock(return_value=sensor))}):
            self.assertTrue(n.cube1_task(None, Mock(), Mock(), 12.0, Mock(), Mock()))
        self.assertEqual(events, [
            "R90", "arm", "transport", "fold",
            "reverse", "align", "R90", "reacquire",
        ])
        self.assertEqual(n.align_to_crossbar_center.call_count, 1,
                         "Only final R90 may align; first turn must be immediate")

    def test_stop_sends_real_zero_pwm(self):
        n = self.nav
        n.motors.motor_stop.reset_mock()
        n.stop()
        n.motors.motor_stop.assert_called_once_with()

    def test_route_is_not_a_broken_comment_example(self):
        self.assertEqual(self.nav.load_route(), [
            "CUBE1", "R", "L", "R", "CHARGING", "COLOR", "L", "Q"
        ])

    def test_two_arm_targets(self):
        n = self.nav
        arm = Mock()
        web = types.SimpleNamespace(
            arm=None, STEP_DELAY=0.01,
            plan_auto_path=Mock(return_value=([{"base":90}],10.0)),
            plan_target_pitch_path=Mock(return_value=[{"base":90}]),
        )
        execute = Mock()
        with patch.dict(sys.modules, {
            "roboarm":types.SimpleNamespace(get_shared_arm=Mock(return_value=(arm,True))),
            "button_arm":types.SimpleNamespace(_execute_plan=execute),
            "arm_web":web,
        }):
            self.assertTrue(n.cube1_arm_action(Mock()))
        first=web.plan_auto_path.call_args.args[0]
        second=web.plan_target_pitch_path.call_args.args[0]
        self.assertEqual((first["x"],first["y"],first["z"]),(250,0,100))
        self.assertEqual((second["x"],second["y"],second["z"],second["tool_pitch"]),
                         (200,0,-180,-90))
        self.assertEqual(execute.call_count,2)


if __name__=="__main__":
    unittest.main()

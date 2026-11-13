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
        n.pivot_90 = Mock(side_effect=lambda *args, **kw: events.append("R90") or (True,-1))
        n.turn_90 = Mock(side_effect=lambda *args, **kw: events.append("R90") or (True,-1))
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
            "reverse", "R90", "reacquire",
        ])
        n.pivot_90.assert_not_called()
        self.assertEqual(n.turn_90.call_count, 2)
        self.assertEqual(n.turn_90.call_args_list[0].args, (None, None, 12.0, "R"))
        self.assertEqual(n.turn_90.call_args_list[1].args, (None, None, 12.0, "R"))

    def test_button_uses_shared_l90_and_r90_and_3s_line_follow(self):
        n = self.nav
        n.turn_90 = Mock(return_value=(True, -1))
        n.reacquire_line = Mock(return_value=True)
        n.follow_for = Mock(return_value=True)
        n.button_bumper_approach = Mock(side_effect=AssertionError("old bumper mode invoked"))
        n.button_return = Mock(return_value=True)
        self.assertTrue(n.button_task("cal", Mock(), Mock(), "bus", 0.1, Mock()))
        self.assertEqual([call.args[-1] for call in n.turn_90.call_args_list], ["L", "R"])
        n.follow_for.assert_called_once_with("cal", 3.0)
        n.button_bumper_approach.assert_not_called()

    def test_turn_pwm_is_constant_and_ignores_line_sensors(self):
        n = self.nav
        self.assertEqual(n.turn_90_pwm(0, False), n.TURN_PWM_FAST)
        self.assertEqual(n.turn_90_pwm(80, True), n.TURN_PWM_FAST)
        self.assertEqual(n.turn_90_pwm(80, True, slow=True), n.SLOW_90_PWM_FAST)
        self.assertEqual(n.turn_90_pwm(0, False, direction="R"), n.TURN_PWM_FAST)
        self.assertEqual(n.turn_90_pwm(0, False, direction="L"), n.TURN_PWM_FAST)
        self.assertEqual(n.TURN_R90_ANGLE, 86.0)
        self.assertEqual(n.TURN_90_ANGLE, 86.0)

    def test_gyro_turn_r_uses_86_and_faster_pwm(self):
        n = self.nav
        clock = [0.0]
        def tick():
            clock[0] += 0.01
            return clock[0]
        n.gyro_rate = Mock(return_value=250.0)
        n.time.monotonic = tick
        n.time.sleep = Mock()
        n.line_tracker.all_channels_fresh = Mock(side_effect=AssertionError("ADC queried"))
        result, _ = n.pivot_90(Mock(), 0, "R")
        self.assertTrue(result)
        n.line_tracker.all_channels_fresh.assert_not_called()
        self.assertTrue(any(call.args == (n.TURN_PWM_FAST, -n.TURN_PWM_FAST)
                            for call in n.motors.set_speeds.call_args_list))

    def test_gyro_turn_does_not_read_adc(self):
        n = self.nav
        # A short deterministic fake gyro trace: the only input to pivot_90.
        clock = [0.0]
        def tick():
            clock[0] += 0.01
            return clock[0]
        n.gyro_rate = Mock(return_value=250.0)
        n.time.monotonic = tick
        n.time.sleep = Mock()
        n.line_tracker.all_channels_fresh = Mock(side_effect=AssertionError("ADC queried"))
        n.line_tracker.update = Mock(side_effect=AssertionError("ADC queried"))
        result, _ = n.pivot_90(Mock(), 0, "L")
        self.assertTrue(result)
        n.line_tracker.all_channels_fresh.assert_not_called()
        n.line_tracker.update.assert_not_called()

    def test_shared_turn_uses_only_saved_pulse_and_gyro(self):
        n = self.nav
        n.load_junction_drive = Mock(return_value=(0.5, 80))
        n.line_tracker.all_channels_fresh = Mock(return_value=True)
        n.time.sleep = Mock()
        n.stop = Mock()
        n.motors.set_speeds = Mock()
        n.pivot_90 = Mock(return_value=(True, 1))
        self.assertEqual(n.turn_90("cal", "bus", 0.2, "R"), (True, 1))
        self.assertEqual(n.turn_90("cal", "bus", 0.2, "L"), (True, 1))
        self.assertEqual(n.motors.set_speeds.call_args_list,
                         [unittest.mock.call(80, 80)] * 2)
        self.assertEqual(n.time.sleep.call_args_list,
                         [unittest.mock.call(0.5)] * 2)
        self.assertEqual([c.args[2] for c in n.pivot_90.call_args_list], ["R", "L"])

    def test_invalid_pulse_calibration_prevents_pivot(self):
        n = self.nav
        n.load_junction_drive = Mock(side_effect=ValueError("missing"))
        n.pivot_90 = Mock()
        self.assertEqual(n.turn_90("cal", "bus", 0.2, "R"), (False, None))
        n.pivot_90.assert_not_called()

    def test_reacquire_never_stops_a_wheel(self):
        n = self.nav
        for position in (-3.5, -1.0, 0.0, 1.0, 3.5):
            left, right = n.reacquire_pwm(position)
            self.assertGreaterEqual(left, n.REACQUIRE_MIN_PWM)
            self.assertGreaterEqual(right, n.REACQUIRE_MIN_PWM)

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

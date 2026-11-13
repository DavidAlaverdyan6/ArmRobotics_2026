"""Offline checks for sensor-guided turns and calibrated axle travel."""
import unittest
import sys
import types
from unittest.mock import Mock, patch

from tests.test_button_positioning import load_navigation
from line_tracker import LineTrackingEngine
from measure_55mm import estimate


class RevisionTests(unittest.TestCase):
    def test_cube_sensor_loss_uses_fast_blink_and_pit_after_junction(self):
        self._cube_transport(fault=True)

    def test_cube_normal_transport_lights_red_then_green_and_returns(self):
        self._cube_transport(fault=False)

    def test_cube_does_not_move_arm_after_hardware_turn_failure(self):
        nav = load_navigation()
        nav.route_90 = Mock(return_value=(False, None))
        arm = Mock()
        with patch.dict(sys.modules, {
            'firefighter': types.SimpleNamespace(Firefighter=Mock()),
            'roboarm': types.SimpleNamespace(get_shared_arm=Mock(return_value=(arm, False))),
        }):
            self.assertFalse(nav.cube1_task_v2({}, Mock(), Mock(), 0, Mock()))
        arm.write.assert_not_called()

    def _cube_transport(self, fault):
        nav = load_navigation()
        nav.route_90 = Mock(return_value=(True, -1))
        nav.cube1_fold_arm = Mock(return_value=True)
        nav.button_return = Mock(return_value=True)
        nav.stop = Mock()
        nav.differential_drive = Mock(return_value=True)
        nav.FreshFrames = Mock(return_value=Mock(accept=Mock(return_value=True)))
        plain = {'position': 0, 'normalized': [0, 0, 0, 1, 1, 0, 0, 0]}
        cross = {'position': 0, 'normalized': [1] * 8}
        states = [plain] * 3 + [cross] * 5 + [plain] + [cross] * 3
        nav.line_tracker.update = Mock(side_effect=states)
        nav.line_tracker.all_channels_fresh = Mock(return_value=True)
        sensor = Mock()
        sensor.detected.side_effect = ([True, False] + [False] * 20 if fault
                                      else [True] * (len(states) - 1) + [False])
        rgb = Mock()
        nav.cube1_create_indicator = Mock(return_value=rgb)
        detector = Mock()
        detector.read.side_effect = lambda state: (True, True, state is cross)
        with patch.dict(sys.modules, {
            'firefighter': types.SimpleNamespace(Firefighter=Mock(return_value=sensor)),
            'roboarm': types.SimpleNamespace(get_shared_arm=Mock(return_value=(Mock(), False))),
        }), patch.object(nav.time, 'sleep'):
            self.assertTrue(nav.cube1_task_v2({}, detector, Mock(), 0, Mock()))
        self.assertEqual(nav.line_tracker.update.call_count, len(states))
        if fault:
            rgb.green.assert_not_called()
        else:
            rgb.green.assert_called_once()
        rgb.red.assert_called()
        self.assertIsNone(nav.button_return.call_args.kwargs['max_return_time'])
        self.assertTrue(nav.route_90.call_args.kwargs['at_edge'])

    def test_measured_speed_includes_start_stop_offset(self):
        speed, offset, duration = estimate(0.4, 35, 0.8, 75)
        self.assertAlmostEqual(speed, 100)
        self.assertAlmostEqual(offset, -5)
        self.assertAlmostEqual(duration, 0.6)

    def test_integral_accumulates_and_resets_on_stop(self):
        driver = Mock()
        engine = LineTrackingEngine(kp=0, kd=0, ki=10, motor_driver=driver)
        with patch('line_tracker.time.monotonic', side_effect=[1, 1.1]):
            engine.drive({'position': 0.5})
            engine.drive({'position': 0.5})
        self.assertAlmostEqual(engine.integral, 0.05)
        engine.stop()
        self.assertEqual(engine.integral, 0)

    def test_button_uses_common_turns_and_three_second_run(self):
        nav = load_navigation()
        nav.route_90 = Mock(side_effect=[(True, 1), (True, -1)])
        nav.follow_for = Mock(return_value=True)
        nav.button_return = Mock(return_value=True)
        bus = Mock()
        self.assertTrue(nav.button_task({}, Mock(), Mock(), bus, 0, Mock()))
        self.assertEqual([c.args[3] for c in nav.route_90.call_args_list], ['L', 'R'])
        nav.follow_for.assert_called_once_with({}, 3.0)
        self.assertIsNone(nav.button_return.call_args.kwargs['max_return_time'])

    def test_turn_uses_only_directional_outer_sensor_and_falls_back(self):
        for direction, correct in [('L', 0), ('R', 7)]:
            for side in (correct, 7 - correct):
                nav = load_navigation()
                clock = [1.0]
                nav.stop = Mock()
                nav.gyro_rate = Mock(return_value=120.0)
                nav.motors.set_speeds = Mock()
                nav.pivot = Mock(return_value=(True, 1))
                nav.line_tracker.all_channels_fresh = Mock(return_value=True)
                nav.FreshFrames = Mock(return_value=Mock(accept=Mock(return_value=True)))
                values = [0.0] * 8
                values[side] = 1.0
                values[2:6] = [1.0] * 4
                nav.line_tracker.update = Mock(return_value={'normalized': values})
                with patch.object(nav.time, 'monotonic', side_effect=lambda: clock[0]), \
                     patch.object(nav.time, 'sleep', side_effect=lambda dt: clock.__setitem__(0, clock[0] + dt)):
                    ok, _ = nav.pivot_90(Mock(), 0, direction, calibration={})
                self.assertTrue(ok)
                if side == correct:
                    nav.pivot.assert_not_called()
                else:
                    self.assertEqual(nav.pivot.call_count, 2)
                    self.assertEqual(nav.pivot.call_args.args[2], direction)


if __name__ == '__main__':
    unittest.main()

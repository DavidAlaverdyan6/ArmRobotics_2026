"""Offline tests; no GPIO, serial connection, or physical motion."""
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import firefighter as hardware


def load_navigation():
    # Execute the navigation module with only its hardware imports stubbed.
    stubs = {name: types.ModuleType(name) for name in
             ('smbus', 'telemetrix', 'motors', 'line_tracker',
              'sensors.vl53l0x_sensor')}
    stubs['telemetrix'].telemetrix = Mock()
    stubs['sensors.vl53l0x_sensor'].VL53L0XSensor = Mock()
    with patch.dict(sys.modules, stubs):
        import importlib.util
        spec = importlib.util.spec_from_file_location('navigation_test',
            Path(__file__).resolve().parents[1] / 'main_ggs_interactive.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class FireTests(unittest.TestCase):
    def test_samples_and_staleness(self):
        fire = hardware.Firefighter(Mock())
        with self.assertRaises(RuntimeError):
            fire.detected()
        with patch.object(hardware.time, 'monotonic', return_value=10):
            fire._callback([3, hardware.pins.fire_sensor, 3500])
            self.assertFalse(fire.detected())
            fire._callback([3, hardware.pins.fire_sensor, 0])
            self.assertTrue(fire.detected())
            with patch.object(hardware, 'FIRE_ACTIVE_LOW', False):
                self.assertFalse(fire.detected())
        with patch.object(hardware.time, 'monotonic', return_value=11):
            with self.assertRaises(RuntimeError):
                fire.detected()

    def test_water_duration_and_interrupt(self):
        for error in (None, KeyboardInterrupt()):
            board = Mock()
            fire = hardware.Firefighter(board)
            with patch.object(hardware.time, 'sleep', side_effect=error) as sleep:
                if error is None:
                    fire.spray()
                else:
                    with self.assertRaises(KeyboardInterrupt):
                        fire.spray()
                sleep.assert_called_once_with(5.0)
            self.assertEqual([c.args for c in board.digital_write.call_args_list],
                             [(hardware.pins.pump, 1), (hardware.pins.pump, 0)])

    def test_task_order_and_failure(self):
        for line_ok in (True, False):
            nav = load_navigation()
            events = []
            fire = Mock()
            fire.detected.side_effect = [False, False, True]
            fire.spray.side_effect = lambda: events.append('water')
            arm = Mock()
            home = {'base': 95, 'shoulder': 180, 'elbow': 180,
                    'wrist': 40, 'gripper': 180}
            nav.stop = lambda: events.append('stop')
            nav.pivot = lambda bus, offset, direction, **kw: (
                events.append(direction) or True, 1)
            nav.differential_drive = Mock(return_value=line_ok)
            nav.line_tracker.update = Mock(return_value={'position': 0})
            nav.button_return = lambda *a: events.append('reverse') or True
            nav.follow_for = lambda *a: events.append('forward') or True
            nav.reacquire_line = lambda *a: events.append('reacquire') or True
            with patch.dict(sys.modules, {
                'roboarm': types.SimpleNamespace(RoboArm=Mock(return_value=arm)),
                'arm_web': types.SimpleNamespace(START_ANGLES=home),
            }), patch.object(hardware, 'Firefighter', return_value=fire), \
                 patch.object(nav.time, 'sleep'):
                self.assertEqual(nav.firefighter_task(None, None, None, 0, Mock()), line_ok)
            arm.write.assert_called_once_with(**{**home, 'wrist': 0})
            fire.off.assert_called_once()
            if line_ok:
                self.assertEqual(events, ['stop', 'L', 'stop', 'water', 'reverse',
                                          'forward', 'R', 'reacquire', 'stop'])
            else:
                fire.spray.assert_not_called()
                self.assertNotIn('reverse', events)

    def test_route_commands(self):
        nav = load_navigation()
        nav.ROUTE = ['F', 'C', 'P', 'Q']
        self.assertEqual(nav.load_route(), nav.ROUTE)


if __name__ == '__main__':
    unittest.main()

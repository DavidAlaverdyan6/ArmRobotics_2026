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
        self.assertIsNone(fire.detected())
        with patch.object(hardware.time, 'monotonic', return_value=10):
            for value, expected in ((0, False), (1999, False),
                                    (2000, True), (2500, True), (4095, True)):
                fire._callback([3, hardware.pins.fire_sensor, value])
                self.assertEqual(fire.detected(), expected, value)
            with patch.object(hardware, 'FIRE_ACTIVE_LOW', True):
                self.assertFalse(fire.detected())
        with patch.object(hardware.time, 'monotonic', return_value=11):
            self.assertIsNone(fire.detected())

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
        for line_ok, still_burning, arm_reused in (
            (True, False, False),
            (True, True, True),
            (False, False, True),
        ):
            nav = load_navigation()
            events = []
            fire = Mock()
            fire.detected.side_effect = [False, True]
            fire.confirm_flame.side_effect = [True, still_burning]
            fire.spray.side_effect = lambda: events.append('water')
            fire.wait_for_sample.side_effect = lambda *args, **kwargs: events.append('sample')
            arm = Mock()
            home = {'base': 90, 'shoulder': 180, 'elbow': 180,
                    'wrist': 40, 'gripper': 180}
            nav.stop = lambda: events.append('stop')
            nav.pivot = lambda bus, offset, direction, **kw: (
                events.append(direction) or True, 1)
            nav.differential_drive = lambda *a, **kw: events.append('drive') or line_ok
            nav.line_tracker.update = Mock(return_value={'position': 0})
            nav.button_return = lambda *a: events.append('reverse') or True
            nav.follow_for = lambda *a: events.append('forward') or True
            nav.reacquire_line = lambda *a: events.append('reacquire') or True
            move = Mock(side_effect=lambda arm, pose, *a: events.append(
                'home' if pose == home else 'pose'))
            with patch.dict(sys.modules, {
                'roboarm': types.SimpleNamespace(
                    get_shared_arm=Mock(return_value=(arm, not arm_reused))),
                'arm_web': types.SimpleNamespace(START_ANGLES=home,
                    AUTO_MAX_STEP_DEG=2, STEP_DELAY=0.03),
                'button_arm': types.SimpleNamespace(_return_home=move),
            }), patch.object(hardware, 'Firefighter', return_value=fire), \
                 patch.object(nav.time, 'sleep'):
                self.assertEqual(nav.firefighter_task(None, None, None, 0, Mock()), line_ok)
            fire.off.assert_called_once()
            self.assertLess(events.index('L'), events.index('pose'))
            self.assertLess(events.index('pose'), events.index('drive'))
            self.assertIn('sample', events[events.index('pose') + 1:events.index('drive')])
            if arm_reused:
                self.assertLess(events.index('home'), events.index('pose'))
                arm.write.assert_not_called()
            else:
                arm.write.assert_called_once_with(**home)
            if line_ok:
                self.assertEqual(fire.spray.call_count, 2 if still_burning else 1)
                self.assertLess(events.index('home'), events.index('reverse'))
                self.assertLess(events.index('reverse'), events.index('R'))
                self.assertIn('after', fire.wait_for_sample.call_args.kwargs)
            else:
                fire.spray.assert_not_called()
                self.assertNotIn('reverse', events)

    def test_confirmation_and_fresh_post_spray_reading(self):
        fire = hardware.Firefighter(Mock())
        clock = [0.0]
        def sleep(dt):
            clock[0] += dt
            fire.sample = (2500, clock[0])
        with patch.object(hardware.time, 'monotonic', side_effect=lambda: clock[0]), \
             patch.object(hardware.time, 'sleep', side_effect=sleep):
            fire.sample = (2500, 0.0)
            self.assertTrue(fire.confirm_flame())
            self.assertGreaterEqual(clock[0], 1.0)
            now = clock[0]
            fire.wait_for_sample(after=now)
            self.assertGreater(fire.sample[1], now)
            with patch.object(fire, 'detected', side_effect=[True, True, False]):
                self.assertFalse(fire.confirm_flame())
            with patch.object(fire, 'detected', return_value=None):
                self.assertFalse(fire.confirm_flame())
            start = clock[0]
            def delayed_sample(dt):
                clock[0] += dt
                if clock[0] >= start + 3.0:
                    fire.sample = (0, clock[0])
            with patch.object(hardware.time, 'sleep', side_effect=delayed_sample):
                fire.wait_for_sample(after=start)
            self.assertGreaterEqual(clock[0] - start, 3.0)

    def test_firefighting_pose(self):
        self.assertEqual(hardware.firefighting_pose(), {
            'base': 90,
            'shoulder': 90,
            'elbow': 160,
            'wrist': 70,
            'gripper': 180,
        })

    def test_route_commands(self):
        nav = load_navigation()
        nav.ROUTE = ['F', 'C', 'Q']
        self.assertEqual(nav.load_route(), nav.ROUTE)
        nav.ROUTE = ['P', 'Q']
        with self.assertRaises(ValueError):
            nav.load_route()


if __name__ == '__main__':
    unittest.main()

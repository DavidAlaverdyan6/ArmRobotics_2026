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
              'sensors.vl53l0x_sensor', 'led', 'robot_pin_config')}
    stubs['telemetrix'].telemetrix = Mock()
    stubs['line_tracker'].LineTrackingEngine = Mock(return_value=Mock())
    stubs['line_tracker'].CENTER_POSITION = 0.0
    stubs['led'].RGBLED = Mock(return_value=Mock())
    stubs['robot_pin_config'].LED_RED = 17
    stubs['robot_pin_config'].LED_GREEN = 27
    stubs['robot_pin_config'].LED_BLUE = 22
    stubs['sensors.vl53l0x_sensor'].VL53L0XSensor = Mock()
    with patch.dict(sys.modules, stubs):
        import importlib.util
        spec = importlib.util.spec_from_file_location('navigation_test',
            Path(__file__).resolve().parents[1] / 'main_test.py')
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
            camera_stream = Mock()
            arm = Mock()
            home = {'base': 90, 'shoulder': 180, 'elbow': 180,
                    'wrist': 40, 'gripper': 180}
            nav.stop = lambda: events.append('stop')
            nav.pivot = lambda bus, offset, direction, **kw: (
                events.append(direction) or True, 1)
            nav.pivot_90 = lambda bus, offset, direction, **kw: (
                events.append(direction + '90') or True, None)
            nav.differential_drive = lambda *a, **kw: events.append('drive') or line_ok
            nav.line_tracker.update = Mock(return_value={'position': 0})
            nav.button_return = lambda *a, **kw: events.append('reverse') or True
            nav.follow_for = lambda *a, **kw: events.append(
                ('forward', a[1], kw)
            ) or True
            nav.reacquire_line = lambda *a: events.append('reacquire') or True
            arm.open_gripper.side_effect = lambda: events.append('open')
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
                self.assertTrue(
                    nav.firefighter_task(
                        None, None, None, 0, Mock(), camera_stream
                    )
                )
            camera_stream.save_latest_task_frame.assert_called_once_with(
                'FIREFIGHTER: flame confirmed'
            )
            fire.off.assert_called_once()
            self.assertLess(events.index('L'), events.index('pose'))
            self.assertLess(events.index('open'), events.index('pose'))
            self.assertLess(events.index('pose'), events.index('drive'))
            self.assertIn('sample', events[events.index('pose') + 1:events.index('drive')])
            if arm_reused:
                self.assertLess(events.index('home'), events.index('pose'))
                arm.write.assert_not_called()
            else:
                arm.write.assert_called_once_with(**home)
            self.assertEqual(fire.spray.call_count, 2 if still_burning else 1)
            final_advance = (
                'forward', nav.FIRE_FINAL_ADVANCE_SECONDS, {
                    'speed': nav.FIRE_APPROACH_SPEED,
                    'max_speed': nav.FIRE_APPROACH_MAX_SPEED,
                }
            )
            self.assertIn(final_advance, events)
            self.assertLess(events.index(final_advance), events.index('water'))
            self.assertLess(events.index('home'), events.index('reverse'))
            self.assertLess(events.index('reverse'), events.index('R90'))
            self.assertIn('after', fire.wait_for_sample.call_args.kwargs)

    def test_line_end_sprays_once_then_reverses_and_returns(self):
        nav = load_navigation()
        events = []
        fire = Mock()
        fire.detected.return_value = False
        fire.wait_for_sample.side_effect = lambda *a, **kw: None
        fire.spray.side_effect = lambda: events.append('water')
        arm = Mock()
        home = {'base': 90, 'shoulder': 180, 'elbow': 180,
                'wrist': 40, 'gripper': 180}
        missing = {'position': None}
        gray = {
            'position': 0.2,
            'normalized': [0.22, 0.18, 0.20, 0.27, 0.24, 0.21, 0.19, 0.23],
        }
        found = {
            'position': 0.2,
            'normalized': [0.05, 0.02, 0.08, 0.88, 0.91, 0.12, 0.03, 0.04],
        }
        nav.line_tracker.update = Mock(side_effect=[
            missing, gray, found, found,
        ])
        nav.stop = Mock(side_effect=lambda: events.append('stop'))
        nav.line_follower.reverse_straight = Mock(
            side_effect=lambda speed: events.append(('recover', speed))
        )
        nav.pivot = Mock(return_value=(True, 1))
        nav.pivot_90 = Mock(return_value=(True, None))
        nav.button_return = Mock(
            side_effect=lambda *a, **kw: events.append('return') or True
        )
        nav.follow_for = Mock(return_value=True)
        nav.reacquire_line = Mock(return_value=True)
        camera_stream = Mock()
        move = Mock()

        with patch.dict(sys.modules, {
            'roboarm': types.SimpleNamespace(
                get_shared_arm=Mock(return_value=(arm, False))),
            'arm_web': types.SimpleNamespace(
                START_ANGLES=home, AUTO_MAX_STEP_DEG=2, STEP_DELAY=0.03),
            'button_arm': types.SimpleNamespace(_return_home=move),
        }), patch.object(hardware, 'Firefighter', return_value=fire), \
             patch.object(nav.time, 'sleep'):
            self.assertTrue(
                nav.firefighter_task(
                    None, Mock(), Mock(), 0, Mock(), camera_stream
                )
            )

        camera_stream.save_latest_task_frame.assert_called_once_with(
            'FIREFIGHTER: fallback spray at line end'
        )

        self.assertEqual(nav.line_follower.reverse_straight.call_count, 2)
        nav.line_follower.reverse_straight.assert_called_with(
            nav.FIRE_LINE_RECOVERY_SPEED
        )
        nav.button_return.assert_called_once()
        return_kwargs = nav.button_return.call_args.kwargs
        self.assertEqual(return_kwargs['label'], 'FIREFIGHTER')
        self.assertTrue(return_kwargs['accept_any_junction'])
        self.assertEqual(
            return_kwargs['max_return_time'], nav.FIRE_RETURN_MAX_TIME
        )
        self.assertEqual(
            return_kwargs['line_loss_timeout'],
            nav.BUTTON_REVERSE_LINE_LOSS_TIMEOUT,
        )
        nav.pivot_90.assert_called_once_with(
            unittest.mock.ANY,
            0,
            'R',
            angle_target=nav.TURN_90_ANGLE,
        )
        fire.spray.assert_called_once_with()
        nav.follow_for.assert_called_once_with(None, nav.RETURN_FORWARD_TIME)
        self.assertLess(events.index('water'), events.index(
            ('recover', nav.FIRE_LINE_RECOVERY_SPEED)
        ))
        self.assertLess(events.index(
            ('recover', nav.FIRE_LINE_RECOVERY_SPEED)
        ), events.index('return'))

    def test_fire_reverse_ignores_gray_and_accepts_black(self):
        nav = load_navigation()
        gray = {
            'position': 0.0,
            'normalized': [0.20, 0.18, 0.22, 0.30, 0.28, 0.21, 0.19, 0.20],
        }
        black = {
            'position': 0.0,
            'normalized': [0.02, 0.01, 0.04, 0.90, 0.92, 0.03, 0.01, 0.02],
        }
        nav.line_tracker.update = Mock(
            side_effect=[gray, gray, black, black]
        )
        nav.line_follower.reverse_straight = Mock()
        nav.stop = Mock()

        self.assertTrue(nav.firefighter_reverse_until_line(None))
        self.assertEqual(
            nav.line_follower.reverse_straight.call_count, 3
        )

    def test_fire_reverse_rejects_dark_gray_using_its_raw_calibration(self):
        nav = load_navigation()
        calibration = {
            'white': [3100] * 8,
            'gray': [3700] * 8,
            'black': [3900] * 8,
        }
        gray = {
            'position': 0.0,
            'raw': [3700] * 8,
            'normalized': [0.75] * 8,
        }
        black = {
            'position': 0.0,
            'raw': [3900] * 8,
            'normalized': [1.0] * 8,
        }
        nav.line_tracker.update = Mock(side_effect=[gray, gray, black, black])
        nav.line_follower.reverse_straight = Mock()
        nav.stop = Mock()

        self.assertTrue(nav.firefighter_reverse_until_line(calibration))
        self.assertEqual(nav.line_follower.reverse_straight.call_count, 3)

    def test_fire_return_accepts_confirmed_one_sided_junction(self):
        nav = load_navigation()
        clock = [0.0]
        nav.line_tracker.update = Mock(return_value={'position': 0.0})
        nav.differential_reverse = Mock(return_value=True)
        detector = Mock()
        detector.read.side_effect = [
            (False, False, False),
            (False, False, False),
            (False, False, False),
            (True, False, False),
            (True, False, False),
        ]

        with patch.object(
            nav.time, 'monotonic', side_effect=lambda: clock[0]
        ), patch.object(
            nav.time, 'sleep',
            side_effect=lambda seconds: clock.__setitem__(
                0, clock[0] + seconds
            ),
        ):
            self.assertTrue(nav.button_return(
                None,
                detector,
                Mock(),
                0.0,
                1,
                label='FIREFIGHTER',
                accept_any_junction=True,
            ))

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
            self.assertGreaterEqual(
                clock[0], hardware.FLAME_CONFIRM_SECONDS
            )
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
        from arm_ik import forward_kinematics

        pose = hardware.firefighting_pose()
        xyz = forward_kinematics(
            pose['base'], pose['shoulder'], pose['elbow'], pose['wrist']
        )
        for actual, expected in zip(xyz, (300.0, 0.0, 50.0)):
            self.assertAlmostEqual(actual, expected, delta=0.1)
        self.assertEqual(pose['gripper'], 0)

    def test_failed_confirmation_scans_minus_20_to_plus_20(self):
        fire = Mock()
        fire.detected.return_value = False
        arm = Mock()
        with patch.object(hardware.time, 'sleep'):
            self.assertFalse(
                hardware.scan_for_confirmed_flame(fire, arm, 90.0)
            )

        commanded = [call.kwargs['base'] for call in arm.write.call_args_list]
        self.assertEqual(commanded[0], 88.0)
        self.assertIn(70.0, commanded)
        self.assertIn(110.0, commanded)
        self.assertEqual(commanded[-1], 90.0)
        fire.confirm_flame.assert_not_called()

    def test_scan_stops_when_flame_is_confirmed(self):
        fire = Mock()
        fire.detected.return_value = True
        fire.confirm_flame.side_effect = [False, True]
        arm = Mock()
        with patch.object(hardware.time, 'sleep'):
            self.assertTrue(
                hardware.scan_for_confirmed_flame(fire, arm, 90.0)
            )

        self.assertEqual(
            [call.kwargs['base'] for call in arm.write.call_args_list],
            [88.0, 86.0],
        )

    def test_route_commands(self):
        nav = load_navigation()
        nav.ROUTE = [
            'Firefighter', 'charging', 'Cube1', 'slow',
            'l90slow', 'r90slow', 'Q',
        ]
        expected = [
            'FIREFIGHTER', 'CHARGING', 'CUBE1', 'SLOW',
            'L90SLOW', 'R90SLOW', 'Q',
        ]
        self.assertEqual(nav.load_route(), expected)
        nav.ROUTE = ['P', 'Q']
        with self.assertRaises(ValueError):
            nav.load_route()

    def test_slow_straight_holds_low_speed_before_ramping(self):
        nav = load_navigation()
        self.assertEqual(
            nav.post_junction_drive_limits(0.0, slow=True),
            (nav.SLOW_STRAIGHT_SPEED, nav.SLOW_STRAIGHT_MAX_SPEED),
        )
        self.assertEqual(
            nav.post_junction_drive_limits(
                nav.SLOW_STRAIGHT_HOLD_SECONDS - 0.01, slow=True
            ),
            (nav.SLOW_STRAIGHT_SPEED, nav.SLOW_STRAIGHT_MAX_SPEED),
        )
        self.assertEqual(
            nav.post_junction_drive_limits(
                nav.SLOW_STRAIGHT_HOLD_SECONDS
                + nav.POST_JUNCTION_ACCEL_SECONDS,
                slow=True,
            ),
            (nav.BASE_SPEED, nav.MAX_SPEED),
        )


if __name__ == '__main__':
    unittest.main()

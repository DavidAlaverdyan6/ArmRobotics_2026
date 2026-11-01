"""Hardware-stubbed startup and motor safety checks."""
import importlib.util
import sys
import threading
import itertools
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import line_tracker
import motors

ROOT = Path(__file__).resolve().parents[1]


def load_main():
    stubs = {name: types.ModuleType(name) for name in
             ('smbus', 'telemetrix', 'motors', 'line_tracker')}
    stubs['telemetrix'].telemetrix = Mock()
    for name in ('motors', 'line_tracker', 'smbus'):
        stubs[name] = Mock()
    with patch.dict(sys.modules, stubs):
        spec = importlib.util.spec_from_file_location('main_safety_test', ROOT/'main.py')
        nav = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(nav)
    return nav


class MainSafetyTests(unittest.TestCase):
    def test_zero_adc_values_are_valid_callbacks(self):
        board = Mock()
        callbacks = {}
        board.set_pin_mode_analog_input.side_effect = (
            lambda pin, callback, differential=0: callbacks.__setitem__(pin, callback))
        line_tracker.initialize(board)
        try:
            for pin in line_tracker.SENSOR_PINS:
                callbacks[pin]([2, pin, 0])
            self.assertTrue(line_tracker.all_channels_fresh())
            self.assertEqual(line_tracker.sensor_values, [0]*8)
            self.assertEqual(line_tracker.sample_ids, [1]*8)
        finally:
            line_tracker.shutdown()

    def test_unarmed_and_stale_motor_commands_rejected(self):
        board = Mock()
        motors.initialize(board)
        motors.enable_guard(lambda: False)
        with self.assertRaises(RuntimeError):
            motors.set_speeds(100, 100)
        motors.arm()
        with self.assertRaises(RuntimeError):
            motors.set_speeds(100, 100)
        self.assertFalse(motors._armed)
        self.assertEqual(board.analog_write.call_args_list[-1].args[1], 0)

    def test_other_thread_cannot_command_drive(self):
        board = Mock()
        motors.initialize(board)
        motors.enable_guard(lambda: True)
        motors.arm()
        errors = []
        thread = threading.Thread(target=lambda: self._other_thread(errors))
        thread.start(); thread.join()
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], RuntimeError)
        motors.disarm()

    @staticmethod
    def _other_thread(errors):
        try:
            motors.set_speeds(100, 100)
        except Exception as exc:
            errors.append(exc)

    def _diagnostic(self, gyro_error=False, line=True):
        nav = load_main()
        nav.load_route = Mock(return_value=['Q'])
        nav.telemetrix.Telemetrix.return_value = Mock()
        nav.line_tracker.load_calibration = Mock(return_value={'white':[0]*8,'black':[1]*8})
        nav.wait_for_adc = Mock(return_value={
            'raw':[0]*8, 'position':0.08 if line else None})
        nav.gyro_setup = Mock(side_effect=RuntimeError('gyro absent') if gyro_error else None)
        nav.gyro_calibrate = Mock(return_value=0)
        nav.smbus.SMBus = Mock(return_value=Mock())
        result = nav.main(['--diagnostic'])
        return nav, result

    def test_diagnostic_never_arms_motors(self):
        nav, result = self._diagnostic()
        self.assertIsNone(result)
        nav.motors.arm.assert_not_called()
        nav.motors.disarm.assert_called_once()

    def test_gyro_init_failure_stops_before_arming(self):
        nav, result = self._diagnostic(gyro_error=True)
        self.assertEqual(result, 1)
        nav.motors.arm.assert_not_called()
        nav.motors.disarm.assert_called_once()

    def test_no_line_stops_before_arming(self):
        nav, result = self._diagnostic(line=False)
        self.assertEqual(result, 1)
        nav.motors.arm.assert_not_called()

    def test_invalid_mpu_identity_is_rejected(self):
        nav = load_main()
        bus = Mock()
        bus.read_byte_data.return_value = 0
        with patch.object(nav.time, 'sleep'):
            with self.assertRaises(RuntimeError):
                nav.gyro_setup(bus)

    def test_line_lock_requires_distinct_central_frames(self):
        nav = load_main()
        clock = [1.0]
        seq = itertools.count(1)
        nav.stop = Mock()
        nav.differential_drive = Mock(return_value=True)
        nav.line_tracker.update.side_effect = lambda _: {
            'position': 0.08,
            'normalized': [0, 0, 0, .9, .9, 0, 0, 0],
            'sample_ids': [next(seq)]*8,
            'sample_times': [clock[0]]*8,
        }
        with patch.object(nav.time, 'monotonic', side_effect=lambda: clock[0]), \
             patch.object(nav.time, 'sleep',
                          side_effect=lambda dt: clock.__setitem__(0, clock[0]+dt)):
            self.assertTrue(nav.reacquire_line(None))
        self.assertEqual(nav.differential_drive.call_count, nav.REACQUIRE_CONFIRM)

    def test_offset_start_centres_with_low_speed(self):
        nav = load_main()
        clock = [1.0]
        seq = itertools.count(1)
        nav.stop = Mock()
        nav.differential_drive = Mock(return_value=True)
        def next_frame(_):
            n = next(seq)
            shifted = n <= 2
            return {
                'position': 1.2 if shifted else 0.08,
                'normalized': [0, 0, 0, .8, .9, .8, 0, 0]
                if shifted else [0, 0, .1, .9, .9, .1, 0, 0],
                'sample_ids': [n]*8,
                'sample_times': [clock[0]]*8,
            }
        nav.line_tracker.update.side_effect = next_frame
        with patch.object(nav.time, 'monotonic', side_effect=lambda: clock[0]), \
             patch.object(nav.time, 'sleep',
                          side_effect=lambda dt: clock.__setitem__(0, clock[0]+dt)):
            nav.initial_centering(None, nav.FreshFrames())
        self.assertGreaterEqual(nav.differential_drive.call_count, 2)
        for call in nav.differential_drive.call_args_list:
            self.assertEqual(call.kwargs['speed'], nav.START_CENTER_SPEED)
        nav.stop.assert_called()

    def test_recovery_without_gyro_fails_stopped(self):
        nav = load_main()
        nav.stop = Mock()
        self.assertFalse(nav.recover_line(None, None, None, 0, 0))
        nav.stop.assert_called_once()

    def test_charging_task_stops_for_five_seconds(self):
        nav = load_main()
        nav.stop = Mock()
        clock = [0.0]
        led = types.SimpleNamespace(RGBLED=Mock(return_value=Mock()))
        pins = types.SimpleNamespace(LED_BLUE=1, LED_GREEN=2, LED_RED=3)
        with patch.dict(sys.modules, {'led': led, 'robot_pin_config': pins}), \
             patch.object(nav.time, 'monotonic', side_effect=lambda: clock[0]), \
             patch.object(nav.time, 'sleep',
                          side_effect=lambda dt: clock.__setitem__(0, clock[0]+dt)):
            self.assertTrue(nav.charging_task())
        self.assertAlmostEqual(clock[0], 5.0)
        nav.stop.assert_called()


if __name__ == '__main__':
    unittest.main()

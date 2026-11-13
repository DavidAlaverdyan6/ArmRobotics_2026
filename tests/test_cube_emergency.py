"""Offline regression tests for CUBE1 pit and LED state transitions.

These tests mock GPIO, Telemetrix and ADC and do not move hardware.
"""
import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

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
        spec = importlib.util.spec_from_file_location("cube1_navigation", ROOT/"main.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class Cube1PitTests(unittest.TestCase):
    def setUp(self):
        self.nav = load_main()
        self.nav.stop = Mock()
        self.nav.differential_drive = Mock(return_value=True)
        self.nav.forward_drive_straight = Mock(return_value=True)
        self.nav.line_tracker.all_channels_fresh = Mock(return_value=True)
        self.signal = Mock()
        self.clock = [0.0]

    def run_sequence(self, sequence):
        """Each entry is (branch_visible, black_pit, cube_present)."""
        index = [0]
        detector = Mock()
        sensor = Mock()
        def update(_calibration):
            i = min(index[0], len(sequence)-1)
            branch, pit, cube = sequence[i]
            self._cube = cube
            return {
                "position": None if pit else 0.0,
                "normalized": [1.0]*8 if pit else [0.,0.,0.9,0.9,0.9,0.9,0.,0.],
                "crossing": "BOTH" if branch else None,
                "_branch": branch,
            }
        self.nav.line_tracker.update = Mock(side_effect=update)
        detector.read.side_effect=lambda state: (
            bool(state["_branch"]), bool(state["_branch"]), bool(state["_branch"])
        )
        sensor.detected.side_effect=lambda: self._cube
        def sleep(seconds):
            self.clock[0] += seconds
            index[0] += 1
            if index[0] > len(sequence)+25:
                raise RuntimeError("State machine never exited")
        with patch.object(self.nav.time,"monotonic",side_effect=lambda:self.clock[0]),\
             patch.object(self.nav.time,"sleep",side_effect=sleep):
            return self.nav.cube1_transport(None,detector,sensor,self.signal)

    def test_emergency_only_detects_pit_after_second_crossing_exits(self):
        clear=(False,False,False)
        crossing=(True,False,False)
        pit=(False,True,False)
        frames=([clear]*4+[crossing]*3+[pit]*2+[clear]*4+[pit]*5)
        self.assertEqual(self.run_sequence(frames),"emergency")
        self.signal.set_mode.assert_any_call("red_blink")
        self.assertFalse(self.nav.differential_drive.call_count == 0)

    def test_premature_cube_loss_is_red_blink_until_pit(self):
        empty=(False,False,False)
        cube=(False,False,True)
        crossing=(True,False,False)
        pit=(False,True,False)
        frames=([empty]*4+[cube]*4+[empty]*4+
                [crossing]*3+[empty]*4+[pit]*5)
        self.assertEqual(self.run_sequence(frames),"emergency")
        self.signal.set_mode.assert_any_call("green_blink")
        self.signal.set_mode.assert_any_call("red_blink")

    def test_drop_after_junction_is_success(self):
        cube=(False,False,True)
        crossing=(True,False,True)
        empty=(False,False,False)
        frames=[cube]*5+[crossing]*3+[cube]*4+[empty]*4
        self.assertEqual(self.run_sequence(frames),"drop")
        self.signal.set_mode.assert_any_call("green_blink")
        self.signal.set_mode.assert_any_call("green")


if __name__=="__main__":
    unittest.main()

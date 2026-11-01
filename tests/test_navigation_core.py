"""Synthetic eight-channel sequences; no serial, GPIO, I2C or motors."""
import math
import tempfile
import unittest
from pathlib import Path

import line_tracker
from navigation_core import (FreshFrames, JunctionTracker, LineMonitor,
                             MotionCalibration, turn_stop_angle,
                             SENSOR_TO_AXLE_MM, CROSSBAR_WIDTH_MM)

CENTER = [0, 0, 0.1, 0.9, 0.9, 0.1, 0, 0]
LEFT = [1, 0.9, 0.1, 0.9, 0.9, 0.1, 0, 0]
RIGHT = [0, 0, 0.1, 0.9, 0.9, 0.1, 0.9, 1]
BOTH = [1, 0.9, 0.1, 0.9, 0.9, 0.1, 0.9, 1]
WHITE = [0] * 8


def frame(values, seq, when):
    return {'normalized': values, 'position': line_tracker.calculate_position(values),
            'sample_ids': [seq] * 8, 'sample_times': [when] * 8}


class NavigationCoreTests(unittest.TestCase):
    def test_01_centered_start(self):
        self.assertAlmostEqual(frame(CENTER, 1, 1)['position'], 0)
        self.assertTrue(FreshFrames().accept(frame(CENTER, 1, 1), 1.05))

    def test_02_shifted_start(self):
        shifted = [0, 0, 0, 0, 0.1, 0.9, 0.9, 0]
        self.assertGreater(frame(shifted, 1, 1)['position'], 1)

    def test_03_no_start_line(self):
        self.assertIsNone(frame(WHITE, 1, 1)['position'])

    def test_04_partial_adc_callbacks(self):
        state = frame(CENTER, 1, 1)
        state['sample_ids'][6] = 0
        self.assertFalse(FreshFrames().accept(state, 1.01))

    def test_05_stale_adc(self):
        self.assertFalse(FreshFrames().accept(frame(CENTER, 1, 1), 1.5))

    def test_06_incoherent_adc_frame(self):
        state = frame(CENTER, 1, 1)
        state['sample_times'][7] = 1.3
        self.assertFalse(FreshFrames().accept(state, 1.31))

    def test_07_straight(self):
        tracker = JunctionTracker()
        self.assertEqual([tracker.observe(frame(CENTER, n, n*.04), n*.04)[0]
                          for n in range(1, 8)], [None]*7)

    def _junction(self, pattern, exit_pattern=CENTER):
        tracker = JunctionTracker()
        events = []
        for n, values in enumerate([CENTER] + pattern + [exit_pattern], 1):
            events.append(tracker.observe(frame(values, n, n*.04), n*.04))
        return tracker, events

    def test_08_left_junction(self):
        _, events = self._junction([LEFT]*4)
        self.assertEqual([e[0] for e in events].count('LEFT'), 1)
        self.assertEqual(events[-1], ('EXIT', 'LEFT'))

    def test_09_right_junction(self):
        _, events = self._junction([RIGHT]*4)
        self.assertEqual([e[0] for e in events].count('RIGHT'), 1)
        self.assertEqual(events[-1], ('EXIT', 'RIGHT'))

    def test_10_t_junction(self):
        _, events = self._junction([BOTH]*4, WHITE)
        self.assertEqual(events[-1], ('EXIT', 'T'))

    def test_11_full_cross(self):
        _, events = self._junction([BOTH]*4)
        self.assertEqual(events[-1], ('EXIT', 'CROSS'))

    def test_12_staggered_outer_sensors(self):
        _, events = self._junction([LEFT, BOTH, BOTH, BOTH])
        self.assertEqual([e[0] for e in events].count('CROSS_OR_T'), 1)
        self.assertEqual(events[-1], ('EXIT', 'CROSS'))

    def test_13_same_junction_not_counted_twice(self):
        tracker, _ = self._junction([LEFT]*4)
        events = [tracker.observe(frame(LEFT, n, n*.04), n*.04)[0]
                  for n in range(10, 15)]
        self.assertNotIn('LEFT', events)

    def test_14_turn_lead_before_target(self):
        self.assertLess(turn_stop_angle('L', 90, 100), 90)
        self.assertLess(turn_stop_angle('R', 90, 100), 90)
        self.assertLess(turn_stop_angle('R', 90, 100),
                        turn_stop_angle('L', 90, 100))

    def test_15_loss_on_straight_requires_three_fresh_frames(self):
        monitor = LineMonitor()
        monitor.observe(frame(CENTER, 1, 1), 1, 0.08)
        self.assertFalse(monitor.observe(frame(WHITE, 2, 2), 2, 0.08)[0])
        self.assertFalse(monitor.observe(frame(WHITE, 3, 3), 3, 0.08)[0])
        self.assertTrue(monitor.observe(frame(WHITE, 4, 4), 4, 0.08)[0])

    def test_16_loss_after_turn_is_not_a_lock(self):
        monitor = LineMonitor()
        for n in range(1, 4):
            lost, _ = monitor.observe(frame(WHITE, n, n), n, 0.08)
        self.assertTrue(lost)
        self.assertEqual(len(monitor.history), 0)

    def test_17_one_frame_dropout_recovers(self):
        monitor = LineMonitor()
        monitor.observe(frame(WHITE, 1, 1), 1, 0.08)
        lost, _ = monitor.observe(frame(CENTER, 2, 2), 2, 0.08)
        self.assertFalse(lost)
        self.assertEqual(monitor.missing, 0)

    def test_18_missing_motion_calibration_refuses_alignment(self):
        self.assertIsNone(MotionCalibration().duration_after_trailing_edge())
        self.assertEqual(SENSOR_TO_AXLE_MM-CROSSBAR_WIDTH_MM/2, 55)

    def test_19_measured_motion_profile(self):
        profile = MotionCalibration({100: 100}, {100: 5}, {100: 3})
        self.assertAlmostEqual(profile.duration_after_trailing_edge(), 0.53)

    def test_20_replayed_frame_is_not_confirmation(self):
        gate = FreshFrames()
        state = frame(LEFT, 1, 1)
        self.assertTrue(gate.accept(state, 1.01))
        self.assertFalse(gate.accept(state, 1.02))

    def test_21_reverse_does_not_consume_junction(self):
        tracker = JunctionTracker()
        for n in range(1, 8):
            event, _ = tracker.observe(frame(BOTH, n, n*.04), n*.04,
                                       forward=False)
            self.assertIsNone(event)

    def test_22_line_loss_prediction_at_edge(self):
        monitor = LineMonitor()
        edge = [0, 0, 0, 0, 0, 0, 0.7, 1]
        _, predicted = monitor.observe(frame(edge, 1, 1), 1, 0.08)
        self.assertTrue(predicted)

    def test_23_new_junction_after_physical_clear(self):
        tracker, _ = self._junction([LEFT]*4)
        for n in range(10, 14):
            tracker.observe(frame(CENTER, n, n*.04), n*.04)
        events = [tracker.observe(frame(RIGHT, n, n*.04), n*.04)[0]
                  for n in range(14, 17)]
        self.assertEqual(events.count('RIGHT'), 1)


if __name__ == '__main__':
    unittest.main()

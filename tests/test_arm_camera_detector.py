import unittest

import cv2
import numpy as np

from arm_camera_button import ButtonDetector, IndicatorDetector


class ButtonDetectorTests(unittest.TestCase):
    def frame_with_red_object(self, x1, y1, x2, y2):
        frame = np.full((240, 320, 3), (70, 110, 55), dtype=np.uint8)
        points = np.array([
            [x1, y1],
            [x2, y1 + 8],
            [x2 - 20, y2],
            [x1 + 6, y2 - 15],
        ])
        cv2.fillPoly(frame, [points], (10, 20, 220))
        return frame

    def test_daniel_detection_does_not_require_white_or_round_object(self):
        detector = ButtonDetector()
        objects = detector.candidates(
            self.frame_with_red_object(20, 30, 210, 85)
        )
        self.assertEqual(len(objects), 1)
        self.assertEqual(objects[0]["color"], "R")
        self.assertGreater(objects[0]["w"] / objects[0]["h"], 2.0)

    def test_object_requires_click_then_tracks_across_frames(self):
        detector = ButtonDetector()
        first = self.frame_with_red_object(30, 40, 150, 100)
        target, objects = detector.detect(first)
        self.assertIsNone(target)
        self.assertEqual(len(objects), 1)

        detector.select(objects[0])
        target, _ = detector.detect(
            self.frame_with_red_object(75, 90, 230, 165)
        )
        self.assertIsNotNone(target)
        self.assertEqual(target["target_id"], 1)

    def test_indicator_roi_follows_selected_button(self):
        indicator = IndicatorDetector()
        indicator.set_roi(10, 10, 12, 12, anchor_center=(20, 20))
        self.assertEqual(
            indicator.current_roi((30, 25)),
            (20, 15, 12, 12),
        )
        frame = np.zeros((60, 60, 3), dtype=np.uint8)
        frame[15:27, 20:32] = (0, 0, 255)
        self.assertEqual(indicator.state(frame, (30, 25)), 'RED')


if __name__ == "__main__":
    unittest.main()

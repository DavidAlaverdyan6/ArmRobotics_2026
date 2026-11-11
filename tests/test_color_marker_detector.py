import numpy as np
import unittest
from urllib.request import urlopen

from color_marker_detector import (
    ColorMarkerDetector, ColorPreviewServer, rotate_camera_frame,
    run_color_marker_task,
)


class FakeLED:
    def __init__(self):
        self.calls = []

    def off(self):
        self.calls.append("off")

    def red(self):
        self.calls.append("red")

    def yellow(self):
        self.calls.append("yellow")

    def green(self):
        self.calls.append("green")

    def blue(self):
        self.calls.append("blue")


def test_detects_each_color_and_returns_bounding_boxes():
    frame = np.zeros((140, 240, 3), dtype=np.uint8)
    frame[10:60, 10:60] = (0, 0, 255)
    frame[10:60, 70:120] = (0, 255, 255)
    frame[70:120, 10:60] = (0, 255, 0)
    frame[70:120, 70:120] = (255, 0, 0)

    detector = ColorMarkerDetector(min_object_area=100, stable_frames=1)
    objects, annotated, confirmed = detector.process_frame(frame)

    assert {obj["color"] for obj in objects} == {
        "red", "yellow", "green", "blue"
    }
    assert all(obj["w"] == 50 and obj["h"] == 50 for obj in objects)
    assert annotated.shape == frame.shape
    assert confirmed in {"red", "yellow", "green", "blue"}


def test_camera_frame_rotation_is_180_degrees():
    frame = np.zeros((4, 6, 3), dtype=np.uint8)
    frame[0, 0] = (1, 2, 3)
    rotated = rotate_camera_frame(frame)
    assert tuple(rotated[-1, -1]) == (1, 2, 3)


def test_touching_different_colors_follow_daniel_combined_contour():
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    frame[20:80, 10:50] = (0, 0, 255)
    frame[20:80, 50:90] = (255, 0, 0)

    detector = ColorMarkerDetector(min_object_area=100, stable_frames=1)
    objects, _, _ = detector.process_frame(frame)

    assert len(objects) == 1
    assert objects[0]["w"] == 80


def test_allowed_colors_excludes_yellow_for_robot_task():
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    frame[10:50, 10:50] = (0, 255, 255)
    frame[55:95, 55:95] = (0, 255, 0)
    detector = ColorMarkerDetector(
        min_object_area=100,
        stable_frames=1,
        allowed_colors={"red", "green", "blue"},
    )
    objects, _, confirmed = detector.process_frame(frame)
    assert {obj["color"] for obj in objects} == {"green"}
    assert confirmed == "green"


def test_led_changes_only_after_stable_detection_and_turns_off_on_loss():
    red = np.zeros((80, 80, 3), dtype=np.uint8)
    red[10:70, 10:70] = (0, 0, 255)
    empty = np.zeros_like(red)
    led = FakeLED()
    detector = ColorMarkerDetector(
        led=led, min_object_area=100, stable_frames=2
    )

    assert detector.process_frame(red)[2] is None
    assert led.calls == []
    assert detector.process_frame(red)[2] == "red"
    assert led.calls == ["red"]
    assert detector.process_frame(empty)[2] is None
    assert led.calls == ["red", "off"]


class SharedCameraTests(unittest.TestCase):
    def test_color_task_uses_successive_frames_from_shared_camera(self):
        frame = np.zeros((80, 80, 3), dtype=np.uint8)
        frame[10:70, 10:70] = (0, 0, 255)

        class SharedCamera:
            def __init__(self):
                self.requested = []
                self.sequence = 7

            def get_frame(self, after_sequence=0, timeout=0.5):
                self.requested.append(after_sequence)
                return after_sequence + 1, frame.copy()

        camera = SharedCamera()
        marker = run_color_marker_task(
            timeout_s=1.0,
            hold_seconds=0.0,
            min_object_area=100,
            stable_frames=3,
            control_led=False,
            preview_port=None,
            camera_stream=camera,
        )
        self.assertEqual(marker["color"], "red")
        self.assertEqual(camera.requested, [7, 8, 9])

    def test_preview_keeps_live_and_task_frames_on_separate_endpoints(self):
        preview = ColorPreviewServer(port=0)
        task = np.zeros((80, 100, 3), dtype=np.uint8)
        task[:, :] = (0, 0, 255)
        try:
            preview.start()
            self.assertTrue(preview.update_task(task, "COLOR detected RED"))
            port = preview._httpd.server_port
            with urlopen(
                f"http://127.0.0.1:{port}/", timeout=2
            ) as response:
                page = response.read()
            with urlopen(
                f"http://127.0.0.1:{port}/task.jpg", timeout=2
            ) as response:
                task_jpeg = response.read()
            with urlopen(
                f"http://127.0.0.1:{port}/task-label", timeout=2
            ) as response:
                label = response.read().decode()
            self.assertIn(b"/stream.mjpg", page)
            self.assertIn(b"/task.jpg", page)
            self.assertTrue(task_jpeg.startswith(b"\xff\xd8"))
            self.assertEqual(label, "COLOR detected RED")
        finally:
            preview.stop()

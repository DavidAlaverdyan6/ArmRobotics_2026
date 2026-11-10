import numpy as np

from color_marker_detector import ColorMarkerDetector


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


def test_touching_different_colors_remain_separate_objects():
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    frame[20:80, 10:50] = (0, 0, 255)
    frame[20:80, 50:90] = (255, 0, 0)

    detector = ColorMarkerDetector(min_object_area=100, stable_frames=1)
    objects, _, _ = detector.process_frame(frame)

    assert len(objects) == 2
    assert {obj["color"] for obj in objects} == {"red", "blue"}


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

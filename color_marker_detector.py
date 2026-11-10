"""Color-marker bounding boxes and RGB LED integration.

The pixel classifier is based on
``Archive/2_bar_follow/Color_Detector_Daniel.py``.  This module keeps camera
and GPIO setup out of import time so the detector can be reused safely from
``main.py`` and tested with ordinary NumPy images.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

import cv2
import numpy as np


COLOR_NAMES = {
    1: "red",
    2: "yellow",
    3: "green",
    4: "blue",
}

DRAW_COLORS = {
    "red": (0, 0, 255),
    "yellow": (0, 255, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
}


class ColorMarkerDetector:
    """Find colored objects, draw boxes, and drive an optional ``led.RGBLED``.

    Frames use OpenCV's BGR channel order, matching Daniel's original
    detector.  ``process_frame`` returns ``(objects, annotated_frame,
    confirmed_color)``.  The confirmed color is the largest object's color
    after it has remained unchanged for ``stable_frames`` calls.
    """

    RED = 1
    YELLOW = 2
    GREEN = 3
    BLUE = 4

    def __init__(
        self,
        led=None,
        min_object_area: int = 700,
        min_brightness: int = 60,
        min_channel_diff: int = 25,
        stable_frames: int = 3,
    ):
        if min_object_area <= 0:
            raise ValueError("min_object_area must be positive")
        if stable_frames <= 0:
            raise ValueError("stable_frames must be positive")

        self.led = led
        self.min_object_area = min_object_area
        self.min_brightness = min_brightness
        self.min_channel_diff = min_channel_diff
        self.stable_frames = stable_frames
        self._candidate_color: Optional[str] = None
        self._candidate_frames = 0
        self.confirmed_color: Optional[str] = None

    def classify_colors(self, frame: np.ndarray) -> np.ndarray:
        """Return a map containing one color ID per classified BGR pixel."""
        if frame is None or frame.ndim != 3 or frame.shape[2] < 3:
            raise ValueError("frame must be a BGR image with three channels")

        b = frame[:, :, 0].astype(np.uint16)
        g = frame[:, :, 1].astype(np.uint16)
        r = frame[:, :, 2].astype(np.uint16)
        color_map = np.zeros(frame.shape[:2], dtype=np.uint8)

        max_channel = np.maximum(np.maximum(r, g), b)
        min_channel = np.minimum(np.minimum(r, g), b)
        valid = (
            (max_channel >= self.min_brightness)
            & ((max_channel - min_channel) >= self.min_channel_diff)
        )

        yellow = valid & (
            (r > 120)
            & (g > 100)
            & (b * 100 < np.minimum(r, g) * 65)
        )
        color_map[yellow] = self.YELLOW

        red = valid & (r * 100 > g * 135) & (r * 100 > b * 135)
        color_map[red] = self.RED

        green = valid & (g * 100 > r * 115) & (g * 100 > b * 120)
        color_map[green] = self.GREEN

        blue = valid & (b * 100 > r * 125) & (b * 100 > g * 115)
        color_map[blue] = self.BLUE
        return color_map

    def find_objects(self, color_map: np.ndarray) -> list[dict]:
        """Return one bounding box per sufficiently large colored region."""
        objects = []

        # Contour each color independently.  Daniel's combined mask merged
        # touching markers of different colors into one bounding box.
        for color_id, color_name in COLOR_NAMES.items():
            mask = (color_map == color_id).astype(np.uint8) * 255
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_object_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                objects.append(
                    {
                        "color": color_name,
                        "x": x,
                        "y": y,
                        "w": w,
                        "h": h,
                        "center": (x + w // 2, y + h // 2),
                        "area": int(area),
                    }
                )

        return sorted(objects, key=lambda obj: obj["area"], reverse=True)

    def draw_objects(
        self,
        frame: np.ndarray,
        objects: list[dict],
        color_map: np.ndarray,
    ) -> np.ndarray:
        """Return a copy with classified pixels, labels, and bounding boxes."""
        result = frame.copy()
        for color_id, color_name in COLOR_NAMES.items():
            result[color_map == color_id] = DRAW_COLORS[color_name]

        for index, obj in enumerate(objects, 1):
            x, y, w, h = obj["x"], obj["y"], obj["w"], obj["h"]
            draw_color = DRAW_COLORS[obj["color"]]
            cv2.rectangle(result, (x, y), (x + w - 1, y + h - 1), draw_color, 3)
            cv2.drawMarker(
                result,
                obj["center"],
                (255, 255, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=14,
                thickness=2,
            )
            cv2.putText(
                result,
                f"{index}: {obj['color']}",
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
        return result

    def _set_led(self, color: Optional[str]) -> None:
        if self.led is None:
            return
        if color is None:
            self.led.off()
            return
        getattr(self.led, color)()

    def _update_confirmation(self, objects: list[dict]) -> Optional[str]:
        largest_color = objects[0]["color"] if objects else None
        if largest_color == self._candidate_color:
            self._candidate_frames += 1
        else:
            self._candidate_color = largest_color
            self._candidate_frames = 1

        new_color = (
            largest_color
            if largest_color is not None
            and self._candidate_frames >= self.stable_frames
            else None
        )
        if new_color != self.confirmed_color:
            self.confirmed_color = new_color
            self._set_led(new_color)
        return self.confirmed_color

    def process_frame(self, frame: np.ndarray):
        color_map = self.classify_colors(frame)
        objects = self.find_objects(color_map)
        annotated = self.draw_objects(frame, objects, color_map)
        confirmed = self._update_confirmation(objects)
        return objects, annotated, confirmed

    def reset(self) -> None:
        self._candidate_color = None
        self._candidate_frames = 0
        self.confirmed_color = None
        self._set_led(None)


def run_color_marker_task(
    timeout_s: float = 10.0,
    hold_seconds: float = 2.0,
    min_object_area: int = 10_000,
    stable_frames: int = 3,
    frame_callback: Optional[Callable[[np.ndarray, list[dict]], None]] = None,
) -> Optional[dict]:
    """Capture frames until a stable marker is found and light its LED color.

    Hardware imports are intentionally local.  The returned dictionary is the
    largest matching object, or ``None`` on timeout.  ``frame_callback`` can be
    used by a camera server to publish each annotated frame.
    """
    from picamera2 import Picamera2

    import led
    import robot_pin_config as pins

    camera = None
    rgb = None
    detector = None
    try:
        # These swapped arguments match the robot wiring already documented in
        # main's charging task: its physical red and blue channels are swapped.
        rgb = led.RGBLED(pins.LED_BLUE, pins.LED_GREEN, pins.LED_RED)
        detector = ColorMarkerDetector(
            led=rgb,
            min_object_area=min_object_area,
            stable_frames=stable_frames,
        )

        camera = Picamera2()
        config = camera.create_preview_configuration(
            main={"size": (800, 600), "format": "RGB888"}
        )
        camera.configure(config)
        camera.start()
        time.sleep(1.0)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = camera.capture_array()
            objects, annotated, confirmed = detector.process_frame(frame)
            if frame_callback is not None:
                frame_callback(annotated, objects)
            if confirmed is not None:
                match = next(obj for obj in objects if obj["color"] == confirmed)
                print(
                    "Color marker detected: "
                    f"{confirmed.upper()} box=(x={match['x']}, y={match['y']}, "
                    f"w={match['w']}, h={match['h']})"
                )
                time.sleep(max(0.0, hold_seconds))
                return match

        print("Color marker detection timed out.")
        return None
    finally:
        try:
            if detector is not None:
                detector.reset()
            elif rgb is not None:
                rgb.off()
        except Exception:
            pass
        if camera is not None:
            try:
                camera.stop()
            except Exception:
                pass
            try:
                camera.close()
            except Exception:
                pass
        if rgb is not None:
            try:
                rgb.close()
            except Exception:
                pass

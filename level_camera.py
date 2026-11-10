"""Reusable wrist-camera color recognition for ArmRobotics levels 6-8.

Camera owns its Picamera2 instance. Polling never commands motors/servos.
Uses the repository's existing ColorMarkerDetector (BGR images).
"""
from __future__ import annotations

from dataclasses import dataclass
from threading import Event, Lock, Thread
import time

import cv2
from color_marker_detector import ColorMarkerDetector


@dataclass(frozen=True)
class CameraDetection:
    color: str
    x: int
    y: int
    w: int
    h: int
    area: int
    captured_at: float


class LevelCamera:
    """Independent color camera with bottom-of-frame ROI and stable detection.

    start() starts capture in a worker. poll() is nonblocking and returns each
    confirmed detection at most once (a repeated old frame is not reprocessed).
    Configure expected_colors according to the *actual* level rules.
    """

    def __init__(
        self,
        expected_colors=("red", "green", "blue"),
        bottom_fraction=0.30,
        width=640,
        height=480,
        rotate_180=True,
        min_object_area=700,
        stable_frames=3,
        max_frame_age=0.25,
        led_callback=None,
    ):
        valid_colors = {"red", "green", "blue", "yellow"}
        if not expected_colors or not set(expected_colors) <= valid_colors:
            raise ValueError("expected_colors must be nonempty supported colors")
        if not 0 < bottom_fraction <= 1:
            raise ValueError("bottom_fraction must be in (0, 1]")
        if max_frame_age <= 0:
            raise ValueError("max_frame_age must be positive")
        self.expected_colors = frozenset(expected_colors)
        self.bottom_fraction = bottom_fraction
        self.width, self.height = width, height
        self.rotate_180 = rotate_180
        self.max_frame_age = max_frame_age
        self.led_callback = led_callback
        self.detector = ColorMarkerDetector(
            min_object_area=min_object_area,
            stable_frames=stable_frames,
        )
        self._camera = None
        self._worker = None
        self._stop = Event()
        self._lock = Lock()
        self._latest = None
        self._sequence = 0
        self._processed_sequence = 0
        self._last_report = None
        self._error = None

    def start(self):
        if self._camera is not None:
            return
        from picamera2 import Picamera2

        camera = Picamera2()
        try:
            cfg = camera.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"}
            )
            camera.configure(cfg)
            camera.start()
        except BaseException:
            camera.close()
            raise
        self._camera = camera
        self._error = None
        self._stop.clear()
        self._worker = Thread(target=self._capture, daemon=True)
        self._worker.start()

    def _capture(self):
        while not self._stop.is_set():
            try:
                # Existing detector expects BGR arrays. Picamera2 RGB888 capture
                # arrays are BGR ordered on the supported Pi camera pipeline.
                frame = self._camera.capture_array()
                if self.rotate_180:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                with self._lock:
                    self._sequence += 1
                    self._latest = (self._sequence, time.monotonic(), frame)
            except Exception as exc:
                self._error = exc
                self._stop.set()

    def poll(self):
        """Return a new confirmed CameraDetection or None; never drive robot."""
        if self._error is not None:
            raise RuntimeError("Camera capture failed") from self._error
        with self._lock:
            latest = self._latest
        if latest is None:
            return None
        seq, stamp, frame = latest
        if seq == self._processed_sequence:
            return None
        self._processed_sequence = seq
        if time.monotonic() - stamp > self.max_frame_age:
            self.detector.reset()
            self._last_report = None
            return None

        h = frame.shape[0]
        y0 = max(0, int(h * (1 - self.bottom_fraction)))
        roi = frame[y0:].copy()
        # Restrict classification to configured target colors BEFORE stability
        # confirmation; unrelated colors cannot suppress target detection.
        cmap = self.detector.classify_colors(roi)
        for color_id, name in ((1, "red"), (2, "yellow"),
                               (3, "green"), (4, "blue")):
            if name not in self.expected_colors:
                cmap[cmap == color_id] = 0
        objects = self.detector.find_objects(cmap)
        confirmed = self.detector._update_confirmation(objects)
        if confirmed is None:
            self._last_report = None
            return None
        match = next((obj for obj in objects if obj["color"] == confirmed), None)
        if match is None:
            return None
        detection = CameraDetection(
            confirmed, match["x"], match["y"] + y0,
            match["w"], match["h"], match["area"], stamp
        )
        # Deliver only the rising edge; caller may explicitly rearm.
        if self._last_report == confirmed:
            return None
        self._last_report = confirmed
        if self.led_callback is not None:
            self.led_callback(confirmed)
        return detection

    def arm(self, expected_colors=None):
        """Rearm at a new stage or after a maneuver; resets stable-frame count."""
        if expected_colors is not None:
            valid = {"red", "green", "blue", "yellow"}
            if not expected_colors or not set(expected_colors) <= valid:
                raise ValueError("Invalid target colors")
            self.expected_colors = frozenset(expected_colors)
        self.detector.reset()
        self._last_report = None
        # Do not consume a captured image from the previous stage.
        with self._lock:
            self._processed_sequence = self._sequence

    def close(self):
        self._stop.set()
        if self._worker is not None:
            self._worker.join(timeout=2.0)
            if self._worker.is_alive():
                raise RuntimeError("Camera worker did not stop; cannot close safely")
            self._worker = None
        if self._camera is not None:
            self._camera.stop()
            self._camera.close()
            self._camera = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.close()

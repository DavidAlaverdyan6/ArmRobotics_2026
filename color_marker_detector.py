"""Color-marker bounding boxes and RGB LED integration.

The pixel classifier is based on
``Archive/2_bar_follow/Color_Detector_Daniel.py``.  This module keeps camera
and GPIO setup out of import time so the detector can be reused safely from
``main.py`` and tested with ordinary NumPy images.
"""

from __future__ import annotations

import socket
import socketserver
import threading
import time
from http import server
from urllib.parse import urlsplit
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


class ColorPreviewServer:
    """Publish the annotated detector frame as an MJPEG stream."""

    def __init__(self, port=8000):
        self.port = int(port)
        self._condition = threading.Condition()
        self._frame = None
        self._sequence = 0
        self._task_frame = self._make_placeholder()
        self._task_sequence = 0
        self._task_label = "No completed camera task yet"
        self._closed = False
        self._httpd = None
        self._thread = None
        self._clients = 0

    @staticmethod
    def _make_placeholder():
        frame = np.zeros((600, 800, 3), dtype=np.uint8)
        cv2.putText(
            frame,
            "No completed camera task yet",
            (105, 310),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (210, 210, 210),
            2,
        )
        ok, encoded = cv2.imencode(".jpg", frame)
        return encoded.tobytes() if ok else b""

    @property
    def has_viewers(self):
        with self._condition:
            return self._clients > 0

    def start(self):
        preview = self

        class Handler(server.BaseHTTPRequestHandler):
            def do_GET(self):
                path = urlsplit(self.path).path
                if path in ("/", "/index.html"):
                    content = (
                        '<!doctype html><html><head><meta charset="utf-8">'
                        '<title>Robot cameras</title><style>'
                        'body{background:#111;color:#eee;font-family:Arial;'
                        'margin:18px;text-align:center}.grid{display:grid;'
                        'grid-template-columns:repeat(auto-fit,minmax(360px,1fr));'
                        'gap:18px}.panel{background:#202020;padding:12px;'
                        'border-radius:10px}img{width:100%;height:auto;'
                        'background:#000}h2{margin:4px 0 10px}'
                        '#taskLabel{color:#ffc65c}</style></head><body>'
                        '<h1>Robot cameras</h1><div class="grid">'
                        '<div class="panel"><h2>Live camera</h2>'
                        '<img src="/stream.mjpg"></div>'
                        '<div class="panel"><h2>Task camera</h2>'
                        '<div id="taskLabel">Waiting for a completed task</div>'
                        '<img id="taskFrame" src="/task-stream.mjpg"></div></div>'
                        '<script>async function refreshTask(){'
                        'try{const r=await fetch('
                        '"/task-label?t="+Date.now());document.getElementById('
                        '"taskLabel").textContent=await r.text()}catch(e){}}'
                        'setInterval(refreshTask,1000);refreshTask();</script>'
                        '</body></html>'
                    ).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(content)))
                    self.end_headers()
                    self.wfile.write(content)
                    return
                if path == "/task.jpg":
                    with preview._condition:
                        task_frame = preview._task_frame
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(task_frame)))
                    self.end_headers()
                    self.wfile.write(task_frame)
                    return
                if path == "/task-label":
                    with preview._condition:
                        label = preview._task_label.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(label)))
                    self.end_headers()
                    self.wfile.write(label)
                    return
                if path not in ("/stream.mjpg", "/task-stream.mjpg"):
                    self.send_error(404)
                    return
                task_stream = path == "/task-stream.mjpg"
                self.send_response(200)
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=FRAME",
                )
                self.end_headers()
                last_sequence = -1
                if not task_stream:
                    with preview._condition:
                        preview._clients += 1
                try:
                    while True:
                        with preview._condition:
                            preview._condition.wait_for(
                                lambda: (
                                    preview._task_sequence
                                    if task_stream else preview._sequence
                                ) != last_sequence
                                or preview._closed,
                                timeout=1.0,
                            )
                            if preview._closed:
                                return
                            if task_stream:
                                frame = preview._task_frame
                                last_sequence = preview._task_sequence
                            else:
                                frame = preview._frame
                                last_sequence = preview._sequence
                        if frame is None:
                            continue
                        self.wfile.write(b"--FRAME\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(frame)}\r\n\r\n".encode()
                        )
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    return
                finally:
                    if not task_stream:
                        with preview._condition:
                            preview._clients -= 1

            def log_message(self, _format, *args):
                return

        class ThreadingServer(socketserver.ThreadingMixIn, server.HTTPServer):
            allow_reuse_address = True
            daemon_threads = True

        self._httpd = ThreadingServer(("0.0.0.0", self.port), Handler)
        self._thread = threading.Thread(
            target=self._httpd.serve_forever,
            name="color-preview",
            daemon=True,
        )
        self._thread.start()
        try:
            probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            probe.connect(("8.8.8.8", 80))
            ip = probe.getsockname()[0]
            probe.close()
        except OSError:
            ip = "<robot-ip>"
        print(f"COLOR camera preview: http://{ip}:{self.port}")

    def update(self, frame):
        ok, encoded = cv2.imencode(".jpg", frame)
        if not ok:
            return
        with self._condition:
            self._frame = encoded.tobytes()
            self._sequence += 1
            self._condition.notify_all()

    def update_task(self, frame, label):
        """Replace the persistent result frame after a camera based task."""
        ok, encoded = cv2.imencode(".jpg", frame)
        if not ok:
            return False
        with self._condition:
            self._task_frame = encoded.tobytes()
            self._task_sequence += 1
            self._task_label = str(label)
            self._condition.notify_all()
        return True

    def stop(self):
        with self._condition:
            self._closed = True
            self._condition.notify_all()
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


class ContinuousCamera:
    """Keep one camera and preview running throughout the robot route."""

    def __init__(self, port=8000):
        self.port = port
        self.camera = None
        self.preview = None
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._frame = None
        self._sequence = 0
        self._error = None
        self._thread = None

    @property
    def sequence(self):
        with self._condition:
            return self._sequence

    def start(self):
        from picamera2 import Picamera2

        try:
            self.camera = Picamera2()
            config = self.camera.create_preview_configuration(
                main={"size": (800, 600), "format": "RGB888"},
                controls={"FrameRate": 12},
            )
            self.camera.configure(config)
            self.camera.start()
            self.preview = ColorPreviewServer(self.port)
            self.preview.start()
            self._thread = threading.Thread(
                target=self._capture_loop,
                name="route-camera",
                daemon=True,
            )
            self._thread.start()
        except Exception:
            self.stop()
            raise

    def _capture_loop(self):
        try:
            while not self._stop.is_set():
                frame = rotate_camera_frame(self.camera.capture_array())
                with self._condition:
                    self._frame = frame
                    self._sequence += 1
                    self._condition.notify_all()
                if self.preview.has_viewers:
                    self.preview.update(frame)
        except Exception as exc:
            if not self._stop.is_set():
                with self._condition:
                    self._error = exc
                    self._condition.notify_all()

    def get_frame(self, after_sequence=0, timeout=0.5):
        """Return a new rotated frame, waiting for the next camera exposure."""
        with self._condition:
            self._condition.wait_for(
                lambda: self._sequence > after_sequence
                or self._error is not None or self._stop.is_set(),
                timeout=timeout,
            )
            if self._error is not None:
                raise RuntimeError("Camera capture stopped") from self._error
            if self._sequence <= after_sequence:
                return None
            return self._sequence, self._frame.copy()

    def save_task_frame(self, frame, label):
        """Publish a completed task result without changing the live stream."""
        if self.preview is None:
            return False
        return self.preview.update_task(frame, label)

    def save_latest_task_frame(self, label):
        """Freeze and label the latest live exposure for a sensor based task."""
        with self._condition:
            if self._frame is None:
                return False
            frame = self._frame.copy()
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 48), (0, 0, 0), -1)
        cv2.putText(
            frame,
            str(label),
            (12, 33),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )
        return self.save_task_frame(frame, label)

    def stop(self):
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if self.camera is not None:
            try:
                self.camera.stop()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.preview is not None:
            self.preview.stop()
        if self.camera is not None:
            try:
                self.camera.close()
            except Exception:
                pass


def rotate_camera_frame(frame: np.ndarray) -> np.ndarray:
    """Rotate the camera image to compensate for the inverted mount."""
    if frame is None or frame.ndim != 3 or frame.shape[2] < 3:
        raise ValueError("frame must be a BGR/RGB image with three channels")
    return cv2.rotate(frame, cv2.ROTATE_180)


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
        allowed_colors=None,
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
        self.allowed_colors = (
            None if allowed_colors is None else set(allowed_colors)
        )
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
        hsv = cv2.cvtColor(frame[:, :, :3], cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0]
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
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

        # Blue tape in the live camera reaches B>=113 with at least 23 levels
        # of separation from the next channel. Dark/black material produced
        # only B=60..99 and 10..18 levels of separation, so reject that noise
        # while retaining a margin below the observed blue range.
        blue = (
            valid
            # The camera's white balance makes white floor and black tape
            # look blue in raw BGR ratios. Keep actual saturated blue/cyan
            # light, while rejecting the low-saturation blue cast.
            & (hue >= 95)
            & (hue <= 135)
            & (saturation >= 120)
            & (value >= 80)
            & (b >= 100)
            & (b - np.maximum(r, g) >= 20)
            & (b * 100 > r * 125)
            & (b * 100 > g * 115)
        )
        color_map[blue] = self.BLUE
        return color_map

    def find_objects(self, color_map: np.ndarray) -> list[dict]:
        """Return one bounding box per sufficiently large colored region."""
        objects = []

        # Keep Daniel.py's object extraction exactly: contour the combined
        # color mask and identify the object from its first contour pixel.
        mask = (color_map != 0).astype(np.uint8) * 255
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_object_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            px_x, px_y = contour[0][0]
            color_id = int(color_map[px_y, px_x])
            color_name = COLOR_NAMES.get(color_id)
            if color_name is None:
                continue
            if (self.allowed_colors is not None
                    and color_name not in self.allowed_colors):
                continue
            objects.append({
                "color": color_name,
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "center": (x + w // 2, y + h // 2),
                "area": int(area),
            })

        # The caller treats the first object as the strongest detection.
        # OpenCV's contour order is spatial/arbitrary, not size based.
        objects.sort(key=lambda obj: obj["area"], reverse=True)
        return objects

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
    control_led: bool = True,
    confirmation_callback: Optional[Callable[[dict], None]] = None,
    preview_port: Optional[int] = 8000,
    camera_stream: Optional[ContinuousCamera] = None,
    task_frame_callback: Optional[Callable[[np.ndarray, dict], None]] = None,
) -> Optional[dict]:
    """Capture frames until a stable marker is found and light its LED color.

    Hardware imports are intentionally local.  The returned dictionary is the
    largest matching object, or ``None`` on timeout.  ``frame_callback`` can be
    used by a camera server to publish each annotated frame.
    """
    camera = None
    rgb = None
    detector = None
    preview = None
    status_open = False
    try:
        if control_led:
            import led
            import robot_pin_config as pins
            # Physical red/blue channels are swapped on this robot.
            rgb = led.RGBLED(pins.LED_BLUE, pins.LED_GREEN, pins.LED_RED)
        detector = ColorMarkerDetector(
            led=rgb,
            min_object_area=min_object_area,
            stable_frames=stable_frames,
            allowed_colors={"red", "green", "blue"},
        )

        if camera_stream is None:
            from picamera2 import Picamera2
            camera = Picamera2()
            config = camera.create_preview_configuration(
                main={"size": (800, 600), "format": "RGB888"}
            )
            camera.configure(config)
            camera.start()
            time.sleep(0.25)

        if camera_stream is None and preview_port is not None:
            try:
                preview = ColorPreviewServer(preview_port)
                preview.start()
            except OSError as exc:
                preview = None
                print(f"COLOR camera preview unavailable: {exc}")

        deadline = time.monotonic() + timeout_s
        last_status = 0.0
        # Begin with the next exposure after the arm reached its COLOR pose.
        last_sequence = (
            camera_stream.sequence if camera_stream is not None else 0
        )
        while time.monotonic() < deadline:
            if camera_stream is None:
                # Picamera2 RGB888 is returned in OpenCV's BGR byte order.
                frame = rotate_camera_frame(camera.capture_array())
            else:
                captured = camera_stream.get_frame(
                    after_sequence=last_sequence,
                    timeout=min(0.25, max(0.0, deadline-time.monotonic())),
                )
                if captured is None:
                    continue
                last_sequence, frame = captured
            objects, annotated, confirmed = detector.process_frame(frame)
            if preview is not None:
                preview.update(annotated)
            if frame_callback is not None:
                frame_callback(annotated, objects)
            now = time.monotonic()
            if now - last_status >= 0.2:
                if objects:
                    seen = objects[0]["color"].upper()
                    detail = f" area={objects[0]['area']}"
                else:
                    seen = "NONE"
                    detail = ""
                print(
                    f"\r\033[2KCOLOR CAMERA: {seen}{detail}",
                    end="",
                    flush=True,
                )
                status_open = True
                last_status = now
            if confirmed is not None:
                match = next(obj for obj in objects if obj["color"] == confirmed)
                if status_open:
                    print("\r\033[2K", end="", flush=True)
                    status_open = False
                print(
                    "Color marker detected: "
                    f"{confirmed.upper()} box=(x={match['x']}, y={match['y']}, "
                    f"w={match['w']}, h={match['h']})"
                )
                if confirmation_callback is not None:
                    confirmation_callback(match)
                if task_frame_callback is not None:
                    task_frame_callback(annotated.copy(), match)
                time.sleep(max(0.0, hold_seconds))
                return match

        if status_open:
            print("\r\033[2K", end="", flush=True)
            status_open = False
        print("Color marker detection timed out.")
        return None
    finally:
        if status_open:
            print("\r\033[2K", end="", flush=True)
        if preview is not None:
            preview.stop()
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

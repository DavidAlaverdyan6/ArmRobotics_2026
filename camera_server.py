# camera.py
#
# Central camera driver + HTTP preview server.
#
# Provides:
#   camera.start()
#   camera.stop()
#   camera.get_frame()              -> BGR OpenCV frame
#   camera.set_processed_frame()
#   camera.get_processed_frame()
#
# HTTP:
#   http://<PI-IP>:8000/
#   http://<PI-IP>:8000/stream.mjpg
#   http://<PI-IP>:8000/processed.mjpg
#   http://<PI-IP>:8000/status
#
# No Telemetrix.
# No motors.
# No line-following logic.

import time
import threading
import io
import json

import cv2
import numpy as np
from PIL import Image
from http.server import BaseHTTPRequestHandler, HTTPServer

from picamera2 import Picamera2


# ============================================================
# CONFIGURATION
# ============================================================

WIDTH = 416
HEIGHT = 416

JPEG_QUALITY = 75

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8000

CAPTURE_DELAY = 0.001


# ============================================================
# CAMERA CLASS
# ============================================================

class Camera:

    def __init__(
        self,
        width=WIDTH,
        height=HEIGHT,
        host=SERVER_HOST,
        port=SERVER_PORT
    ):

        self.width = width
        self.height = height

        self.host = host
        self.port = port

        self.picam2 = None

        self.running = False

        self.capture_thread = None
        self.server_thread = None

        self.frame_lock = threading.Lock()
        self.jpeg_lock = threading.Lock()

        self.frame = None
        self.processed_frame = None

        self.latest_jpeg = None
        self.latest_processed_jpeg = None

        self.frame_number = 0

        self.start_time = None

        self.server = None


    # ========================================================
    # START
    # ========================================================

    def start(self):

        if self.running:
            return

        print()
        print("========================================")
        print("Starting camera")
        print("========================================")

        self.picam2 = Picamera2()

        config = self.picam2.create_preview_configuration(
            main={
                "size": (
                    self.width,
                    self.height
                ),
                "format": "RGB888"
            }
        )

        self.picam2.configure(config)

        self.picam2.start()

        time.sleep(1)

        self.running = True
        self.start_time = time.monotonic()

        print(
            f"Camera started: "
            f"{self.width}x{self.height}"
        )

        # ----------------------------------------------------
        # Capture thread
        # ----------------------------------------------------

        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            daemon=True
        )

        self.capture_thread.start()

        # ----------------------------------------------------
        # HTTP server
        # ----------------------------------------------------

        self.server_thread = threading.Thread(
            target=self._server_loop,
            daemon=True
        )

        self.server_thread.start()

        print()
        print("Camera server:")
        print(
            f"http://<PI-IP>:{self.port}"
        )
        print()


    # ========================================================
    # STOP
    # ========================================================

    def stop(self):

        if not self.running:
            return

        print("Stopping camera...")

        self.running = False

        if self.server is not None:

            try:
                self.server.shutdown()
            except Exception:
                pass

            try:
                self.server.server_close()
            except Exception:
                pass

        if self.picam2 is not None:

            try:
                self.picam2.stop()
            except Exception:
                pass

            self.picam2 = None

        print("Camera stopped.")


    # ========================================================
    # CAPTURE LOOP
    # ========================================================

    def _capture_loop(self):

        print("Camera capture thread started.")

        while self.running:

            try:

                frame = self.picam2.capture_array()

                # Picamera2 is configured as RGB888.
                # OpenCV expects BGR.

                frame = cv2.cvtColor(
                    frame,
                    cv2.COLOR_RGB2BGR
                )

                with self.frame_lock:

                    self.frame = frame.copy()

                    self.frame_number += 1

                # ------------------------------------------------
                # Generate JPEG for browser preview
                # ------------------------------------------------

                self._update_jpeg(
                    frame
                )

            except Exception as e:

                print(
                    "Camera capture error:",
                    e
                )

                time.sleep(0.1)

            time.sleep(
                CAPTURE_DELAY
            )

        print("Camera capture thread stopped.")


    # ========================================================
    # JPEG GENERATION
    # ========================================================

    def _update_jpeg(
        self,
        frame
    ):

        try:

            success, encoded = cv2.imencode(
                ".jpg",
                frame,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    JPEG_QUALITY
                ]
            )

            if success:

                jpeg = encoded.tobytes()

                with self.jpeg_lock:

                    self.latest_jpeg = jpeg

        except Exception as e:

            print(
                "JPEG error:",
                e
            )


    # ========================================================
    # GET CURRENT FRAME
    # ========================================================

    def get_frame(self):

        with self.frame_lock:

            if self.frame is None:
                return None

            return self.frame.copy()


    # ========================================================
    # GET FRAME NUMBER
    # ========================================================

    def get_frame_number(self):

        with self.frame_lock:

            return self.frame_number


    # ========================================================
    # SET PROCESSED FRAME
    # ========================================================

    def set_processed_frame(
        self,
        frame
    ):

        if frame is None:
            return

        with self.frame_lock:

            self.processed_frame = frame.copy()

        try:

            success, encoded = cv2.imencode(
                ".jpg",
                frame,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    JPEG_QUALITY
                ]
            )

            if success:

                jpeg = encoded.tobytes()

                with self.jpeg_lock:

                    self.latest_processed_jpeg = jpeg

        except Exception as e:

            print(
                "Processed JPEG error:",
                e
            )


    # ========================================================
    # GET PROCESSED FRAME
    # ========================================================

    def get_processed_frame(self):

        with self.frame_lock:

            if self.processed_frame is None:
                return None

            return self.processed_frame.copy()


    # ========================================================
    # STATUS
    # ========================================================

    def get_status(self):

        uptime = 0

        if self.start_time is not None:

            uptime = (
                time.monotonic()
                - self.start_time
            )

        return {
            "running": self.running,
            "width": self.width,
            "height": self.height,
            "frame": self.frame_number,
            "uptime": round(
                uptime,
                2
            )
        }


    # ========================================================
    # HTTP SERVER
    # ========================================================

    def _server_loop(self):

        camera_instance = self

        class Handler(
            BaseHTTPRequestHandler
        ):

            # ==================================================
            # GET
            # ==================================================

            def do_GET(self):

                # ----------------------------------------------
                # MAIN PAGE
                # ----------------------------------------------

                if self.path == "/":

                    self.send_response(
                        200
                    )

                    self.send_header(
                        "Content-Type",
                        "text/html"
                    )

                    self.end_headers()

                    page = f"""
<!DOCTYPE html>

<html>

<head>

<title>Robot Camera</title>

<style>

body {{
    background: #111;
    color: white;
    font-family: Arial;
    text-align: center;
    margin: 0;
    padding: 20px;
}}

.container {{
    display: flex;
    flex-wrap: wrap;
    justify-content: center;
    gap: 20px;
}}

.camera {{
    display: flex;
    flex-direction: column;
    align-items: center;
}}

img {{
    width: {camera_instance.width * 2}px;
    max-width: 95vw;
    height: auto;
}}

h1 {{
    margin-bottom: 10px;
}}

h2 {{
    margin-bottom: 5px;
}}

</style>

</head>

<body>

<h1>Robot Camera</h1>

<div class="container">

<div class="camera">

<h2>Live</h2>

<img src="/stream.mjpg">

</div>

<div class="camera">

<h2>Processed</h2>

<img src="/processed.mjpg">

</div>

</div>

</body>

</html>
"""

                    self.wfile.write(
                        page.encode()
                    )

                    return


                # ----------------------------------------------
                # LIVE STREAM
                # ----------------------------------------------

                if self.path == "/stream.mjpg":

                    self._stream(
                        processed=False
                    )

                    return


                # ----------------------------------------------
                # PROCESSED STREAM
                # ----------------------------------------------

                if self.path == "/processed.mjpg":

                    self._stream(
                        processed=True
                    )

                    return


                # ----------------------------------------------
                # STATUS
                # ----------------------------------------------

                if self.path == "/status":

                    status = (
                        camera_instance
                        .get_status()
                    )

                    data = json.dumps(
                        status
                    ).encode()

                    self.send_response(
                        200
                    )

                    self.send_header(
                        "Content-Type",
                        "application/json"
                    )

                    self.send_header(
                        "Content-Length",
                        str(len(data))
                    )

                    self.end_headers()

                    self.wfile.write(
                        data
                    )

                    return


                # ----------------------------------------------
                # 404
                # ----------------------------------------------

                self.send_error(
                    404
                )


            # ==================================================
            # STREAM
            # ==================================================

            def _stream(
                self,
                processed=False
            ):

                self.send_response(
                    200
                )

                self.send_header(
                    "Cache-Control",
                    "no-cache"
                )

                self.send_header(
                    "Pragma",
                    "no-cache"
                )

                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame"
                )

                self.end_headers()

                try:

                    while camera_instance.running:

                        with camera_instance.jpeg_lock:

                            if processed:

                                jpeg = (
                                    camera_instance
                                    .latest_processed_jpeg
                                )

                            else:

                                jpeg = (
                                    camera_instance
                                    .latest_jpeg
                                )

                        if jpeg is None:

                            time.sleep(
                                0.01
                            )

                            continue

                        self.wfile.write(
                            b"--frame\r\n"
                        )

                        self.wfile.write(
                            b"Content-Type: image/jpeg\r\n"
                        )

                        self.wfile.write(
                            f"Content-Length: "
                            f"{len(jpeg)}\r\n\r\n"
                            .encode()
                        )

                        self.wfile.write(
                            jpeg
                        )

                        self.wfile.write(
                            b"\r\n"
                        )

                        self.wfile.flush()

                        time.sleep(
                            0.03
                        )

                except (
                    BrokenPipeError,
                    ConnectionResetError
                ):

                    pass


            # ==================================================
            # LOG
            # ==================================================

            def log_message(
                self,
                format,
                *args
            ):

                return


        try:

            self.server = HTTPServer(
                (
                    self.host,
                    self.port
                ),
                Handler
            )

            print(
                f"HTTP camera server listening "
                f"on port {self.port}"
            )

            self.server.serve_forever()

        except Exception as e:

            if self.running:

                print(
                    "Camera server error:",
                    e
                )


# ============================================================
# GLOBAL CAMERA INSTANCE
# ============================================================

camera = Camera()


# ============================================================
# RUN DIRECTLY
# ============================================================

if __name__ == "__main__":

    try:

        camera.start()

        print(
            "Camera service running."
        )

        print(
            "Press CTRL+C to stop."
        )

        while True:

            time.sleep(1)

    except KeyboardInterrupt:

        pass

    finally:

        camera.stop()


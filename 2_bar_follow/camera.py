import io
import threading
import time

import cv2
import numpy as np
from PIL import Image
from picamera2 import Picamera2
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


# ============================================================
# CONFIGURATION
# ============================================================

WIDTH = 800
HEIGHT = 600

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8000

JPEG_QUALITY = 75
STREAM_DELAY = 0.03


# ============================================================
# CAMERA DRIVER
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

        self.running = False

        self.latest_frame = None
        self.latest_processed = None

        self.frame_lock = threading.Lock()
        self.processed_lock = threading.Lock()

        self.picam2 = None

        self.capture_thread = None
        self.server_thread = None
        self.http_server = None


    # ========================================================
    # START CAMERA
    # ========================================================

    def start(self):

        if self.running:
            return

        print()
        print("========================================")
        print("       ARMROBOTICS CAMERA SERVER")
        print("========================================")
        print()

        print("Initializing Picamera2...")

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

        print(
            f"Camera resolution: "
            f"{self.width}x{self.height}"
        )

        print("Starting camera...")

        self.picam2.start()

        time.sleep(1)

        self.running = True

        print("Camera ready.")

        # Start capture thread.

        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            daemon=True
        )

        self.capture_thread.start()

        # Start HTTP server.

        self.server_thread = threading.Thread(
            target=self._server_loop,
            daemon=True
        )

        self.server_thread.start()

        print()
        print("========================================")
        print("CAMERA SERVER STARTED")
        print("Open:")
        print(
            f"http://<PI-IP>:{self.port}/"
        )
        print("========================================")
        print()


    # ========================================================
    # CAMERA CAPTURE LOOP
    # ========================================================

    def _capture_loop(self):

        while self.running:

            try:

                # Picamera2 gives us RGB888 here.

                frame_bgr = self.picam2.capture_array()

                with self.frame_lock:
                    self.latest_frame = frame_bgr.copy()


            except Exception as e:

                print(
                    "Camera capture error:",
                    e
                )

                time.sleep(0.1)


    # ========================================================
    # GET FRAME
    # ========================================================

    def get_frame(self):

        """
        Return latest camera frame.

        FORMAT:
            OpenCV BGR uint8 NumPy array.

        Returns:
            numpy.ndarray or None
        """

        with self.frame_lock:

            if self.latest_frame is None:
                return None

            return self.latest_frame.copy()


    # ========================================================
    # SET PROCESSED FRAME
    # ========================================================

    def set_processed_frame(
        self,
        frame
    ):

        """
        Publish an OpenCV image to the processed stream.

        Input may be:

            BGR color image
            grayscale image
            binary image

        It is copied so the caller can safely
        continue processing.
        """

        if frame is None:
            return

        with self.processed_lock:

            self.latest_processed = frame.copy()


    # ========================================================
    # GET PROCESSED FRAME
    # ========================================================

    def get_processed_frame(self):

        with self.processed_lock:

            if self.latest_processed is None:
                return None

            return self.latest_processed.copy()


    # ========================================================
    # JPEG ENCODING
    # ========================================================

    def _encode_jpeg(
        self,
        frame
    ):

        if frame is None:
            return None

        try:

            # ------------------------------------------------
            # Grayscale / binary image
            # ------------------------------------------------

            if len(frame.shape) == 2:

                success, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [
                        cv2.IMWRITE_JPEG_QUALITY,
                        JPEG_QUALITY
                    ]
                )

            # ------------------------------------------------
            # BGR image
            # ------------------------------------------------

            else:

                success, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [
                        cv2.IMWRITE_JPEG_QUALITY,
                        JPEG_QUALITY
                    ]
                )

            if not success:
                return None

            return encoded.tobytes()

        except Exception as e:

            print(
                "JPEG encoding error:",
                e
            )

            return None


    # ========================================================
    # HTTP SERVER
    # ========================================================

    def _server_loop(self):

        camera = self


        class Handler(
            BaseHTTPRequestHandler
        ):

            # ------------------------------------------------
            # ROOT PAGE
            # ------------------------------------------------

            def do_GET(self):

                path = self.path.split("?")[0]

                # --------------------------------------------
                # MAIN PAGE
                # --------------------------------------------

                if path == "/":

                    page = """
<!DOCTYPE html>

<html>

<head>

<meta charset="UTF-8">

<title>ArmRobotics Camera</title>

<style>

body {
    background: #111;
    color: white;
    font-family: Arial, sans-serif;
    text-align: center;
    margin: 0;
    padding: 20px;
}

.container {
    max-width: 1200px;
    margin: auto;
}

h1 {
    margin-bottom: 25px;
}

.camera {
    display: inline-block;
    margin: 10px;
    vertical-align: top;
}

.camera h2 {
    margin: 5px;
}

img {
    width: 800px;
    max-width: 95vw;
    height: auto;
    border: 2px solid #444;
}

</style>

</head>

<body>

<div class="container">

<h1>ArmRobotics Camera</h1>

<div class="camera">

<h2>Camera</h2>

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

                    self.send_response(200)

                    self.send_header(
                        "Content-Type",
                        "text/html; charset=utf-8"
                    )

                    self.send_header(
                        "Cache-Control",
                        "no-cache"
                    )

                    self.end_headers()

                    self.wfile.write(
                        page.encode("utf-8")
                    )

                    return


                # --------------------------------------------
                # RAW STREAM ALIASES
                # --------------------------------------------

                if path in (
                    "/stream",
                    "/stream.mjpg"
                ):

                    self._stream(
                        processed=False
                    )

                    return


                # --------------------------------------------
                # PROCESSED STREAM ALIASES
                # --------------------------------------------

                if path in (
                    "/processed",
                    "/processed.mjpg"
                ):

                    self._stream(
                        processed=True
                    )

                    return


                # --------------------------------------------
                # 404
                # --------------------------------------------

                self.send_error(
                    404,
                    "Not Found"
                )


            # ------------------------------------------------
            # MJPEG STREAM
            # ------------------------------------------------

            def _stream(
                self,
                processed=False
            ):

                self.send_response(200)

                self.send_header(
                    "Cache-Control",
                    "no-cache"
                )

                self.send_header(
                    "Pragma",
                    "no-cache"
                )

                self.send_header(
                    "Connection",
                    "close"
                )

                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame"
                )

                self.end_headers()

                try:

                    while camera.running:

                        if processed:

                            frame = (
                                camera.get_processed_frame()
                            )

                            # If processing hasn't published
                            # anything yet, display a black image.

                            if frame is None:

                                frame = np.zeros(
                                    (
                                        camera.height,
                                        camera.width
                                    ),
                                    dtype=np.uint8
                                )

                        else:

                            frame = (
                                camera.get_frame()
                            )

                        if frame is None:

                            time.sleep(0.01)

                            continue


                        # ------------------------------------
                        # JPEG
                        # ------------------------------------

                        jpeg = camera._encode_jpeg(
                            frame
                        )

                        if jpeg is None:

                            time.sleep(0.01)

                            continue


                        # ------------------------------------
                        # SEND FRAME
                        # ------------------------------------

                        self.wfile.write(
                            b"--frame\r\n"
                        )

                        self.wfile.write(
                            b"Content-Type: image/jpeg\r\n"
                        )

                        self.wfile.write(
                            (
                                f"Content-Length: "
                                f"{len(jpeg)}\r\n\r\n"
                            ).encode()
                        )

                        self.wfile.write(
                            jpeg
                        )

                        self.wfile.write(
                            b"\r\n"
                        )

                        self.wfile.flush()

                        time.sleep(
                            STREAM_DELAY
                        )


                except (
                    BrokenPipeError,
                    ConnectionResetError,
                    ConnectionAbortedError
                ):

                    pass

                except Exception as e:

                    print(
                        "Stream error:",
                        e
                    )


            def log_message(
                self,
                format,
                *args
            ):

                return


        try:

            self.http_server = ThreadingHTTPServer(
                (
                    self.host,
                    self.port
                ),
                Handler
            )

            self.http_server.daemon_threads = True

            self.http_server.serve_forever()


        except Exception as e:

            print(
                "HTTP server error:",
                e
            )


    # ========================================================
    # STOP
    # ========================================================

    def stop(self):

        if not self.running:
            return

        print(
            "\nStopping camera..."
        )

        self.running = False

        if self.http_server is not None:

            try:

                self.http_server.shutdown()

            except Exception:
                pass

            try:

                self.http_server.server_close()

            except Exception:
                pass


        if self.picam2 is not None:

            try:

                self.picam2.stop()

            except Exception:
                pass


        print(
            "Camera stopped."
        )


# ============================================================
# STANDALONE MODE
# ============================================================

if __name__ == "__main__":

    camera = Camera()

    try:

        camera.start()

        while True:

            time.sleep(1)

    except KeyboardInterrupt:

        pass

    finally:

        camera.stop()


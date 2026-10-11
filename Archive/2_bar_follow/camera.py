import socket
import threading
import time

import cv2
import numpy as np
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

# Small delay prevents the server from hammering the network.
STREAM_DELAY = 0.01

# Don't allow a stuck network connection to block forever.
SOCKET_TIMEOUT = 2.0


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

        # Latest-frame buffers.
        # These are replaced, never queued.
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

        # Capture thread
        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            daemon=True
        )

        self.capture_thread.start()

        # HTTP server thread
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

                # capture_array() already returns a new NumPy
                # array, so no additional copy is necessary.

                frame = self.picam2.capture_array()

                with self.frame_lock:
                    self.latest_frame = frame

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

        with self.frame_lock:
            return self.latest_frame


    # ========================================================
    # SET PROCESSED FRAME
    # ========================================================

    def set_processed_frame(self, frame):

        if frame is None:
            return

        with self.processed_lock:
            self.latest_processed = frame


    # ========================================================
    # GET PROCESSED FRAME
    # ========================================================

    def get_processed_frame(self):

        with self.processed_lock:
            return self.latest_processed


    # ========================================================
    # JPEG ENCODING
    # ========================================================

    def _encode_jpeg(self, frame):

        if frame is None:
            return None

        try:

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

        class Handler(BaseHTTPRequestHandler):

            # Prevent unnecessary buffering.
            wbufsize = 0

            # ------------------------------------------------
            # ROOT PAGE
            # ------------------------------------------------

            def do_GET(self):

                path = self.path.split("?")[0]

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
                        "no-cache, no-store, must-revalidate"
                    )

                    self.send_header(
                        "Pragma",
                        "no-cache"
                    )

                    self.end_headers()

                    self.wfile.write(
                        page.encode("utf-8")
                    )

                    return


                # Raw stream
                if path in (
                    "/stream",
                    "/stream.mjpg"
                ):

                    self._stream(processed=False)
                    return


                # Processed stream
                if path in (
                    "/processed",
                    "/processed.mjpg"
                ):

                    self._stream(processed=True)
                    return


                self.send_error(
                    404,
                    "Not Found"
                )


            # ------------------------------------------------
            # MJPEG STREAM
            # ------------------------------------------------

            def _stream(self, processed=False):

                self.send_response(200)

                self.send_header(
                    "Cache-Control",
                    "no-cache, no-store, must-revalidate"
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

                # Configure the actual TCP socket.
                try:

                    self.connection.setsockopt(
                        socket.IPPROTO_TCP,
                        socket.TCP_NODELAY,
                        1
                    )

                    self.connection.settimeout(
                        SOCKET_TIMEOUT
                    )

                except Exception:
                    pass


                # Empty processed frame.
                empty_frame = np.zeros(
                    (
                        camera.height,
                        camera.width,
                        3
                    ),
                    dtype=np.uint8
                )

                try:

                    while camera.running:

                        # ------------------------------------
                        # GET LATEST FRAME
                        # ------------------------------------

                        if processed:

                            frame = (
                                camera.get_processed_frame()
                            )

                            if frame is None:
                                frame = empty_frame

                        else:

                            frame = camera.get_frame()


                        if frame is None:

                            time.sleep(0.005)
                            continue


                        # ------------------------------------
                        # JPEG ENCODE
                        # ------------------------------------

                        jpeg = camera._encode_jpeg(frame)

                        if jpeg is None:

                            time.sleep(0.005)
                            continue


                        # ------------------------------------
                        # BUILD COMPLETE MJPEG PACKET
                        # ------------------------------------

                        header = (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            +
                            (
                                f"Content-Length: "
                                f"{len(jpeg)}\r\n\r\n"
                            ).encode("ascii")
                        )

                        packet = (
                            header
                            + jpeg
                            + b"\r\n"
                        )


                        # ------------------------------------
                        # SEND COMPLETE FRAME
                        # ------------------------------------

                        self.wfile.write(packet)
                        self.wfile.flush()


                        # ------------------------------------
                        # SMALL NETWORK-FRIENDLY DELAY
                        # ------------------------------------

                        time.sleep(STREAM_DELAY)


                except (
                    BrokenPipeError,
                    ConnectionResetError,
                    ConnectionAbortedError,
                    TimeoutError,
                    socket.timeout
                ):

                    # Client disconnected or stopped receiving.
                    pass

                except Exception as e:

                    print(
                        "Stream error:",
                        e
                    )


            # ------------------------------------------------
            # DISABLE HTTP LOGGING
            # ------------------------------------------------

            def log_message(
                self,
                format,
                *args
            ):

                pass


        try:

            self.http_server = ThreadingHTTPServer(
                (
                    self.host,
                    self.port
                ),
                Handler
            )

            # Allows quick restart after stopping the server.
            self.http_server.allow_reuse_address = True

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

        # Stop HTTP server.
        if self.http_server is not None:

            try:
                self.http_server.shutdown()
            except Exception:
                pass

            try:
                self.http_server.server_close()
            except Exception:
                pass


        # Stop camera.
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

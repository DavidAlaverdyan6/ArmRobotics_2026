"""
ArmRobotics Camera Server
=========================

One shared camera node for the robot.

FEATURES
--------

1. Raw camera feed:

       http://<PI-IP>:8000/

   Shows the unmodified camera image.


2. OpenCV processed feed:

       http://<PI-IP>:8000/processed

   Shows whatever frame OpenCV publishes through:

       set_processed_frame(frame)


3. OpenCV access:

       from camera_server import get_frame

       frame = get_frame()

   Returns a BGR NumPy array suitable for OpenCV.


4. Publish OpenCV result:

       from camera_server import set_processed_frame

       set_processed_frame(frame)


5. One Picamera2 instance is used.

The camera is NOT opened again every time get_frame()
is called.
"""

import io
import time
import threading
import socketserver

import cv2
import numpy as np

from http import server

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput


# ============================================================
# CONFIGURATION
# ============================================================

CAMERA_WIDTH = 800
CAMERA_HEIGHT = 600

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8000

JPEG_QUALITY = 80

FRAME_TIMEOUT = 2.0


# ============================================================
# CAMERA
# ============================================================

print()
print("========================================")
print("       ARMROBOTICS CAMERA SERVER")
print("========================================")
print()

print("Initializing Picamera2...")


picam2 = Picamera2()


camera_config = picam2.create_video_configuration(
    main={
        "size": (
            CAMERA_WIDTH,
            CAMERA_HEIGHT
        )
    }
)


picam2.configure(
    camera_config
)


# ============================================================
# RAW CAMERA STREAM
# ============================================================

class CameraOutput(io.BufferedIOBase):

    def __init__(self):

        super().__init__()

        self.frame = None

        self.frame_number = 0

        self.condition = threading.Condition()


    def write(self, buf):

        with self.condition:

            self.frame = bytes(buf)

            self.frame_number += 1

            self.condition.notify_all()

        return len(buf)


raw_output = CameraOutput()


# ============================================================
# PROCESSED OPENCV FRAME
# ============================================================

processed_frame = None

processed_jpeg = None

processed_frame_number = 0

processed_condition = threading.Condition()


# ============================================================
# CAMERA STATE
# ============================================================

camera_started = False

camera_lock = threading.Lock()


# ============================================================
# START CAMERA
# ============================================================

def start_camera():

    global camera_started

    with camera_lock:

        if camera_started:

            return


        print(
            f"Camera resolution: "
            f"{CAMERA_WIDTH}x{CAMERA_HEIGHT}"
        )

        print("Starting camera...")


        picam2.start_recording(
            MJPEGEncoder()
            ,
            FileOutput(
                raw_output
            )
        )


        time.sleep(
            1
        )


        camera_started = True


        print("Camera ready.")
        print()


# ============================================================
# GET RAW JPEG
# ============================================================

def get_raw_jpeg():

    """
    Return the latest raw camera JPEG.

    Returns:
        bytes
    """

    if not camera_started:

        start_camera()


    with raw_output.condition:

        if raw_output.frame is None:

            if not raw_output.condition.wait(
                timeout=FRAME_TIMEOUT
            ):

                raise TimeoutError(
                    "Timed out waiting for camera frame."
                )


        frame = raw_output.frame


    if frame is None:

        raise RuntimeError(
            "Camera frame unavailable."
        )


    return frame


# ============================================================
# GET RAW OPENCV FRAME
# ============================================================

def get_frame():

    """
    Return the latest camera frame.

    The returned image is:

        NumPy ndarray
        BGR
        OpenCV compatible

    Example:

        frame = get_frame()

        cv2.line(
            frame,
            (0, 300),
            (800, 300),
            (0, 255, 0),
            2
        )
    """

    jpeg = get_raw_jpeg()


    array = np.frombuffer(
        jpeg,
        dtype=np.uint8
    )


    frame = cv2.imdecode(
        array,
        cv2.IMREAD_COLOR
    )


    if frame is None:

        raise RuntimeError(
            "OpenCV could not decode camera frame."
        )


    return frame


# ============================================================
# GET FRAME COPY
# ============================================================

def get_frame_copy():

    """
    Return an independent copy of the camera frame.
    """

    return get_frame().copy()


# ============================================================
# SET PROCESSED FRAME
# ============================================================

def set_processed_frame(frame):

    """
    Publish an OpenCV frame to the website.

    The frame must be a NumPy image.

    Normally this is a BGR OpenCV image.

    Example:

        frame = get_frame()

        cv2.line(
            frame,
            (0, 300),
            (800, 300),
            (0, 255, 0),
            2
        )

        set_processed_frame(frame)
    """

    global processed_frame
    global processed_jpeg
    global processed_frame_number


    if frame is None:

        return


    if not isinstance(
        frame,
        np.ndarray
    ):

        raise TypeError(
            "Processed frame must be a NumPy array."
        )


    if frame.ndim != 3:

        raise ValueError(
            "Processed frame must be a 3-channel image."
        )


    # --------------------------------------------------------
    # Make a copy.
    #
    # This is important because the caller may continue
    # modifying its OpenCV frame after this function returns.
    # --------------------------------------------------------

    frame_copy = frame.copy()


    # --------------------------------------------------------
    # Make sure it is BGR.
    #
    # Normally it already is because get_frame() returns BGR.
    # --------------------------------------------------------

    if frame_copy.shape[2] == 4:

        frame_copy = cv2.cvtColor(
            frame_copy,
            cv2.COLOR_BGRA2BGR
        )


    # --------------------------------------------------------
    # Encode for browser.
    # --------------------------------------------------------

    success, encoded = cv2.imencode(
        ".jpg",
        frame_copy,
        [
            cv2.IMWRITE_JPEG_QUALITY,
            JPEG_QUALITY
        ]
    )


    if not success:

        return


    jpeg = encoded.tobytes()


    # --------------------------------------------------------
    # Store both image and JPEG.
    # --------------------------------------------------------

    with processed_condition:

        processed_frame = frame_copy

        processed_jpeg = jpeg

        processed_frame_number += 1

        processed_condition.notify_all()


# ============================================================
# GET PROCESSED FRAME
# ============================================================

def get_processed_frame():

    """
    Return the latest OpenCV processed frame.

    Returns:
        NumPy BGR image

    Returns None if OpenCV has not published a frame yet.
    """

    with processed_condition:

        if processed_frame is None:

            return None

        return processed_frame.copy()


# ============================================================
# GET PROCESSED JPEG
# ============================================================

def get_processed_jpeg():

    """
    Return the latest processed JPEG.

    Returns None until OpenCV publishes its first frame.
    """

    with processed_condition:

        if processed_jpeg is None:

            return None

        return processed_jpeg


# ============================================================
# HTML PAGE
# ============================================================

PAGE = f"""
<!DOCTYPE html>

<html>

<head>

<meta charset="UTF-8">

<meta name="viewport"
      content="width=device-width, initial-scale=1">

<title>ArmRobotics Camera</title>

<style>

body {{

    background: #111;

    color: white;

    font-family: Arial, sans-serif;

    margin: 0;

    padding: 20px;

    text-align: center;

}}

h1 {{

    margin-top: 5px;

    margin-bottom: 25px;

}}

.container {{

    display: flex;

    justify-content: center;

    align-items: flex-start;

    gap: 25px;

    flex-wrap: wrap;

}}

.camera-box {{

    background: #222;

    padding: 12px;

    border-radius: 10px;

    width: {CAMERA_WIDTH}px;

    max-width: 90vw;

    box-sizing: border-box;

}}

.camera-box h2 {{

    margin-top: 5px;

    margin-bottom: 10px;

    font-size: 20px;

}}

.camera-box img {{

    width: 100%;

    height: auto;

    display: block;

    border-radius: 5px;

}}

.status {{

    margin-top: 20px;

    color: #aaa;

    font-size: 14px;

}}

</style>

</head>


<body>

<h1>ArmRobotics Camera</h1>


<div class="container">


    <div class="camera-box">

        <h2>Raw Camera</h2>

        <img
            src="/stream.mjpg"
            alt="Raw camera feed"
        >

    </div>


    <div class="camera-box">

        <h2>OpenCV Processed</h2>

        <img
            src="/processed.mjpg"
            alt="OpenCV processed feed"
        >

    </div>


</div>


<div class="status">

    Camera: {CAMERA_WIDTH}x{CAMERA_HEIGHT}

    &nbsp; | &nbsp;

    Port: {SERVER_PORT}

</div>


</body>

</html>
"""


# ============================================================
# HTTP HANDLER
# ============================================================

class StreamingHandler(
    server.BaseHTTPRequestHandler
):


    # ========================================================
    # GET
    # ========================================================

    def do_GET(self):


        # ----------------------------------------------------
        # MAIN PAGE
        # ----------------------------------------------------

        if self.path == "/":

            self.send_response(
                200
            )

            self.send_header(
                "Content-Type",
                "text/html; charset=utf-8"
            )

            self.send_header(
                "Cache-Control",
                "no-cache"
            )

            self.end_headers()


            try:

                self.wfile.write(
                    PAGE.encode(
                        "utf-8"
                    )
                )

            except (
                BrokenPipeError,
                ConnectionResetError
            ):

                pass


            return


        # ----------------------------------------------------
        # RAW CAMERA STREAM
        # ----------------------------------------------------

        if self.path == "/stream.mjpg":

            self.stream_raw()

            return


        # ----------------------------------------------------
        # PROCESSED STREAM
        # ----------------------------------------------------

        if self.path == "/processed.mjpg":

            self.stream_processed()

            return


        # ----------------------------------------------------
        # INVALID URL
        # ----------------------------------------------------

        self.send_error(
            404
        )


    # ========================================================
    # RAW STREAM
    # ========================================================

    def stream_raw(self):

        self.send_response(
            200
        )

        self.send_header(
            "Cache-Control",
            "no-cache, private"
        )

        self.send_header(
            "Pragma",
            "no-cache"
        )

        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=FRAME"
        )

        self.end_headers()


        last_frame = -1


        try:

            while camera_started:

                with raw_output.condition:

                    while (
                        raw_output.frame_number
                        == last_frame
                    ):

                        raw_output.condition.wait(
                            timeout=1
                        )


                    frame = raw_output.frame

                    number = (
                        raw_output.frame_number
                    )


                if frame is None:

                    continue


                last_frame = number


                self.wfile.write(
                    b"--FRAME\r\n"
                )

                self.wfile.write(
                    b"Content-Type: image/jpeg\r\n"
                )

                self.wfile.write(
                    (
                        f"Content-Length: "
                        f"{len(frame)}\r\n\r\n"
                    ).encode()
                )

                self.wfile.write(
                    frame
                )

                self.wfile.write(
                    b"\r\n"
                )

                self.wfile.flush()


        except (
            BrokenPipeError,
            ConnectionResetError
        ):

            pass


    # ========================================================
    # PROCESSED STREAM
    # ========================================================

    def stream_processed(self):

        self.send_response(
            200
        )

        self.send_header(
            "Cache-Control",
            "no-cache, private"
        )

        self.send_header(
            "Pragma",
            "no-cache"
        )

        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=FRAME"
        )

        self.end_headers()


        last_frame = -1


        try:

            while camera_started:

                with processed_condition:

                    while (
                        processed_frame_number
                        == last_frame
                    ):

                        processed_condition.wait(
                            timeout=1
                        )


                    frame = processed_jpeg

                    number = (
                        processed_frame_number
                    )


                # ------------------------------------------------
                # OpenCV hasn't published anything yet.
                # ------------------------------------------------

                if frame is None:

                    # Send a simple black frame instead of
                    # leaving the browser waiting forever.

                    placeholder = np.zeros(
                        (
                            CAMERA_HEIGHT,
                            CAMERA_WIDTH,
                            3
                        ),
                        dtype=np.uint8
                    )


                    cv2.putText(
                        placeholder,
                        "Waiting for OpenCV...",
                        (
                            180,
                            CAMERA_HEIGHT // 2
                        ),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA
                    )


                    success, encoded = cv2.imencode(
                        ".jpg",
                        placeholder,
                        [
                            cv2.IMWRITE_JPEG_QUALITY,
                            JPEG_QUALITY
                        ]
                    )


                    if not success:

                        continue


                    frame = encoded.tobytes()

                    number = 0


                if number != 0:

                    last_frame = number


                self.wfile.write(
                    b"--FRAME\r\n"
                )

                self.wfile.write(
                    b"Content-Type: image/jpeg\r\n"
                )

                self.wfile.write(
                    (
                        f"Content-Length: "
                        f"{len(frame)}\r\n\r\n"
                    ).encode()
                )

                self.wfile.write(
                    frame
                )

                self.wfile.write(
                    b"\r\n"
                )

                self.wfile.flush()


        except (
            BrokenPipeError,
            ConnectionResetError
        ):

            pass


    # ========================================================
    # DISABLE HTTP LOGGING
    # ========================================================

    def log_message(
        self,
        format,
        *args
    ):

        return


# ============================================================
# THREADED HTTP SERVER
# ============================================================

class StreamingServer(
    socketserver.ThreadingMixIn,
    server.HTTPServer
):

    allow_reuse_address = True

    daemon_threads = True


# ============================================================
# SERVER STATE
# ============================================================

http_server = None

server_thread = None

server_lock = threading.Lock()


# ============================================================
# START WEB SERVER
# ============================================================

def start_server():

    """
    Start the camera and HTTP server.

    Safe to call multiple times.
    """

    global http_server
    global server_thread


    with server_lock:

        if (
            server_thread is not None
            and server_thread.is_alive()
        ):

            return


        start_camera()


        http_server = StreamingServer(
            (
                SERVER_HOST,
                SERVER_PORT
            ),
            StreamingHandler
        )


        server_thread = threading.Thread(
            target=http_server.serve_forever,
            daemon=True
        )


        server_thread.start()


        print(
            "========================================"
        )

        print(
            "CAMERA SERVER STARTED"
        )

        print(
            f"Resolution: "
            f"{CAMERA_WIDTH}x{CAMERA_HEIGHT}"
        )

        print()

        print(
            "Website:"
        )

        print(
            f"http://<PI-IP>:{SERVER_PORT}"
        )

        print()

        print(
            "Raw stream:"
        )

        print(
            f"http://<PI-IP>:{SERVER_PORT}/stream.mjpg"
        )

        print()

        print(
            "OpenCV stream:"
        )

        print(
            f"http://<PI-IP>:{SERVER_PORT}/processed.mjpg"
        )

        print(
            "========================================"
        )

        print()


# ============================================================
# STOP SERVER
# ============================================================

def stop_server():

    """
    Stop the HTTP server.
    """

    global http_server
    global server_thread


    with server_lock:

        if http_server is not None:

            try:

                http_server.shutdown()

            except Exception:

                pass


            try:

                http_server.server_close()

            except Exception:

                pass


            http_server = None


        server_thread = None


# ============================================================
# STOP CAMERA
# ============================================================

def stop_camera():

    """
    Safely stop HTTP server and Picamera2.
    """

    global camera_started


    stop_server()


    with camera_lock:

        if camera_started:

            try:

                picam2.stop_recording()

            except Exception:

                pass


            camera_started = False


    print(
        "Camera stopped."
    )


# ============================================================
# CAMERA STATUS
# ============================================================

def camera_running():

    return camera_started


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    try:

        start_server()


        print(
            "Press CTRL+C to stop."
        )


        while True:

            time.sleep(
                1
            )


    except KeyboardInterrupt:

        print()
        print(
            "Stopping camera server..."
        )


    finally:

        stop_camera()

        print(
            "Camera server stopped safely."
        )
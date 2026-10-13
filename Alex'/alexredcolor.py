#!/usr/bin/env python3

import cv2
import numpy as np
import threading
import time
import sys

from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn


# ============================================================
# CONFIGURATION
# ============================================================

CAMERA_DEVICE = 0

WIDTH = 640
HEIGHT = 480
FPS = 120

SERVER_PORT = 8000

# Ignore objects smaller than this
MIN_COLOR_AREA = 300


# ============================================================
# CAMERA ORIENTATION
# ============================================================

# True = rotate camera image 180 degrees
ROTATE_180 = True


# ============================================================
# COLOR DEFINITIONS
# ============================================================
#
# OpenCV HSV:
# H = 0..179
# S = 0..255
# V = 0..255
#
# Red requires two ranges because it wraps around
# the HSV hue boundary.
# ============================================================


COLOR_RANGES = {

    "RED": [
        (
            np.array([0, 100, 70]),
            np.array([10, 255, 255])
        ),
        (
            np.array([170, 100, 70]),
            np.array([179, 255, 255])
        )
    ],

    "YELLOW": [
        (
            np.array([20, 100, 70]),
            np.array([35, 255, 255])
        )
    ],

    "BLUE": [
        (
            np.array([90, 100, 50]),
            np.array([135, 255, 255])
        )
    ]
}


# ============================================================
# DRAWING SETTINGS
# ============================================================

# BGR colors used for drawing

DRAW_COLORS = {

    "RED": (
        0,
        0,
        255
    ),

    "YELLOW": (
        0,
        255,
        255
    ),

    "BLUE": (
        255,
        0,
        0
    )
}


# ============================================================
# SHARED JPEG FRAME
# ============================================================

latest_jpeg = None

frame_lock = threading.Lock()


# ============================================================
# LOGGING
# ============================================================

def log(message):

    print(
        message,
        flush=True
    )


# ============================================================
# CAMERA INITIALIZATION
# ============================================================

log("")
log("==========================================")
log("       ARM ROBOTICS COLOR DETECTOR")
log("==========================================")
log("")

log("[1] Python started")

log(
    f"[2] OpenCV version: "
    f"{cv2.__version__}"
)

log(
    f"[3] NumPy version: "
    f"{np.__version__}"
)

log(
    f"[4] Camera device: "
    f"/dev/video{CAMERA_DEVICE}"
)

log("")


# ============================================================
# OPEN CAMERA
# ============================================================

log("[5] Opening USB camera...")

camera = cv2.VideoCapture(
    CAMERA_DEVICE,
    cv2.CAP_V4L2
)

log("[6] VideoCapture returned")


if not camera.isOpened():

    log("")
    log("ERROR: Camera could not be opened.")
    log("")
    log("Check available cameras with:")
    log("")
    log("    ls -l /dev/video*")
    log("")

    sys.exit(1)


log("[7] Camera opened successfully")


# ============================================================
# CAMERA SETTINGS
# ============================================================

camera.set(
    cv2.CAP_PROP_FRAME_WIDTH,
    WIDTH
)

camera.set(
    cv2.CAP_PROP_FRAME_HEIGHT,
    HEIGHT
)

camera.set(
    cv2.CAP_PROP_FPS,
    FPS
)


# Prefer MJPEG for USB cameras

camera.set(
    cv2.CAP_PROP_FOURCC,
    cv2.VideoWriter_fourcc(
        'M',
        'J',
        'P',
        'G'
    )
)


actual_width = int(
    camera.get(
        cv2.CAP_PROP_FRAME_WIDTH
    )
)

actual_height = int(
    camera.get(
        cv2.CAP_PROP_FRAME_HEIGHT
    )
)

actual_fps = camera.get(
    cv2.CAP_PROP_FPS
)


log(
    f"[8] Resolution: "
    f"{actual_width}x{actual_height}"
)

log(
    f"[9] FPS: "
    f"{actual_fps:.1f}"
)


# ============================================================
# FIRST FRAME TEST
# ============================================================

log(
    "[10] Reading first camera frame..."
)

ret, frame = camera.read()


if not ret or frame is None:

    log("")
    log(
        "ERROR: Camera opened, "
        "but no frame was received."
    )

    log("")

    camera.release()

    sys.exit(1)


# Apply orientation to first frame too

if ROTATE_180:

    frame = cv2.rotate(
        frame,
        cv2.ROTATE_180
    )


log(
    f"[11] First frame received: "
    f"{frame.shape}"
)


# ============================================================
# COLOR DETECTION
# ============================================================

def detect_colors(frame):

    # --------------------------------------------------------
    # Convert BGR -> HSV
    # --------------------------------------------------------

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )


    # --------------------------------------------------------
    # Morphology kernel
    # --------------------------------------------------------

    kernel = np.ones(
        (5, 5),
        np.uint8
    )


    # --------------------------------------------------------
    # Process every color
    # --------------------------------------------------------

    for color_name, ranges in COLOR_RANGES.items():

        combined_mask = None


        # ----------------------------------------------------
        # Build mask
        # ----------------------------------------------------

        for lower, upper in ranges:

            current_mask = cv2.inRange(
                hsv,
                lower,
                upper
            )


            if combined_mask is None:

                combined_mask = current_mask

            else:

                combined_mask = cv2.bitwise_or(
                    combined_mask,
                    current_mask
                )


        # ----------------------------------------------------
        # Remove small noise
        # ----------------------------------------------------

        combined_mask = cv2.morphologyEx(
            combined_mask,
            cv2.MORPH_OPEN,
            kernel
        )


        # ----------------------------------------------------
        # Fill small gaps
        # ----------------------------------------------------

        combined_mask = cv2.morphologyEx(
            combined_mask,
            cv2.MORPH_CLOSE,
            kernel
        )


        # ----------------------------------------------------
        # Find contours
        # ----------------------------------------------------

        contours, _ = cv2.findContours(
            combined_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )


        # ----------------------------------------------------
        # Find largest object of this color
        # ----------------------------------------------------

        largest_contour = None

        largest_area = 0


        for contour in contours:

            area = cv2.contourArea(
                contour
            )


            if area < MIN_COLOR_AREA:

                continue


            if area > largest_area:

                largest_area = area

                largest_contour = contour


        # ----------------------------------------------------
        # Nothing detected
        # ----------------------------------------------------

        if largest_contour is None:

            continue


        # ====================================================
        # OBJECT DETECTED
        # ====================================================

        draw_color = DRAW_COLORS[
            color_name
        ]


        # ----------------------------------------------------
        # Draw contour
        # ----------------------------------------------------

        cv2.drawContours(
            frame,
            [largest_contour],
            -1,
            draw_color,
            3
        )


        # ----------------------------------------------------
        # Bounding box
        # ----------------------------------------------------

        x, y, w, h = cv2.boundingRect(
            largest_contour
        )


        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            draw_color,
            2
        )


        # ----------------------------------------------------
        # Calculate centroid
        # ----------------------------------------------------

        moments = cv2.moments(
            largest_contour
        )


        if moments["m00"] == 0:

            continue


        center_x = int(
            moments["m10"] /
            moments["m00"]
        )

        center_y = int(
            moments["m01"] /
            moments["m00"]
        )


        # ====================================================
        # CENTER DOT
        # ====================================================

        cv2.circle(
            frame,
            (
                center_x,
                center_y
            ),
            10,
            draw_color,
            -1
        )


        # White center

        cv2.circle(
            frame,
            (
                center_x,
                center_y
            ),
            3,
            (
                255,
                255,
                255
            ),
            -1
        )


        # ====================================================
        # CROSSHAIR
        # ====================================================

        cv2.line(
            frame,
            (
                center_x - 25,
                center_y
            ),
            (
                center_x + 25,
                center_y
            ),
            draw_color,
            2
        )

        cv2.line(
            frame,
            (
                center_x,
                center_y - 25
            ),
            (
                center_x,
                center_y + 25
            ),
            draw_color,
            2
        )


        # ====================================================
        # LABEL
        # ====================================================

        label = (
            f"{color_name} "
            f"({center_x}, {center_y})"
        )


        # Position label above object

        label_y = max(
            y - 10,
            25
        )


        cv2.putText(
            frame,
            label,
            (
                x,
                label_y
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            draw_color,
            2
        )


        # ====================================================
        # AREA
        # ====================================================

        area_text = (
            f"Area: {int(largest_area)}"
        )


        cv2.putText(
            frame,
            area_text,
            (
                x,
                y + h + 25
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            draw_color,
            2
        )


    # --------------------------------------------------------
    # Return processed image
    # --------------------------------------------------------

    return frame


# ============================================================
# WEB SERVER
# ============================================================

class StreamingHandler(
    BaseHTTPRequestHandler
):


    # --------------------------------------------------------
    # Disable HTTP logging
    # --------------------------------------------------------

    def log_message(
        self,
        format,
        *args
    ):

        pass


    # --------------------------------------------------------
    # HTTP GET
    # --------------------------------------------------------

    def do_GET(self):


        # ====================================================
        # MAIN PAGE
        # ====================================================

        if self.path == "/":

            html = """
<!DOCTYPE html>

<html>

<head>

<meta charset="UTF-8">

<meta name="viewport"
      content="width=device-width, initial-scale=1">

<title>Arm Robotics Color Detector</title>

<style>

body {
    background: #111;
    color: white;
    font-family: Arial, sans-serif;
    text-align: center;
    margin: 0;
    padding: 20px;
}

h1 {
    margin-bottom: 20px;
}

img {
    width: 100%;
    max-width: 1000px;
    height: auto;
    border: 2px solid #444;
}

</style>

</head>

<body>

<h1>Arm Robotics Color Detector</h1>

<img src="/stream">

</body>

</html>
"""


            data = html.encode()


            self.send_response(
                200
            )


            self.send_header(
                "Content-Type",
                "text/html"
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


        # ====================================================
        # MJPEG STREAM
        # ====================================================

        if self.path == "/stream":

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
                "multipart/x-mixed-replace; boundary=frame"
            )


            self.end_headers()


            try:

                while True:

                    # ----------------------------------------
                    # Get latest frame
                    # ----------------------------------------

                    with frame_lock:

                        jpeg = latest_jpeg


                    # ----------------------------------------
                    # Send frame
                    # ----------------------------------------

                    if jpeg is not None:

                        self.wfile.write(
                            b"--frame\r\n"
                        )


                        self.wfile.write(
                            b"Content-Type: "
                            b"image/jpeg\r\n"
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


                    time.sleep(
                        0.033
                    )


            except (
                BrokenPipeError,
                ConnectionResetError
            ):

                pass


            return


        # ====================================================
        # 404
        # ====================================================

        self.send_error(
            404
        )


# ============================================================
# THREADED HTTP SERVER
# ============================================================

class ThreadedHTTPServer(
    ThreadingMixIn,
    HTTPServer
):

    daemon_threads = True


# ============================================================
# START WEB SERVER
# ============================================================

def start_server():

    try:

        server = ThreadedHTTPServer(
            (
                "0.0.0.0",
                SERVER_PORT
            ),
            StreamingHandler
        )


        log("")
        log(
            f"[12] Web server listening on "
            f"0.0.0.0:{SERVER_PORT}"
        )

        log(
            f"[13] Open: "
            f"http://<PI-IP>:{SERVER_PORT}"
        )

        log("")


        server.serve_forever()


    except Exception as e:

        log(
            f"SERVER ERROR: {e}"
        )


server_thread = threading.Thread(
    target=start_server,
    daemon=True
)

server_thread.start()


# ============================================================
# MAIN LOOP
# ============================================================

log(
    "[14] Starting color detection..."
)

log("")

log(
    "=========================================="
)

log(
    "          DETECTOR RUNNING"
)

log(
    "=========================================="
)

log("")

log(
    "Detecting: RED / YELLOW / BLUE"
)

log(
    "Camera rotation: "
    + (
        "180 degrees"
        if ROTATE_180
        else "normal"
    )
)

log("")


frame_count = 0

last_status_time = time.time()


try:

    while True:

        # ----------------------------------------------------
        # Read USB camera
        # ----------------------------------------------------

        ret, frame = camera.read()


        if not ret:

            log(
                "WARNING: Failed to read frame"
            )

            time.sleep(
                0.1
            )

            continue


        # ----------------------------------------------------
        # Rotate image
        # ----------------------------------------------------

        if ROTATE_180:

            frame = cv2.rotate(
                frame,
                cv2.ROTATE_180
            )


        # ----------------------------------------------------
        # Detect colors
        # ----------------------------------------------------

        processed_frame = detect_colors(
            frame
        )


        # ----------------------------------------------------
        # Encode JPEG
        # ----------------------------------------------------

        success, encoded = cv2.imencode(
            ".jpg",
            processed_frame,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                85
            ]
        )


        if success:

            jpeg = encoded.tobytes()


            # ----------------------------------------------
            # Update shared web frame
            # ----------------------------------------------

            with frame_lock:

                latest_jpeg = jpeg


        frame_count += 1


        # ----------------------------------------------------
        # Status once per second
        # ----------------------------------------------------

        now = time.time()


        if (
            now -
            last_status_time
            >= 1.0
        ):

            log(
                f"Running | "
                f"Frames: {frame_count}"
            )

            last_status_time = now


except KeyboardInterrupt:

    log("")
    log("Stopping...")


finally:

    camera.release()

    log(
        "Camera released."
    )

    log(
        "Program stopped."
    )
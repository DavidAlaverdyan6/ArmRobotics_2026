#!/usr/bin/env python3

import io
import logging
import socketserver
import time
from http import server
from threading import Condition, Lock

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput


# ============================================================
# Configuration
# ============================================================

TUNING_FILE = "/usr/share/libcamera/ipa/rpi/vc4/ov5647_dw9714.json"

SERVER_ADDRESS = ("0.0.0.0", 8000)

WIDTH = 800
HEIGHT = 600

# Your DW9714 mapping:
#   0    = infinity
#   512  = ~20 cm
#   1023 = macro
MIN_FOCUS = 0
MAX_FOCUS = 1023


# ============================================================
# Web page
# ============================================================

PAGE = """\
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>OV5647 + DW9714 Camera</title>

    <style>
        body {
            background: #111;
            color: #eee;
            font-family: Arial, sans-serif;
            text-align: center;
            margin: 20px;
        }

        img {
            max-width: 95vw;
            width: 800px;
            height: auto;
            border: 2px solid #444;
        }

        .controls {
            margin: 20px auto;
            max-width: 800px;
            padding: 20px;
            background: #222;
            border-radius: 10px;
        }

        button {
            font-size: 18px;
            padding: 10px 20px;
            margin: 5px;
            cursor: pointer;
        }

        input[type=range] {
            width: 80%;
        }

        #focusValue {
            font-size: 24px;
            font-weight: bold;
        }

        #status {
            margin-top: 15px;
            font-size: 18px;
            color: #aaa;
        }
    </style>
</head>

<body>

<h1>OV5647 + DW9714</h1>

<img src="/stream.mjpg">

<div class="controls">

    <h2>Autofocus</h2>

    <button onclick="singleAF()">
        Single AF
    </button>

    <button onclick="continuousAF()">
        Continuous AF
    </button>

    <button onclick="manualAF()">
        Manual Focus
    </button>

    <h2>Manual Focus</h2>

    <input
        id="focusSlider"
        type="range"
        min="0"
        max="1023"
        value="0"
        oninput="updateFocusLabel(this.value)"
        onchange="setFocus(this.value)"
    >

    <div>
        Focus position:
        <span id="focusValue">0</span>
    </div>

    <div id="status">
        Connecting...
    </div>

</div>

<script>

function updateFocusLabel(value) {
    document.getElementById("focusValue").innerText = value;
}

function setFocus(value) {
    fetch("/focus?value=" + value)
        .then(response => response.text())
        .then(text => {
            document.getElementById("status").innerText = text;
        })
        .catch(err => {
            document.getElementById("status").innerText =
                "Error: " + err;
        });
}

function singleAF() {
    fetch("/af/single")
        .then(response => response.text())
        .then(text => {
            document.getElementById("status").innerText = text;
        });
}

function continuousAF() {
    fetch("/af/continuous")
        .then(response => response.text())
        .then(text => {
            document.getElementById("status").innerText = text;
        });
}

function manualAF() {
    fetch("/af/manual")
        .then(response => response.text())
        .then(text => {
            document.getElementById("status").innerText = text;
        });
}

</script>

</body>
</html>
"""


# ============================================================
# MJPEG output
# ============================================================

class StreamingOutput(io.BufferedIOBase):

    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


# ============================================================
# Camera state
# ============================================================

camera_lock = Lock()


def set_manual_focus(position):
    position = max(MIN_FOCUS, min(MAX_FOCUS, int(position)))

    with camera_lock:
        picam2.set_controls({
            "AfMode": 0,
            "LensPosition": float(position)
        })

    return position


def set_continuous_af():
    with camera_lock:
        picam2.set_controls({
            "AfMode": 2
        })


def trigger_single_af():
    with camera_lock:
        picam2.set_controls({
            "AfMode": 1
        })

        time.sleep(0.05)

        picam2.set_controls({
            "AfTrigger": 1
        })


def get_lens_position():
    try:
        metadata = picam2.capture_metadata()

        if "LensPosition" in metadata:
            return metadata["LensPosition"]

    except Exception:
        pass

    return None


# ============================================================
# HTTP handler
# ============================================================

class StreamingHandler(server.BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        logging.info(
            "%s - %s",
            self.client_address[0],
            format % args
        )

    def send_text(self, text, status=200):

        content = text.encode("utf-8")

        self.send_response(status)
        self.send_header(
            "Content-Type",
            "text/plain; charset=utf-8"
        )
        self.send_header(
            "Content-Length",
            len(content)
        )
        self.end_headers()

        self.wfile.write(content)

    def do_GET(self):

        # ----------------------------------------------------
        # Main page
        # ----------------------------------------------------

        if self.path == "/":

            self.send_response(301)
            self.send_header(
                "Location",
                "/index.html"
            )
            self.end_headers()

            return

        if self.path == "/index.html":

            content = PAGE.encode("utf-8")

            self.send_response(200)
            self.send_header(
                "Content-Type",
                "text/html"
            )
            self.send_header(
                "Content-Length",
                len(content)
            )
            self.end_headers()

            self.wfile.write(content)

            return

        # ----------------------------------------------------
        # MJPEG stream
        # ----------------------------------------------------

        if self.path == "/stream.mjpg":

            self.send_response(200)

            self.send_header("Age", 0)
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

            try:

                while True:

                    with output.condition:

                        output.condition.wait()

                        frame = output.frame

                    if frame is None:
                        continue

                    self.wfile.write(
                        b"--FRAME\r\n"
                    )

                    self.send_header(
                        "Content-Type",
                        "image/jpeg"
                    )

                    self.send_header(
                        "Content-Length",
                        len(frame)
                    )

                    self.end_headers()

                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")

            except Exception as e:

                logging.warning(
                    "Client %s disconnected: %s",
                    self.client_address,
                    e
                )

            return

        # ----------------------------------------------------
        # Manual focus
        # ----------------------------------------------------

        if self.path.startswith("/focus?value="):

            try:

                value = int(
                    self.path.split(
                        "=", 1
                    )[1]
                )

                value = set_manual_focus(value)

                self.send_text(
                    f"Manual focus set to {value}"
                )

            except Exception as e:

                logging.exception(
                    "Manual focus error"
                )

                self.send_text(
                    f"Focus error: {e}",
                    500
                )

            return

        # ----------------------------------------------------
        # Single AF
        # ----------------------------------------------------

        if self.path == "/af/single":

            try:

                trigger_single_af()

                self.send_text(
                    "Single autofocus triggered"
                )

            except Exception as e:

                logging.exception(
                    "Single AF error"
                )

                self.send_text(
                    f"AF error: {e}",
                    500
                )

            return

        # ----------------------------------------------------
        # Continuous AF
        # ----------------------------------------------------

        if self.path == "/af/continuous":

            try:

                set_continuous_af()

                self.send_text(
                    "Continuous autofocus enabled"
                )

            except Exception as e:

                logging.exception(
                    "Continuous AF error"
                )

                self.send_text(
                    f"AF error: {e}",
                    500
                )

            return

        # ----------------------------------------------------
        # Manual AF mode
        # ----------------------------------------------------

        if self.path == "/af/manual":

            try:

                set_manual_focus(0)

                self.send_text(
                    "Manual autofocus mode enabled"
                )

            except Exception as e:

                logging.exception(
                    "Manual AF error"
                )

                self.send_text(
                    f"AF error: {e}",
                    500
                )

            return

        # ----------------------------------------------------
        # Current focus position
        # ----------------------------------------------------

        if self.path == "/focus":

            position = get_lens_position()

            if position is None:

                self.send_text(
                    "LensPosition unavailable",
                    500
                )

            else:

                self.send_text(
                    f"LensPosition={position}"
                )

            return

        # ----------------------------------------------------
        # Not found
        # ----------------------------------------------------

        self.send_error(404)

        self.end_headers()


# ============================================================
# HTTP server
# ============================================================

class StreamingServer(
    socketserver.ThreadingMixIn,
    server.HTTPServer
):

    allow_reuse_address = True
    daemon_threads = True


# ============================================================
# Camera initialization
# ============================================================

print()
print("======================================")
print(" OV5647 + DW9714 Camera Server")
print("======================================")
print()
print("Loading tuning file:")
print(TUNING_FILE)
print()


# IMPORTANT:
#
# The tuning file MUST be loaded before Picamera2()
# is created.
#
# set_tuning_file() does not exist in this Picamera2 version.
#

tuning = Picamera2.load_tuning_file(
    TUNING_FILE
)

picam2 = Picamera2(
    tuning=tuning
)


# ============================================================
# Camera configuration
# ============================================================

config = picam2.create_video_configuration(
    main={
        "size": (WIDTH, HEIGHT),
        "format": "XBGR8888"
    }
)

picam2.configure(config)


# ============================================================
# Start camera
# ============================================================

picam2.start()

time.sleep(1)


# ============================================================
# Enable continuous autofocus
# ============================================================

print("Starting autofocus...")

try:

    picam2.set_controls({
        "AfMode": 2
    })

    print("Autofocus enabled.")

except Exception as e:

    print("WARNING: Could not enable autofocus:")
    print(e)


# ============================================================
# MJPEG encoder
# ============================================================

output = StreamingOutput()

picam2.start_recording(
    MJPEGEncoder(),
    FileOutput(output)
)


# ============================================================
# Start server
# ============================================================

try:

    address = SERVER_ADDRESS

    http_server = StreamingServer(
        address,
        StreamingHandler
    )

    print()
    print("======================================")
    print(" OV5647 + DW9714 Camera Server")
    print(" Autofocus: ENABLED")
    print("======================================")
    print()
    print("Open from another computer:")
    print()
    print("http://<PI-IP>:8000")
    print()
    print("Server running...")
    print()

    http_server.serve_forever()

except KeyboardInterrupt:

    print()
    print("Stopping server...")

finally:

    try:
        picam2.stop_recording()
    except Exception:
        pass

    try:
        picam2.stop()
    except Exception:
        pass

    try:
        picam2.close()
    except Exception:
        pass

    print("Camera stopped.")


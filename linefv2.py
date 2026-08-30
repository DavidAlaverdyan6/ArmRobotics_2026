import time
import threading
import io
import numpy as np

from http.server import BaseHTTPRequestHandler, HTTPServer

from picamera2 import Picamera2
from telemetrix import telemetrix

from pins import PB9, PB8, PB7, PB6


# ============================================================
# CONFIGURATION
# ============================================================

WIDTH = 416
HEIGHT = 416

THRESHOLD = 128


# ============================================================
# NORMAL LINE FOLLOWING
# ============================================================

ROI_TOP = 300
ROI_BOTTOM = 400

MIN_LINE_PIXELS = 20

DEAD_ZONE = 25

LOOP_DELAY = 0.02


# ============================================================
# LINE LOST BEHAVIOR
# ============================================================

# How long the robot reverses when it completely
# loses the line.

LINE_LOST_REVERSE_TIME = 0.25


# ============================================================
# THREE-POINT INTERSECTION DETECTOR
# ============================================================
#
#                  ●
#
#
#       ●                    ●
#
#
# FORWARD / LEFT / RIGHT
#
# These points are ONLY used for intersection detection.
# Normal line following still uses find_line().
# ============================================================

DOT_CENTER_X = WIDTH // 2

DOT_CENTER_Y = 300

# Distance between LEFT and RIGHT points.

DOT_HORIZONTAL_SPACING = 150

# Distance between LEFT/RIGHT and FORWARD.

DOT_VERTICAL_OFFSET = 100

DOT_RADIUS = 5

DOT_BLACK_RATIO = 0.50


# ============================================================
# INTERSECTION CONFIRMATION
# ============================================================

CROSSING_CONFIRM_FRAMES = 3

CROSSING_COOLDOWN = 0.8


# ============================================================
# THREE POINT POSITIONS
# ============================================================

CROSSING_POINTS = {

    "FORWARD": (
        DOT_CENTER_X,
        DOT_CENTER_Y - DOT_VERTICAL_OFFSET
    ),

    "LEFT": (
        DOT_CENTER_X - DOT_HORIZONTAL_SPACING,
        DOT_CENTER_Y
    ),

    "RIGHT": (
        DOT_CENTER_X + DOT_HORIZONTAL_SPACING,
        DOT_CENTER_Y
    )
}


# ============================================================
# L298N PIN ASSIGNMENT
# ============================================================

LEFT_IN1 = PB9
LEFT_IN2 = PB8

RIGHT_IN1 = PB7
RIGHT_IN2 = PB6


# ============================================================
# INTERSECTION TURN CONFIGURATION
# ============================================================

# LEFT and RIGHT intersection turns last 1.5 seconds.

TURN_TIME = 1.5

# Small forward movement after turning.

EXIT_TIME = 0.15


# ============================================================
# CAMERA INITIALIZATION
# ============================================================

print("Starting camera...")

picam2 = Picamera2()

picam2.configure(
    picam2.create_preview_configuration(
        main={
            "size": (
                WIDTH,
                HEIGHT
            )
        }
    )
)

picam2.start()

time.sleep(1)

print("Camera ready")


# ============================================================
# LOCAL MJPEG SERVER
# ============================================================

latest_jpeg = None

jpeg_lock = threading.Lock()

server_running = True


def camera_server_frame():

    global latest_jpeg

    from PIL import Image

    while server_running:

        try:

            frame = picam2.capture_array()

            # XBGR8888 -> RGB

            rgb = frame[:, :, :3][:, :, ::-1]

            image = Image.fromarray(
                rgb
            )

            buffer = io.BytesIO()

            image.save(
                buffer,
                format="JPEG",
                quality=70
            )

            jpeg = buffer.getvalue()

            with jpeg_lock:

                latest_jpeg = jpeg

        except Exception as e:

            print(
                "Camera server error:",
                e
            )

            time.sleep(0.1)


class CameraHandler(
    BaseHTTPRequestHandler
):

    def do_GET(self):

        # ====================================================
        # MAIN PAGE
        # ====================================================

        if self.path == "/":

            self.send_response(200)

            self.send_header(
                "Content-Type",
                "text/html"
            )

            self.end_headers()

            page = """
<!DOCTYPE html>

<html>

<head>

<title>Robot Camera</title>

<style>

body {
    background: #111;
    color: white;
    font-family: Arial;
    text-align: center;
    margin: 0;
    padding: 20px;
}

img {
    width: 832px;
    max-width: 95vw;
    height: auto;
}

</style>

</head>

<body>

<h1>Robot Camera</h1>

<img src="/stream">

</body>

</html>
"""

            self.wfile.write(
                page.encode()
            )

            return


        # ====================================================
        # MJPEG STREAM
        # ====================================================

        if self.path == "/stream":

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
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame"
            )

            self.end_headers()

            try:

                while server_running:

                    with jpeg_lock:

                        jpeg = latest_jpeg

                    if jpeg is None:

                        time.sleep(0.01)

                        continue

                    self.wfile.write(
                        b"--frame\r\n"
                    )

                    self.wfile.write(
                        b"Content-Type: image/jpeg\r\n"
                    )

                    self.wfile.write(
                        f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                    )

                    self.wfile.write(
                        jpeg
                    )

                    self.wfile.write(
                        b"\r\n"
                    )

                    self.wfile.flush()

                    time.sleep(0.03)

            except (
                BrokenPipeError,
                ConnectionResetError
            ):

                pass

            return


        self.send_error(404)


    def log_message(
        self,
        format,
        *args
    ):

        return


def start_web_server():

    server = HTTPServer(
        (
            "0.0.0.0",
            8000
        ),
        CameraHandler
    )

    print()
    print(
        "========================================"
    )
    print(
        "CAMERA SERVER STARTED"
    )
    print(
        "http://<PI-IP>:8000"
    )
    print(
        "========================================"
    )
    print()

    server.serve_forever()


# ============================================================
# START CAMERA SERVER
# ============================================================

camera_thread = threading.Thread(
    target=camera_server_frame,
    daemon=True
)

camera_thread.start()


web_thread = threading.Thread(
    target=start_web_server,
    daemon=True
)

web_thread.start()


# ============================================================
# STM32 INITIALIZATION
# ============================================================

print(
    "Connecting to STM32..."
)

board = telemetrix.Telemetrix(
    com_port="/dev/serial0"
)

print(
    "STM32 connected!"
)


# ============================================================
# MOTOR PIN INITIALIZATION
# ============================================================

board.set_pin_mode_digital_output(
    LEFT_IN1
)

board.set_pin_mode_digital_output(
    LEFT_IN2
)

board.set_pin_mode_digital_output(
    RIGHT_IN1
)

board.set_pin_mode_digital_output(
    RIGHT_IN2
)

print(
    "L298N initialized!"
)


# ============================================================
# MOTOR CONTROL
# ============================================================

def left_forward():

    board.digital_write(
        LEFT_IN1,
        1
    )

    board.digital_write(
        LEFT_IN2,
        0
    )


def left_backward():

    board.digital_write(
        LEFT_IN1,
        0
    )

    board.digital_write(
        LEFT_IN2,
        1
    )


def left_stop():

    board.digital_write(
        LEFT_IN1,
        0
    )

    board.digital_write(
        LEFT_IN2,
        0
    )


def right_forward():

    board.digital_write(
        RIGHT_IN1,
        1
    )

    board.digital_write(
        RIGHT_IN2,
        0
    )


def right_backward():

    board.digital_write(
        RIGHT_IN1,
        0
    )

    board.digital_write(
        RIGHT_IN2,
        1
    )


def right_stop():

    board.digital_write(
        RIGHT_IN1,
        0
    )

    board.digital_write(
        RIGHT_IN2,
        0
    )


def forward():

    left_forward()

    right_forward()


def backward():

    left_backward()

    right_backward()


def stop():

    left_stop()

    right_stop()


# ============================================================
# NORMAL LINE-FOLLOWING TURNING
# ============================================================

def turn_left():

    left_stop()

    right_forward()


def turn_right():

    left_forward()

    right_stop()


# ============================================================
# THRESHOLD IMAGE
# ============================================================

def threshold_image(
    frame
):

    image = frame[:, :, :3]

    gray = np.mean(
        image,
        axis=2
    )

    black = (
        gray < THRESHOLD
    )

    return black


# ============================================================
# NORMAL LINE DETECTION
# ============================================================

def find_line(
    frame
):

    image = frame[:, :, :3]

    gray = np.mean(
        image,
        axis=2
    )

    black = (
        gray < THRESHOLD
    )

    roi = black[
        ROI_TOP:ROI_BOTTOM,
        :
    ]

    y, x = np.where(
        roi
    )

    if len(x) < MIN_LINE_PIXELS:

        return None

    line_x = np.mean(
        x
    )

    return line_x


# ============================================================
# SAMPLE ONE POINT
# ============================================================

def sample_point(
    black,
    x,
    y
):

    x1 = max(
        0,
        x - DOT_RADIUS
    )

    x2 = min(
        WIDTH,
        x + DOT_RADIUS + 1
    )

    y1 = max(
        0,
        y - DOT_RADIUS
    )

    y2 = min(
        HEIGHT,
        y + DOT_RADIUS + 1
    )

    region = black[
        y1:y2,
        x1:x2
    ]

    if region.size == 0:

        return False

    black_pixels = np.count_nonzero(
        region
    )

    black_ratio = (
        black_pixels
        / region.size
    )

    return (
        black_ratio
        >= DOT_BLACK_RATIO
    )


# ============================================================
# READ THREE POINTS
# ============================================================

def read_crossing_points(
    black
):

    states = {}

    for name, (
        x,
        y
    ) in CROSSING_POINTS.items():

        states[name] = sample_point(
            black,
            x,
            y
        )

    return states


# ============================================================
# PRINT THREE-POINT PATTERN
# ============================================================

def print_pattern(
    states
):

    print()

    print(
        "              "
        f"{'●' if states['FORWARD'] else '○'}"
    )

    print()

    print(
        "      "
        f"{'●' if states['LEFT'] else '○'}"
        "                 "
        f"{'●' if states['RIGHT'] else '○'}"
    )

    print()


# ============================================================
# DETECT INTERSECTION
# ============================================================

def is_intersection(
    states
):

    left = states["LEFT"]

    right = states["RIGHT"]

    forward_point = states["FORWARD"]

    detected = (
        int(left)
        + int(right)
        + int(forward_point)
    )

    return (
        detected >= 2
    )


# ============================================================
# ASK USER FOR DIRECTION
# ============================================================

def ask_direction():

    while True:

        direction = input(
            "Choose direction [L/R/F]: "
        ).strip().upper()

        if direction in (
            "L",
            "R",
            "F"
        ):

            return direction

        print(
            "Invalid direction."
        )

        print(
            "Enter L, R or F."
        )


# ============================================================
# EXECUTE INTERSECTION DIRECTION
# ============================================================

def execute_direction(
    direction
):

    print()

    print(
        f"Executing direction: {direction}"
    )


    # ========================================================
    # LEFT
    # ========================================================

    if direction == "L":

        print(
            "Turning LEFT for 1.5 seconds..."
        )

        stop()

        time.sleep(0.05)

        turn_left()

        # 1.5 SECOND LEFT TURN

        time.sleep(
            TURN_TIME
        )

        stop()

        time.sleep(0.05)

        forward()

        time.sleep(
            EXIT_TIME
        )

        stop()


    # ========================================================
    # RIGHT
    # ========================================================

    elif direction == "R":

        print(
            "Turning RIGHT for 1.5 seconds..."
        )

        stop()

        time.sleep(0.05)

        turn_right()

        # 1.5 SECOND RIGHT TURN

        time.sleep(
            TURN_TIME
        )

        stop()

        time.sleep(0.05)

        forward()

        time.sleep(
            EXIT_TIME
        )

        stop()


    # ========================================================
    # STRAIGHT
    # ========================================================

    elif direction == "F":

        print(
            "Going STRAIGHT..."
        )

        # No turning maneuver.

        forward()


# ============================================================
# STARTUP INFORMATION
# ============================================================

print()

print(
    "========================================"
)

print(
    "     THREE-POINT VISION LINE FOLLOWER"
)

print(
    "========================================"
)

print(
    f"Resolution : {WIDTH}x{HEIGHT}"
)

print(
    f"Threshold  : {THRESHOLD}"
)

print(
    f"Line ROI   : {ROI_TOP}:{ROI_BOTTOM}"
)

print(
    f"Dead zone  : {DEAD_ZONE}"
)

print(
    f"Turn time  : {TURN_TIME}s"
)

print(
    f"Lost-line reverse: {LINE_LOST_REVERSE_TIME}s"
)

print()

print(
    "Intersection points:"
)

print()

print(
    "              ●"
)

print()

print(
    "      ●                 ●"
)

print()

print(
    "Camera server:"
)

print(
    "http://<PI-IP>:8000"
)

print()

print(
    "Starting robot..."
)

print(
    "Press CTRL+C to stop."
)

print()


# ============================================================
# STATE
# ============================================================

crossing_frames = 0

crossing_locked = False

crossing_cooldown_until = 0


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while True:

        # ----------------------------------------------------
        # CAPTURE FRAME
        # ----------------------------------------------------

        frame = picam2.capture_array()


        # ----------------------------------------------------
        # NORMAL LINE FOLLOWING
        # ----------------------------------------------------

        line_x = find_line(
            frame
        )


        # ----------------------------------------------------
        # LINE LOST
        # ----------------------------------------------------

        if line_x is None:

            print(
                "LINE LOST -> REVERSING"
            )

            crossing_frames = 0

            crossing_locked = False

            # Reverse both motors.

            backward()

            time.sleep(
                LINE_LOST_REVERSE_TIME
            )

            # Stop before taking the next frame.

            stop()

            time.sleep(
                LOOP_DELAY
            )

            continue


        # ----------------------------------------------------
        # THRESHOLD
        # ----------------------------------------------------

        black = threshold_image(
            frame
        )


        # ----------------------------------------------------
        # READ THREE POINTS
        # ----------------------------------------------------

        states = read_crossing_points(
            black
        )


        # ----------------------------------------------------
        # INTERSECTION DETECTION
        # ----------------------------------------------------

        intersection = is_intersection(
            states
        )


        # ----------------------------------------------------
        # COOLDOWN
        # ----------------------------------------------------

        now = time.monotonic()

        cooldown = (
            now
            < crossing_cooldown_until
        )


        # ----------------------------------------------------
        # CONFIRM INTERSECTION
        # ----------------------------------------------------

        if (
            intersection
            and not crossing_locked
            and not cooldown
        ):

            crossing_frames += 1

        else:

            crossing_frames = 0


        # ----------------------------------------------------
        # CONFIRMED INTERSECTION
        # ----------------------------------------------------

        if (
            crossing_frames
            >= CROSSING_CONFIRM_FRAMES
            and not crossing_locked
            and not cooldown
        ):

            crossing_locked = True

            crossing_frames = 0

            stop()

            print()

            print(
                "========================================"
            )

            print(
                "          INTERSECTION DETECTED!"
            )

            print(
                "========================================"
            )

            print()

            print_pattern(
                states
            )


            # ------------------------------------------------
            # ASK USER
            # ------------------------------------------------

            direction = ask_direction()


            # ------------------------------------------------
            # EXECUTE
            # ------------------------------------------------

            execute_direction(
                direction
            )


            # ------------------------------------------------
            # COOLDOWN
            # ------------------------------------------------

            crossing_cooldown_until = (
                time.monotonic()
                + CROSSING_COOLDOWN
            )

            continue


        # ----------------------------------------------------
        # UNLOCK AFTER LEAVING INTERSECTION
        # ----------------------------------------------------

        if (
            crossing_locked
            and not intersection
        ):

            crossing_locked = False


        # ----------------------------------------------------
        # NORMAL LINE FOLLOWING
        # ----------------------------------------------------

        image_center = WIDTH / 2

        error = (
            line_x
            - image_center
        )


        # ----------------------------------------------------
        # CENTER
        # ----------------------------------------------------

        if abs(error) <= DEAD_ZONE:

            forward()

            state = "FORWARD"


        # ----------------------------------------------------
        # LINE LEFT
        # ----------------------------------------------------

        elif error < 0:

            turn_left()

            state = "LEFT"


        # ----------------------------------------------------
        # LINE RIGHT
        # ----------------------------------------------------

        else:

            turn_right()

            state = "RIGHT"


        # ----------------------------------------------------
        # DEBUG
        # ----------------------------------------------------

        pattern = (
            f"{int(states['FORWARD'])}"
            f"{int(states['LEFT'])}"
            f"{int(states['RIGHT'])}"
        )

        print(
            f"line={line_x:6.1f} "
            f"error={error:7.1f} "
            f"dots={pattern} "
            f"{state}"
        )

        time.sleep(
            LOOP_DELAY
        )


# ============================================================
# SAFE SHUTDOWN
# ============================================================

except KeyboardInterrupt:

    print(
        "\nStopping robot..."
    )


finally:

    server_running = False

    stop()

    picam2.stop()

    board.shutdown()

    print(
        "Robot stopped safely."
    )
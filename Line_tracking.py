import time
import io
import threading
import numpy as np

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from picamera2 import Picamera2
from PIL import Image, ImageDraw

from telemetrix import telemetrix

from pins import PB9, PB8, PB7, PB6


# ============================================================
# CONFIGURATION
# ============================================================

WIDTH = 416
HEIGHT = 416

# Pixels darker than this are considered BLACK.
THRESHOLD = 128


# ============================================================
# LINE FOLLOWING
# ============================================================

# Bottom part of the image used for line tracking.
ROI_TOP = 300
ROI_BOTTOM = 400

MIN_LINE_PIXELS = 20

# Robot drives straight when line center is this close
# to the image center.
DEAD_ZONE = 25

LOOP_DELAY = 0.02


# ============================================================
# CROSSING DETECTION
# ============================================================

# Horizontal scan line used to find the normal line edges.
#
# The line on the ground is normally vertical in the image,
# therefore we scan horizontally across it.

CROSSING_SCAN_Y = 350


# Minimum number of consecutive black pixels required
# to consider something an actual branch.
#
# This prevents isolated black pixels from triggering.
MIN_BRANCH_WIDTH = 8


# How far outside the normal line we require black pixels
# before considering them a branch.
BRANCH_MARGIN = 5


# Number of frames that must continuously show the same
# crossing before it is accepted.
CROSSING_CONFIRM_FRAMES = 4


# After detecting a crossing, don't detect another one
# for this amount of time.
CROSSING_COOLDOWN = 0.8


# ============================================================
# MOTOR PINS
# ============================================================

LEFT_IN1 = PB9
LEFT_IN2 = PB8

RIGHT_IN1 = PB7
RIGHT_IN2 = PB6


# ============================================================
# INTERSECTION TURNING
# ============================================================

# How long the robot rotates when choosing L/R.
#
# CHANGE THIS VALUE TO TUNE TURNING.
#
# Example:
#
# 0.5  = half a second
# 1.0  = one second
# 1.5  = one and a half seconds
#
TURN_TIME = 1.5


# Small forward movement after completing a turn.
EXIT_TIME = 0.15


# ============================================================
# CAMERA
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
# CAMERA FRAME SHARING
# ============================================================

latest_jpeg = None

frame_lock = threading.Lock()

server_running = True


# ============================================================
# CAMERA SERVER
# ============================================================

def camera_server_loop():

    global latest_jpeg

    while server_running:

        try:

            frame = picam2.capture_array()

            # ------------------------------------------------
            # RGB
            # ------------------------------------------------

            image = frame[:, :, :3]

            # Picamera2 normally gives XBGR8888.
            #
            # Reverse BGR -> RGB for Pillow.

            rgb = image[:, :, ::-1]


            # ------------------------------------------------
            # GRAYSCALE
            # ------------------------------------------------

            gray = np.mean(
                image,
                axis=2
            )


            # ------------------------------------------------
            # BINARY IMAGE
            # ------------------------------------------------

            black = (
                gray < THRESHOLD
            )


            binary = np.where(
                black,
                0,
                255
            ).astype(
                np.uint8
            )


            # ------------------------------------------------
            # CREATE DISPLAY IMAGE
            # ------------------------------------------------

            display = Image.fromarray(
                rgb
            ).convert(
                "RGB"
            )

            draw = ImageDraw.Draw(
                display
            )


            # ------------------------------------------------
            # DRAW ROI
            # ------------------------------------------------

            draw.line(
                [
                    (0, ROI_TOP),
                    (WIDTH, ROI_TOP)
                ],
                fill="yellow",
                width=2
            )

            draw.line(
                [
                    (0, ROI_BOTTOM),
                    (WIDTH, ROI_BOTTOM)
                ],
                fill="yellow",
                width=2
            )


            # ------------------------------------------------
            # DRAW CROSSING SCAN LINE
            # ------------------------------------------------

            draw.line(
                [
                    (0, CROSSING_SCAN_Y),
                    (WIDTH, CROSSING_SCAN_Y)
                ],
                fill="cyan",
                width=2
            )


            # ------------------------------------------------
            # FIND CURRENT LINE
            # ------------------------------------------------

            roi = black[
                ROI_TOP:ROI_BOTTOM,
                :
            ]

            y_pixels, x_pixels = np.where(
                roi
            )

            line_x = None

            if len(x_pixels) >= MIN_LINE_PIXELS:

                line_x = float(
                    np.mean(x_pixels)
                )


            # ------------------------------------------------
            # DRAW IMAGE CENTER
            # ------------------------------------------------

            center_x = WIDTH // 2

            draw.line(
                [
                    (center_x, 0),
                    (center_x, HEIGHT)
                ],
                fill="blue",
                width=2
            )


            # ------------------------------------------------
            # FIND ADAPTIVE EDGES
            # ------------------------------------------------

            edges = find_line_edges(
                black
            )


            if edges is not None:

                left_edge, right_edge = edges


                # --------------------------------------------
                # LEFT EDGE
                # --------------------------------------------

                draw.line(
                    [
                        (left_edge, 0),
                        (left_edge, HEIGHT)
                    ],
                    fill="green",
                    width=3
                )


                # --------------------------------------------
                # RIGHT EDGE
                # --------------------------------------------

                draw.line(
                    [
                        (right_edge, 0),
                        (right_edge, HEIGHT)
                    ],
                    fill="green",
                    width=3
                )


            # ------------------------------------------------
            # CROSSING DETECTION
            # ------------------------------------------------

            crossing = detect_crossing(
                black
            )


            # ------------------------------------------------
            # STATUS TEXT
            # ------------------------------------------------

            draw.rectangle(
                [
                    (5, 5),
                    (411, 75)
                ],
                fill="black"
            )


            if line_x is None:

                status = "LINE LOST"

            else:

                status = (
                    f"LINE X: {line_x:.1f}"
                )


            draw.text(
                (10, 10),
                status,
                fill="white"
            )

            draw.text(
                (10, 30),
                f"Crossing: {crossing}",
                fill="white"
            )

            draw.text(
                (10, 50),
                f"Threshold: {THRESHOLD}",
                fill="white"
            )


            # ------------------------------------------------
            # SAVE JPEG
            # ------------------------------------------------

            buffer = io.BytesIO()

            display.save(
                buffer,
                format="JPEG",
                quality=75
            )

            jpeg = buffer.getvalue()


            with frame_lock:

                latest_jpeg = jpeg


        except Exception as e:

            print(
                "Camera server error:",
                e
            )

            time.sleep(
                0.1
            )


# ============================================================
# WEB SERVER
# ============================================================

class CameraHandler(
    BaseHTTPRequestHandler
):


    # ========================================================
    # GET
    # ========================================================

    def do_GET(self):


        # ====================================================
        # MAIN PAGE
        # ====================================================

        if (
            self.path == "/"
            or self.path == "/index.html"
        ):

            page = """
<!DOCTYPE html>

<html>

<head>

<title>Robot Line Follower</title>

<meta
    name="viewport"
    content="width=device-width, initial-scale=1"
>

<style>

body {
    background: #111;
    color: white;
    font-family: Arial;
    text-align: center;
    margin: 0;
    padding: 20px;
}

h1 {
    margin-bottom: 15px;
}

img {
    width: 832px;
    max-width: 95vw;
    height: auto;
    border: 2px solid #444;
}

</style>

</head>

<body>

<h1>Robot Line Follower</h1>

<img src="/stream.mjpg">

</body>

</html>
"""

            data = page.encode()


            self.send_response(
                200
            )

            self.send_header(
                "Content-Type",
                "text/html"
            )

            self.send_header(
                "Content-Length",
                len(data)
            )

            self.end_headers()

            self.wfile.write(
                data
            )

            return


        # ====================================================
        # MJPEG STREAM
        # ====================================================

        if self.path == "/stream.mjpg":

            self.send_response(
                200
            )

            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame"
            )

            self.send_header(
                "Cache-Control",
                "no-cache"
            )

            self.send_header(
                "Pragma",
                "no-cache"
            )

            self.end_headers()


            try:

                while server_running:

                    with frame_lock:

                        jpeg = latest_jpeg


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
                        f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
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

            return


        # ====================================================
        # INVALID URL
        # ====================================================

        self.send_error(
            404
        )


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
# START WEB SERVER
# ============================================================

def start_web_server():

    server = ThreadingHTTPServer(
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
        "Open in browser:"
    )

    print(
        "http://<PI-IP>:8000"
    )

    print(
        "========================================"
    )

    print()


    try:

        server.serve_forever()

    finally:

        server.server_close()


# ============================================================
# START CAMERA THREAD
# ============================================================

camera_thread = threading.Thread(
    target=camera_server_loop,
    daemon=True
)

camera_thread.start()


# ============================================================
# START WEB SERVER THREAD
# ============================================================

web_thread = threading.Thread(
    target=start_web_server,
    daemon=True
)

web_thread.start()


# ============================================================
# STM32
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
# MOTOR INITIALIZATION
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


def turn_left():

    left_stop()

    right_forward()


def turn_right():

    left_forward()

    right_stop()


# ============================================================
# THRESHOLDING
# ============================================================

def threshold_image(
    frame
):

    image = frame[
        :,
        :,
        :3
    ]

    gray = np.mean(
        image,
        axis=2
    )

    black = (
        gray < THRESHOLD
    )

    return black


# ============================================================
# FIND NORMAL LINE
# ============================================================

def find_line(
    frame
):

    black = threshold_image(
        frame
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


    return float(
        np.mean(x)
    )


# ============================================================
# FIND ADAPTIVE LINE EDGES
# ============================================================

def find_line_edges(
    black
):

    y = CROSSING_SCAN_Y


    if y < 0 or y >= HEIGHT:

        return None


    row = black[y]


    # --------------------------------------------------------
    # Find all black runs in the scan row.
    # --------------------------------------------------------

    runs = []


    in_black = False

    start = None


    for x in range(WIDTH):

        if row[x] and not in_black:

            in_black = True

            start = x


        elif (
            not row[x]
            and in_black
        ):

            end = x - 1

            width = (
                end
                - start
                + 1
            )


            if width >= MIN_BRANCH_WIDTH:

                runs.append(
                    (
                        start,
                        end
                    )
                )


            in_black = False


    # --------------------------------------------------------
    # Handle black reaching image edge.
    # --------------------------------------------------------

    if in_black:

        end = WIDTH - 1

        width = (
            end
            - start
            + 1
        )


        if width >= MIN_BRANCH_WIDTH:

            runs.append(
                (
                    start,
                    end
                )
            )


    if not runs:

        return None


    # --------------------------------------------------------
    # Find run closest to image center.
    #
    # This should be the main line.
    # --------------------------------------------------------

    center = WIDTH / 2


    def distance_to_center(run):

        start, end = run

        run_center = (
            start + end
        ) / 2

        return abs(
            run_center - center
        )


    main_run = min(
        runs,
        key=distance_to_center
    )


    left_edge = main_run[0]

    right_edge = main_run[1]


    return (
        left_edge,
        right_edge
    )


# ============================================================
# CROSSING DETECTION
# ============================================================

def detect_crossing(
    black
):

    edges = find_line_edges(
        black
    )


    if edges is None:

        return "NONE"


    left_edge, right_edge = edges


    row = black[
        CROSSING_SCAN_Y
    ]


    # --------------------------------------------------------
    # SEARCH LEFT OF NORMAL LINE
    # --------------------------------------------------------

    left_region_end = max(
        0,
        left_edge - BRANCH_MARGIN
    )


    left_region = row[
        :left_region_end
    ]


    # --------------------------------------------------------
    # SEARCH RIGHT OF NORMAL LINE
    # --------------------------------------------------------

    right_region_start = min(
        WIDTH,
        right_edge + BRANCH_MARGIN + 1
    )


    right_region = row[
        right_region_start:
    ]


    # --------------------------------------------------------
    # Determine whether there is meaningful black.
    # --------------------------------------------------------

    left_black = (
        np.count_nonzero(
            left_region
        )
        >= MIN_BRANCH_WIDTH
    )


    right_black = (
        np.count_nonzero(
            right_region
        )
        >= MIN_BRANCH_WIDTH
    )


    # --------------------------------------------------------
    # RESULT
    # --------------------------------------------------------

    if (
        left_black
        and right_black
    ):

        return "BOTH"


    if left_black:

        return "LEFT"


    if right_black:

        return "RIGHT"


    return "NONE"


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
# EXECUTE INTERSECTION
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
            f"Turning LEFT for {TURN_TIME}s..."
        )


        stop()

        time.sleep(
            0.05
        )


        turn_left()

        time.sleep(
            TURN_TIME
        )


        stop()

        time.sleep(
            0.05
        )


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
            f"Turning RIGHT for {TURN_TIME}s..."
        )


        stop()

        time.sleep(
            0.05
        )


        turn_right()

        time.sleep(
            TURN_TIME
        )


        stop()

        time.sleep(
            0.05
        )


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


        # No turning.
        # No unnecessary stop.
        #
        # Just continue forward.

        forward()


# ============================================================
# STARTUP
# ============================================================

print()

print(
    "========================================"
)

print(
    "    ADAPTIVE VISION LINE FOLLOWER"
)

print(
    "========================================"
)

print(
    f"Resolution       : {WIDTH}x{HEIGHT}"
)

print(
    f"Threshold        : {THRESHOLD}"
)

print(
    f"Line ROI         : {ROI_TOP}:{ROI_BOTTOM}"
)

print(
    f"Scan Y           : {CROSSING_SCAN_Y}"
)

print(
    f"Dead zone        : {DEAD_ZONE}"
)

print(
    f"Turn time        : {TURN_TIME}s"
)

print(
    f"Debounce frames  : {CROSSING_CONFIRM_FRAMES}"
)

print(
    f"Cooldown         : {CROSSING_COOLDOWN}s"
)

print()

print(
    "Crossing detection:"
)

print()

print(
    "LEFT  = black outside left edge"
)

print(
    "RIGHT = black outside right edge"
)

print(
    "BOTH  = black outside both edges"
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

last_crossing = "NONE"

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
        # FIND LINE
        # ----------------------------------------------------

        line_x = find_line(
            frame
        )


        # ----------------------------------------------------
        # LINE LOST
        # ----------------------------------------------------

        if line_x is None:

            print(
                "LINE LOST"
            )


            # Stop rather than continuing
            # blindly.

            stop()


            crossing_frames = 0


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
        # FIND ADAPTIVE EDGES
        # ----------------------------------------------------

        edges = find_line_edges(
            black
        )


        # ----------------------------------------------------
        # CROSSING
        # ----------------------------------------------------

        crossing = detect_crossing(
            black
        )


        # ----------------------------------------------------
        # TIME
        # ----------------------------------------------------

        now = time.monotonic()


        cooldown = (
            now
            < crossing_cooldown_until
        )


        # ----------------------------------------------------
        # DEBOUNCE
        # ----------------------------------------------------

        if (
            crossing != "NONE"
            and not crossing_locked
            and not cooldown
        ):

            # Same detection as previous frame.

            if crossing == last_crossing:

                crossing_frames += 1

            else:

                crossing_frames = 1

                last_crossing = crossing


        else:

            crossing_frames = 0

            if crossing == "NONE":

                last_crossing = "NONE"


        # ----------------------------------------------------
        # CONFIRMED CROSSING
        # ----------------------------------------------------

        if (
            crossing_frames
            >= CROSSING_CONFIRM_FRAMES
            and not crossing_locked
            and not cooldown
        ):

            crossing_locked = True

            crossing_frames = 0


            # ------------------------------------------------
            # STOP
            # ------------------------------------------------

            stop()


            print()

            print(
                "========================================"
            )

            print(
                "          CROSSING DETECTED!"
            )

            print(
                "========================================"
            )

            print(
                f"Type: {crossing}"
            )

            print()


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
        # UNLOCK AFTER LEAVING CROSSING
        # ----------------------------------------------------

        if (
            crossing_locked
            and crossing == "NONE"
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
        # LEFT
        # ----------------------------------------------------

        elif error < 0:

            turn_left()

            state = "LEFT"


        # ----------------------------------------------------
        # RIGHT
        # ----------------------------------------------------

        else:

            turn_right()

            state = "RIGHT"


        # ----------------------------------------------------
        # DEBUG
        # ----------------------------------------------------

        if edges is not None:

            left_edge, right_edge = edges

            edge_text = (
                f"edges={left_edge}:{right_edge}"
            )

        else:

            edge_text = (
                "edges=None"
            )


        print(
            f"line={line_x:6.1f} "
            f"error={error:7.1f} "
            f"{edge_text} "
            f"cross={crossing:5s} "
            f"confirm={crossing_frames} "
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


    try:

        picam2.stop()

    except Exception:

        pass


    try:

        board.shutdown()

    except Exception:

        pass


    print(
        "Robot stopped safely."
    )
import time

import cv2
import numpy as np

from camera import Camera


# ============================================================
# CAMERA
# ============================================================

WIDTH = 800
HEIGHT = 600


# ============================================================
# LINE THRESHOLDING
# ============================================================

# Pixels darker than this are black.

THRESHOLD = 100


# ============================================================
# ROI
# ============================================================

# Only this part of the image is used.

ROI_TOP = 300
ROI_BOTTOM = 570


# ============================================================
# LINE CLEANING
# ============================================================

MIN_LINE_PIXELS = 100

MORPH_KERNEL_SIZE = 5


# ============================================================
# LINE TRACKING
# ============================================================

# How far from the previous line center
# the search is allowed to move.

MAX_CENTER_JUMP = 250


# Normal steering dead zone.

DEAD_ZONE = 35


# ============================================================
# EDGE SCANNING
# ============================================================

# The scan is performed around this Y position.

EDGE_SCAN_Y = 450


# Start positions of the two probes.

LEFT_SCAN_START = 20

RIGHT_SCAN_START = WIDTH - 20


# Number of pixels skipped during scan.

SCAN_STEP = 2


# Width around the detected edge used
# when checking for extra branches.

EDGE_MARGIN = 15


# ============================================================
# CROSSING DETECTION
# ============================================================

# Minimum number of black pixels outside
# the normal line to count as an external branch.

EXTERNAL_BLACK_MIN = 8


# Number of consecutive frames required.

CROSSING_CONFIRM_FRAMES = 4


# Minimum time between crossing events.

CROSSING_COOLDOWN = 1.0


# ============================================================
# DEBUG
# ============================================================

SHOW_DEBUG = True


# ============================================================
# THRESHOLD IMAGE
# ============================================================

def threshold_image(frame):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Black becomes white in the mask.

    black = cv2.inRange(gray, 0, THRESHOLD)

    # Remove small noise.

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
    )

    black = cv2.morphologyEx(black, cv2.MORPH_OPEN, kernel)
    black = cv2.morphologyEx(black, cv2.MORPH_CLOSE, kernel)

    return black


# ============================================================
# FIND LINE CENTER
# ============================================================

def find_line(black):

    roi = black[ROI_TOP:ROI_BOTTOM,:]

    y, x = np.where(roi > 0)

    if len(x) < MIN_LINE_PIXELS:
        return None

    # Median is less affected by occasional
    # black noise than a simple mean.

    line_x = float(np.median(x))

    return line_x


# ============================================================
# FIND LINE EDGES
# ============================================================

def find_line_edges(
    black,
    scan_y=EDGE_SCAN_Y
):

    """
    Find the left and right edges of the
    current black line.

    LEFT:
        scans left -> right.

    RIGHT:
        scans right -> left.

    The first black region encountered
    is treated as the tracked line.
    """

    scan_y = max(
        0,
        min(
            HEIGHT - 1,
            scan_y
        )
    )


    row = black[
        scan_y,
        :
    ] > 0


    # --------------------------------------------------------
    # LEFT EDGE
    # --------------------------------------------------------

    left_edge = None

    for x in range(
        LEFT_SCAN_START,
        WIDTH,
        SCAN_STEP
    ):

        if row[x]:

            left_edge = x

            break


    # --------------------------------------------------------
    # RIGHT EDGE
    # --------------------------------------------------------

    right_edge = None

    for x in range(
        RIGHT_SCAN_START,
        -1,
        -SCAN_STEP
    ):

        if row[x]:

            right_edge = x

            break


    # --------------------------------------------------------
    # VALIDATE
    # --------------------------------------------------------

    if (
        left_edge is None
        or right_edge is None
        or left_edge >= right_edge
    ):

        return None, None


    return (
        left_edge,
        right_edge
    )


# ============================================================
# CROSSING DETECTION
# ============================================================

def detect_crossing(
    black,
    left_edge,
    right_edge,
    scan_y=EDGE_SCAN_Y
):

    """
    Look for black pixels outside the
    currently tracked line.

    Returns:

        LEFT
        RIGHT
        BOTH
        NONE
    """

    if (
        left_edge is None
        or right_edge is None
    ):

        return "NONE"


    scan_y = max(
        0,
        min(
            HEIGHT - 1,
            scan_y
        )
    )


    row = (
        black[scan_y, :] > 0
    )


    # --------------------------------------------------------
    # LEFT EXTERNAL REGION
    # --------------------------------------------------------

    left_start = 0

    left_end = max(
        0,
        left_edge - EDGE_MARGIN
    )

    left_external = row[
        left_start:left_end
    ]


    # --------------------------------------------------------
    # RIGHT EXTERNAL REGION
    # --------------------------------------------------------

    right_start = min(
        WIDTH,
        right_edge + EDGE_MARGIN
    )

    right_external = row[
        right_start:WIDTH
    ]


    left_black = np.count_nonzero(
        left_external
    )

    right_black = np.count_nonzero(
        right_external
    )


    left_detected = (
        left_black
        >= EXTERNAL_BLACK_MIN
    )

    right_detected = (
        right_black
        >= EXTERNAL_BLACK_MIN
    )


    if (
        left_detected
        and right_detected
    ):

        return "BOTH"


    if left_detected:

        return "LEFT"


    if right_detected:

        return "RIGHT"


    return "NONE"


# ============================================================
# DRAW DEBUG IMAGE
# ============================================================

def create_debug_image(
    frame,
    black,
    line_x,
    left_edge,
    right_edge,
    crossing
):

    # Convert binary mask into BGR.

    debug = cv2.cvtColor(
        black,
        cv2.COLOR_GRAY2BGR
    )


    # --------------------------------------------------------
    # ROI
    # --------------------------------------------------------

    cv2.line(
        debug,
        (
            0,
            ROI_TOP
        ),
        (
            WIDTH,
            ROI_TOP
        ),
        (128, 128, 128),
        1
    )


    # --------------------------------------------------------
    # EDGE SCAN LINE
    # --------------------------------------------------------

    cv2.line(
        debug,
        (
            0,
            EDGE_SCAN_Y
        ),
        (
            WIDTH,
            EDGE_SCAN_Y
        ),
        (128, 128, 128),
        1
    )


    # --------------------------------------------------------
    # LINE CENTER
    # --------------------------------------------------------

    if line_x is not None:

        x = int(
            line_x
        )

        cv2.line(
            debug,
            (
                x,
                ROI_TOP
            ),
            (
                x,
                ROI_BOTTOM
            ),
            (255, 255, 255),
            2
        )


    # --------------------------------------------------------
    # LEFT EDGE
    # --------------------------------------------------------

    if left_edge is not None:

        cv2.line(
            debug,
            (
                left_edge,
                0
            ),
            (
                left_edge,
                HEIGHT
            ),
            (255, 0, 0),
            2
        )


    # --------------------------------------------------------
    # RIGHT EDGE
    # --------------------------------------------------------

    if right_edge is not None:

        cv2.line(
            debug,
            (
                right_edge,
                0
            ),
            (
                right_edge,
                HEIGHT
            ),
            (0, 0, 255),
            2
        )


    # --------------------------------------------------------
    # CROSSING TEXT
    # --------------------------------------------------------

    cv2.putText(
        debug,
        f"CROSSING: {crossing}",
        (
            20,
            40
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255, 255, 255),
        2
    )


    return debug


# ============================================================
# MAIN
# ============================================================

camera = Camera(
    width=WIDTH,
    height=HEIGHT
)


# ============================================================
# STATE
# ============================================================

crossing_counter = 0

last_crossing = "NONE"

crossing_cooldown_until = 0


# ============================================================
# START
# ============================================================

camera.start()


print(
    "Line follower started."
)

print(
    "Camera preview:"
)

print(
    f"http://<PI-IP>:{camera.port}/"
)

print()

print(
    "Press CTRL+C to stop."
)

print()


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while True:

        # ----------------------------------------------------
        # GET CAMERA FRAME
        # ----------------------------------------------------

        frame = camera.get_frame()


        if frame is None:

            time.sleep(
                0.01
            )

            continue


        # ----------------------------------------------------
        # THRESHOLD
        # ----------------------------------------------------

        black = threshold_image(
            frame
        )


        # ----------------------------------------------------
        # FIND LINE CENTER
        # ----------------------------------------------------

        line_x = find_line(
            black
        )


        # ----------------------------------------------------
        # FIND ADAPTIVE EDGES
        # ----------------------------------------------------

        left_edge, right_edge = (
            find_line_edges(
                black
            )
        )


        # ----------------------------------------------------
        # LINE LOST
        # ----------------------------------------------------

        if line_x is None:

            print(
                "LINE LOST"
            )

            # Publish threshold image anyway.

            camera.set_processed_frame(
                black
            )

            crossing_counter = 0

            time.sleep(
                0.02
            )

            continue


        # ----------------------------------------------------
        # CROSSING
        # ----------------------------------------------------

        crossing = detect_crossing(
            black,
            left_edge,
            right_edge
        )


        now = time.monotonic()


        # ----------------------------------------------------
        # CROSSING DEBOUNCE
        # ----------------------------------------------------

        if (
            crossing != "NONE"
            and now >= crossing_cooldown_until
        ):

            if crossing == last_crossing:

                crossing_counter += 1

            else:

                crossing_counter = 1

                last_crossing = crossing


        else:

            crossing_counter = 0

            last_crossing = "NONE"


        # ----------------------------------------------------
        # CONFIRMED CROSSING
        # ----------------------------------------------------

        if (
            crossing_counter
            >= CROSSING_CONFIRM_FRAMES
        ):

            print()
            print(
                "========================================"
            )

            print(
                "        CROSSING DETECTED"
            )

            print(
                f"        Direction: {crossing}"
            )

            print(
                "========================================"
            )

            print()


            crossing_cooldown_until = (
                time.monotonic()
                + CROSSING_COOLDOWN
            )


            crossing_counter = 0

            last_crossing = "NONE"


        # ----------------------------------------------------
        # NORMAL LINE FOLLOWING ERROR
        # ----------------------------------------------------

        image_center = (
            WIDTH / 2
        )

        error = (
            line_x
            - image_center
        )


        # ----------------------------------------------------
        # STEERING STATE
        # ----------------------------------------------------

        if abs(error) <= DEAD_ZONE:

            state = "FORWARD"

        elif error < 0:

            state = "LEFT"

        else:

            state = "RIGHT"


        # ----------------------------------------------------
        # DEBUG IMAGE
        # ----------------------------------------------------

        if SHOW_DEBUG:

            processed = create_debug_image(
                frame,
                black,
                line_x,
                left_edge,
                right_edge,
                crossing
            )

        else:

            processed = black


        # ----------------------------------------------------
        # SEND PROCESSED IMAGE TO CAMERA SERVER
        # ----------------------------------------------------

        camera.set_processed_frame(
            processed
        )


        # ----------------------------------------------------
        # TERMINAL DEBUG
        # ----------------------------------------------------

        print(
            f"line={line_x:6.1f} "
            f"left={str(left_edge):>4} "
            f"right={str(right_edge):>4} "
            f"error={error:7.1f} "
            f"cross={crossing:<5} "
            f"{state}"
        )


        time.sleep(
            0.02
        )


# ============================================================
# SAFE SHUTDOWN
# ============================================================

except KeyboardInterrupt:

    print(
        "\nStopping line follower..."
    )


finally:

    camera.stop()

    print(
        "Line follower stopped safely."
    )


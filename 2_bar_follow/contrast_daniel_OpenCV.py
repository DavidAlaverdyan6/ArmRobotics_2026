import time
import cv2
import numpy as np
from camera import Camera


# ============================================================
# CONFIGURATION
# ============================================================

WIDTH = 800
HEIGHT = 600

MIN_OBJECT_AREA = 700

# Tweak these
MIN_BRIGHTNESS = 60
MIN_CHANNEL_DIFF = 25

# Color IDs
RED = 1
YELLOW = 2
GREEN = 3
BLUE = 4


# ============================================================
# CAMERA
# ============================================================

ContCam = Camera(
    width=WIDTH,
    height=HEIGHT
)

ContCam.start()


# ============================================================
# COLOR CLASSIFICATION
# ============================================================

def classify_colors(frame):

    # OpenCV BGR
    b = frame[:, :, 0].astype(np.uint16)
    g = frame[:, :, 1].astype(np.uint16)
    r = frame[:, :, 2].astype(np.uint16)

    color_map = np.zeros(
        (HEIGHT, WIDTH),
        dtype=np.uint8
    )

    # --------------------------------------------------------
    # BRIGHTNESS / COLOR FILTER
    # --------------------------------------------------------

    max_channel = np.maximum(
        np.maximum(r, g),
        b
    )

    min_channel = np.minimum(
        np.minimum(r, g),
        b
    )

    bright = max_channel >= MIN_BRIGHTNESS

    colorful = (
        (max_channel - min_channel)
        >= MIN_CHANNEL_DIFF
    )

    valid = bright & colorful


    # ========================================================
    # YELLOW
    # ========================================================

    yellow = valid & (
        (r > 120) &
        (g > 100) &
        (b * 100 < np.minimum(r, g) * 65)
    )

    color_map[yellow] = YELLOW


    # ========================================================
    # RED
    # ========================================================

    red = valid & (
        (r * 100 > g * 135) &
        (r * 100 > b * 135)
    )

    color_map[red] = RED


    # ========================================================
    # GREEN
    # ========================================================

    green = valid & (
        (g * 100 > r * 115) &
        (g * 100 > b * 110)
    )

    color_map[green] = GREEN


    # ========================================================
    # BLUE
    # ========================================================

    blue = valid & (
        (b * 100 > r * 125) &
        (b * 100 > g * 115)
    )

    color_map[blue] = BLUE


    return color_map


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while True:

        frame = ContCam.get_frame()

        if frame is None:
            time.sleep(0.001)
            continue


        # ====================================================
        # CLASSIFY COLORS
        # ====================================================

        color_map = classify_colors(frame)


        # ====================================================
        # OBJECT MASK
        # ====================================================

        mask = (
            color_map != 0
        ).astype(np.uint8) * 255


        # ====================================================
        # FIND OBJECTS
        # ====================================================

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )


        # ====================================================
        # RESULT IMAGE
        # ====================================================

        result = frame.copy()

        # Fill detected pixels with their classified color
        result[color_map == RED] = (0, 0, 255)
        result[color_map == YELLOW] = (0, 255, 255)
        result[color_map == GREEN] = (0, 255, 0)
        result[color_map == BLUE] = (255, 0, 0)

        objects = []


        # ====================================================
        # PROCESS OBJECTS
        # ====================================================

        for contour in contours:

            area = cv2.contourArea(contour)

            if area < MIN_OBJECT_AREA:
                continue

            x, y, width, height = cv2.boundingRect(
                contour
            )


            # ------------------------------------------------
            # Get color from contour point
            # ------------------------------------------------

            px_x, px_y = contour[0][0]

            color_id = int(
                color_map[px_y, px_x]
            )

            # If the contour point somehow isn't classified,
            # skip it.
            if color_id == 0:
                continue


            objects.append({
                "color": color_id,
                "x": int(x),
                "y": int(y),
                "width": int(width),
                "height": int(height),
                "area": int(area)
            })


            # ------------------------------------------------
            # Bounding box
            # ------------------------------------------------

            cv2.rectangle(
                result,
                (x, y),
                (x + width - 1, y + height - 1),
                (255, 255, 255),
                3
            )

            # ------------------------------------------------
            # Center crosshair
            # ------------------------------------------------

            center_x = x + width // 2
            center_y = y + height // 2

            crosshair_size = 6

            cv2.line(
                 result,
                (center_x - crosshair_size, center_y),
                (center_x + crosshair_size, center_y),
                (255, 255, 255),
                2
            )

            cv2.line(
                result,
                (center_x, center_y - crosshair_size),
                (center_x, center_y + crosshair_size),
                (255, 255, 255),
                2
            )

            # ------------------------------------------------
            # Object number
            # ------------------------------------------------

            cv2.putText(
                result,
                str(len(objects)),
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )


        # ====================================================
        # PRINT OBJECT DATA
        # ====================================================

        print(f"Objects: {len(objects)}")

        for i, obj in enumerate(objects):

            print(
                f"  Object {i + 1}: "
                f"color={obj['color']} "
                f"x={obj['x']} "
                f"y={obj['y']} "
                f"size={obj['width']}x{obj['height']} "
                f"area={obj['area']}"
            )


        # ====================================================
        # SEND PROCESSED FRAME
        # ====================================================

        ContCam.set_processed_frame(result)


except KeyboardInterrupt:
    pass


finally:
    ContCam.stop()
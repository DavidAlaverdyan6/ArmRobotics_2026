# vision.py
#
# Vision-only module.
#
# No camera hardware.
# No motors.
# No Telemetrix.
#
# Uses an OpenCV frame supplied by camera.py.
#
# Features:
#   - BW thresholding
#   - Noise cleanup
#   - Adaptive left/right line-edge detection
#   - Crossing detection
#   - Debouncing / confirmation
#   - Debug visualization
#
# Edge detection:
#
# LEFT:
#     scans LEFT -> RIGHT
#
# RIGHT:
#     scans RIGHT -> LEFT
#
# This means the line edges continuously follow
# the actual position of the line.

import time

import cv2
import numpy as np


# ============================================================
# CONFIGURATION
# ============================================================

THRESHOLD = 100

# Region used for line tracking.

ROI_TOP = 260
ROI_BOTTOM = 410


# Minimum amount of black pixels
# required for a valid line.

MIN_LINE_PIXELS = 20


# Morphological cleanup.

MORPH_KERNEL_SIZE = 5


# ============================================================
# EDGE SEARCH
# ============================================================

# Number of vertical samples used to determine
# the line position.

EDGE_SAMPLE_HEIGHT = 30

# Ignore tiny black regions.

MIN_RUN_WIDTH = 3


# ============================================================
# CROSSING DETECTION
# ============================================================

# Extra black pixels outside the normal
# line edges required to call it a branch.

CROSSING_MIN_PIXELS = 8

# How many consecutive frames must agree
# before triggering.

CROSSING_CONFIRM_FRAMES = 4

# Minimum time between crossing events.

CROSSING_COOLDOWN = 1.0


# ============================================================
# LINE TRACKER
# ============================================================

class LineTracker:

    def __init__(self):
        self.crossing_counter = 0
        self.last_crossing_time = (-float("inf"))
        self.last_crossing = None

        self.line_left = None
        self.line_right = None
        self.line_center = None

        self.last_error = 0


    # ========================================================
    # THRESHOLD
    # ========================================================

    def threshold_image(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Black = white in mask.

        _, binary = cv2.threshold(
            gray,
            THRESHOLD,
            255,
            cv2.THRESH_BINARY_INV
        )

        # Remove isolated noise.

        kernel = np.ones(
            (
                MORPH_KERNEL_SIZE,
                MORPH_KERNEL_SIZE
            ),
            np.uint8
        )

        binary = cv2.morphologyEx(
            binary,
            cv2.MORPH_OPEN,
            kernel
        )

        binary = cv2.morphologyEx(
            binary,
            cv2.MORPH_CLOSE,
            kernel
        )

        return binary


    # ========================================================
    # FIND LINE EDGES
    # ========================================================

    def find_line_edges(
        self,
        binary
    ):

        height, width = binary.shape

        y1 = max(
            0,
            ROI_TOP
        )

        y2 = min(
            height,
            ROI_BOTTOM
        )

        roi = binary[
            y1:y2,
            :
        ]

        if roi.size == 0:

            return None, None


        # ----------------------------------------------------
        # Collapse vertically.
        #
        # For each X coordinate we calculate how much
        # black is present.
        # ----------------------------------------------------

        black_count = np.count_nonzero(
            roi,
            axis=0
        )


        # ----------------------------------------------------
        # Ignore very weak pixels.
        # ----------------------------------------------------

        min_vertical_pixels = max(
            3,
            EDGE_SAMPLE_HEIGHT // 5
        )

        line_columns = (
            black_count
            >= min_vertical_pixels
        )


        # ----------------------------------------------------
        # LEFT EDGE
        #
        # Search from LEFT -> RIGHT.
        #
        # First sufficiently large black region
        # is considered the line.
        # ----------------------------------------------------

        left_edge = None

        run_start = None

        for x in range(width):

            if line_columns[x]:

                if run_start is None:

                    run_start = x

            else:

                if run_start is not None:

                    run_width = (
                        x
                        - run_start
                    )

                    if run_width >= MIN_RUN_WIDTH:

                        left_edge = run_start

                        break

                    run_start = None


        # Handle run reaching image edge.

        if (
            left_edge is None
            and run_start is not None
        ):

            if (
                width
                - run_start
                >= MIN_RUN_WIDTH
            ):

                left_edge = run_start


        # ----------------------------------------------------
        # RIGHT EDGE
        #
        # Search from RIGHT -> LEFT.
        # ----------------------------------------------------

        right_edge = None

        run_end = None

        for x in range(
            width - 1,
            -1,
            -1
        ):

            if line_columns[x]:

                if run_end is None:

                    run_end = x

            else:

                if run_end is not None:

                    run_width = (
                        run_end
                        - x
                    )

                    if run_width >= MIN_RUN_WIDTH:

                        right_edge = run_end

                        break

                    run_end = None


        if (
            right_edge is None
            and run_end is not None
        ):

            if (
                run_end + 1
                >= MIN_RUN_WIDTH
            ):

                right_edge = run_end


        # ----------------------------------------------------
        # Validate
        # ----------------------------------------------------

        if (
            left_edge is None
            or right_edge is None
        ):

            return None, None


        if right_edge <= left_edge:

            return None, None


        return (
            left_edge,
            right_edge
        )


    # ========================================================
    # FIND LINE CENTER
    # ========================================================

    def find_line_center(
        self,
        binary
    ):

        height, width = binary.shape

        y1 = max(
            0,
            ROI_TOP
        )

        y2 = min(
            height,
            ROI_BOTTOM
        )

        roi = binary[
            y1:y2,
            :
        ]

        y, x = np.where(
            roi > 0
        )

        if len(x) < MIN_LINE_PIXELS:

            return None


        return float(
            np.mean(x)
        )


    # ========================================================
    # CROSSING DETECTION
    # ========================================================

    def detect_crossing(
        self,
        binary,
        left_edge,
        right_edge
    ):

        height, width = binary.shape

        y1 = max(
            0,
            ROI_TOP
        )

        y2 = min(
            height,
            ROI_BOTTOM
        )

        roi = binary[
            y1:y2,
            :
        ]


        # ----------------------------------------------------
        # We inspect BLACK outside the detected line.
        #
        # left side:
        #   [0 ... left_edge]
        #
        # right side:
        #   [right_edge ... width]
        # ----------------------------------------------------

        left_region = roi[
            :,
            :left_edge
        ]

        right_region = roi[
            :,
            right_edge + 1:
        ]


        left_pixels = np.count_nonzero(
            left_region
        )

        right_pixels = np.count_nonzero(
            right_region
        )


        left_detected = (
            left_pixels
            >= CROSSING_MIN_PIXELS
        )

        right_detected = (
            right_pixels
            >= CROSSING_MIN_PIXELS
        )


        if (
            left_detected
            and right_detected
        ):

            direction = "BOTH"

        elif left_detected:

            direction = "LEFT"

        elif right_detected:

            direction = "RIGHT"

        else:

            direction = None


        return (
            direction,
            left_pixels,
            right_pixels
        )


    # ========================================================
    # DEBOUNCE CROSSING
    # ========================================================

    def debounce_crossing(
        self,
        crossing
    ):

        now = time.monotonic()


        # ----------------------------------------------------
        # No crossing this frame.
        # ----------------------------------------------------

        if crossing is None:

            self.crossing_counter = 0

            return None


        # ----------------------------------------------------
        # Cooldown.
        # ----------------------------------------------------

        if (
            now
            - self.last_crossing_time
            < CROSSING_COOLDOWN
        ):

            return None


        # ----------------------------------------------------
        # Consecutive confirmation.
        # ----------------------------------------------------

        self.crossing_counter += 1


        if (
            self.crossing_counter
            < CROSSING_CONFIRM_FRAMES
        ):

            return None


        # ----------------------------------------------------
        # Confirmed.
        # ----------------------------------------------------

        self.crossing_counter = 0

        self.last_crossing_time = now

        self.last_crossing = crossing

        return crossing


    # ========================================================
    # PROCESS FRAME
    # ========================================================

    def process(
        self,
        frame
    ):

        height, width = frame.shape[:2]

        binary = self.threshold_image(
            frame
        )


        # ----------------------------------------------------
        # Find line edges.
        # ----------------------------------------------------

        left_edge, right_edge = (
            self.find_line_edges(
                binary
            )
        )


        # ----------------------------------------------------
        # Line lost.
        # ----------------------------------------------------

        if (
            left_edge is None
            or right_edge is None
        ):

            self.line_left = None
            self.line_right = None
            self.line_center = None

            self.crossing_counter = 0

            display = frame.copy()

            cv2.putText(
                display,
                "LINE LOST",
                (
                    10,
                    30
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

            return {
                "binary": binary,
                "display": display,
                "line_left": None,
                "line_right": None,
                "line_center": None,
                "crossing": None,
                "confirmed_crossing": None
            }


        self.line_left = left_edge
        self.line_right = right_edge


        # ----------------------------------------------------
        # Center based on detected edges.
        # ----------------------------------------------------

        center = (
            left_edge
            + right_edge
        ) / 2.0

        self.line_center = center


        # ----------------------------------------------------
        # Error.
        # ----------------------------------------------------

        image_center = width / 2.0

        error = (
            center
            - image_center
        )

        self.last_error = error


        # ----------------------------------------------------
        # Crossing.
        # ----------------------------------------------------

        (
            crossing,
            left_pixels,
            right_pixels
        ) = self.detect_crossing(
            binary,
            left_edge,
            right_edge
        )


        confirmed_crossing = (
            self.debounce_crossing(
                crossing
            )
        )


        # ----------------------------------------------------
        # DEBUG IMAGE.
        # ----------------------------------------------------

        display = frame.copy()


        # ----------------------------------------------------
        # ROI
        # ----------------------------------------------------

        cv2.line(
            display,
            (
                0,
                ROI_TOP
            ),
            (
                width,
                ROI_TOP
            ),
            (255, 255, 0),
            1
        )

        cv2.line(
            display,
            (
                0,
                ROI_BOTTOM
            ),
            (
                width,
                ROI_BOTTOM
            ),
            (255, 255, 0),
            1
        )


        # ----------------------------------------------------
        # Left edge.
        # ----------------------------------------------------

        cv2.line(
            display,
            (
                left_edge,
                ROI_TOP
            ),
            (
                left_edge,
                ROI_BOTTOM
            ),
            (0, 255, 0),
            2
        )


        # ----------------------------------------------------
        # Right edge.
        # ----------------------------------------------------

        cv2.line(
            display,
            (
                right_edge,
                ROI_TOP
            ),
            (
                right_edge,
                ROI_BOTTOM
            ),
            (0, 255, 0),
            2
        )


        # ----------------------------------------------------
        # Center.
        # ----------------------------------------------------

        cv2.line(
            display,
            (
                int(center),
                ROI_TOP
            ),
            (
                int(center),
                ROI_BOTTOM
            ),
            (255, 0, 0),
            2
        )


        # ----------------------------------------------------
        # Image center.
        # ----------------------------------------------------

        cv2.line(
            display,
            (
                int(image_center),
                ROI_TOP
            ),
            (
                int(image_center),
                ROI_BOTTOM
            ),
            (255, 0, 255),
            1
        )


        # ----------------------------------------------------
        # Crossing status.
        # ----------------------------------------------------

        if confirmed_crossing is not None:

            text = (
                "CROSSING: "
                + confirmed_crossing
            )

            cv2.putText(
                display,
                text,
                (
                    10,
                    30
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2
            )

        elif crossing is not None:

            text = (
                "DETECTING: "
                + crossing
                + f" "
                + str(
                    self.crossing_counter
                )
                + "/"
                + str(
                    CROSSING_CONFIRM_FRAMES
                )
            )

            cv2.putText(
                display,
                text,
                (
                    10,
                    30
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2
            )

        else:

            cv2.putText(
                display,
                "NORMAL",
                (
                    10,
                    30
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )


        # ----------------------------------------------------
        # Debug information.
        # ----------------------------------------------------

        cv2.putText(
            display,
            f"L={left_edge} "
            f"R={right_edge} "
            f"C={center:.1f}",
            (
                10,
                height - 35
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )


        cv2.putText(
            display,
            f"Outside: "
            f"L={left_pixels} "
            f"R={right_pixels}",
            (
                10,
                height - 15
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )


        return {
            "binary": binary,
            "display": display,
            "line_left": left_edge,
            "line_right": right_edge,
            "line_center": center,
            "error": error,
            "crossing": crossing,
            "confirmed_crossing": confirmed_crossing
        }


# ============================================================
# DIRECT TEST
# ============================================================

if __name__ == "__main__":

    from camera import camera

    tracker = LineTracker()

    camera.start()

    print(
        "Vision test running."
    )

    try:

        while True:

            frame = camera.get_frame()

            if frame is None:

                time.sleep(
                    0.01
                )

                continue


            result = tracker.process(
                frame
            )


            camera.set_processed_frame(
                result["display"]
            )


            if (
                result["confirmed_crossing"]
                is not None
            ):

                print(
                    "CROSSING:",
                    result[
                        "confirmed_crossing"
                    ]
                )


            time.sleep(
                0.01
            )


    except KeyboardInterrupt:

        pass

    finally:

        camera.stop()


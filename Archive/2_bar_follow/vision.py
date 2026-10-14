import cv2
import numpy as np
import time


# ============================================================
# SETTINGS
# ============================================================

THRESHOLD = 128

MIN_LINE_LENGTH = 30
MIN_LINE_WIDTH = 3
MAX_LINE_WIDTH = 80

# How far a branch must extend outside the normal line.
JUNCTION_EXTENSION = 10

# Maximum distance from the main line to inspect.
JUNCTION_RADIUS = 60

# Minimum amount of branch pixels.
MIN_BRANCH_PIXELS = 8

# Confirmation.
CROSSING_CONFIRM_FRAMES = 3
CROSSING_COOLDOWN = 1.0


class LineTracker:

    def __init__(self):
        self.pending_crossing = None
        self.pending_frames = 0
        self.last_confirmed_time = 0.0

    # ============================================================
    # THRESHOLD
    # ============================================================

    def threshold_image(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        _, binary = cv2.threshold(
            gray,
            THRESHOLD,
            255,
            cv2.THRESH_BINARY_INV
        )

        return binary

    # ============================================================
    # LONGEST CONTINUOUS RUN
    # ============================================================

    @staticmethod
    def longest_run(values):
        values = np.asarray(values, dtype=np.bool_)

        if not np.any(values):
            return 0

        padded = np.concatenate(
            (
                [False],
                values,
                [False]
            )
        )

        changes = np.flatnonzero(
            padded[1:] != padded[:-1]
        )

        lengths = (
            changes[1::2]
            - changes[::2]
        )

        if lengths.size == 0:
            return 0

        return int(lengths.max())

    # ============================================================
    # FIND VERTICAL
    # ============================================================

    def find_vertical(self, binary):

        h, w = binary.shape

        # Number of black pixels in every column.
        counts = np.count_nonzero(
            binary,
            axis=0
        )

        x = int(np.argmax(counts))

        strength = int(counts[x])

        if strength < MIN_LINE_LENGTH:
            return None

        # Require a genuinely continuous vertical line.
        run = self.longest_run(
            binary[:, x] > 0
        )

        if run < MIN_LINE_LENGTH:
            return None

        # --------------------------------------------------------
        # Estimate normal width.
        # Use a few rows near the top where a junction is unlikely.
        # --------------------------------------------------------

        sample_y = h // 4

        y0 = max(
            0,
            sample_y - 3
        )

        y1 = min(
            h,
            sample_y + 4
        )

        sample = binary[y0:y1] > 0

        width_profile = np.count_nonzero(
            sample,
            axis=0
        )

        occupied = (
            width_profile
            >= max(
                1,
                (y1 - y0) // 2
            )
        )

        if occupied[x]:

            left = x

            while (
                left > 0
                and occupied[left - 1]
            ):
                left -= 1

            right = x

            while (
                right < w - 1
                and occupied[right + 1]
            ):
                right += 1

        else:

            left = x
            right = x

        width = right - left + 1

        if width < MIN_LINE_WIDTH:
            width = MIN_LINE_WIDTH

        if width > MAX_LINE_WIDTH:
            return None

        return {
            "center": x,
            "left": left,
            "right": right,
            "width": width,
            "run": run,
            "strength": strength,
        }

    # ============================================================
    # FIND HORIZONTAL
    # ============================================================

    def find_horizontal(self, binary):

        h, w = binary.shape

        counts = np.count_nonzero(
            binary,
            axis=1
        )

        y = int(np.argmax(counts))

        strength = int(counts[y])

        if strength < MIN_LINE_LENGTH:
            return None

        run = self.longest_run(
            binary[y, :] > 0
        )

        if run < MIN_LINE_LENGTH:
            return None

        # Estimate normal thickness.
        sample_x = w // 4

        x0 = max(
            0,
            sample_x - 3
        )

        x1 = min(
            w,
            sample_x + 4
        )

        sample = binary[:, x0:x1] > 0

        height_profile = np.count_nonzero(
            sample,
            axis=1
        )

        occupied = (
            height_profile
            >= max(
                1,
                (x1 - x0) // 2
            )
        )

        if occupied[y]:

            top = y

            while (
                top > 0
                and occupied[top - 1]
            ):
                top -= 1

            bottom = y

            while (
                bottom < h - 1
                and occupied[bottom + 1]
            ):
                bottom += 1

        else:

            top = y
            bottom = y

        height = bottom - top + 1

        if height < MIN_LINE_WIDTH:
            height = MIN_LINE_WIDTH

        if height > MAX_LINE_WIDTH:
            return None

        return {
            "center": y,
            "top": top,
            "bottom": bottom,
            "height": height,
            "run": run,
            "strength": strength,
        }

    # ============================================================
    # VERTICAL JUNCTION
    #
    #        |
    #        |
    # --------+       LEFT
    #        |
    #
    #        |
    #        |
    #        +-------- RIGHT
    #        |
    #
    #        |
    # --------+-------- CROSS
    #        |
    #
    #        |
    #        |
    #        |        STRAIGHT
    #
    # ============================================================

    def detect_vertical(
        self,
        binary,
        line
    ):

        h, w = binary.shape

        x = line["center"]

        half_width = max(
            1,
            line["width"] // 2
        )

        normal_left = max(
            0,
            x - half_width
        )

        normal_right = min(
            w - 1,
            x + half_width
        )

        radius = min(
            JUNCTION_RADIUS,
            x,
            w - x - 1
        )

        if radius <= JUNCTION_EXTENSION:
            return "STRAIGHT", None

        local_left = x - radius
        local_right = x + radius + 1

        patch = (
            binary[
                :,
                local_left:local_right
            ] > 0
        )

        normal_l = (
            normal_left
            - local_left
        )

        normal_r = (
            normal_right
            - local_left
        )

        # --------------------------------------------------------
        # Pixels beyond normal line on each side.
        # --------------------------------------------------------

        left_part = patch[:, :normal_l]

        right_part = patch[:, normal_r + 1:]

        if left_part.shape[1] > 0:

            left_extension = np.count_nonzero(
                left_part,
                axis=1
            )

        else:

            left_extension = np.zeros(
                h,
                dtype=np.int32
            )

        if right_part.shape[1] > 0:

            right_extension = np.count_nonzero(
                right_part,
                axis=1
            )

        else:

            right_extension = np.zeros(
                h,
                dtype=np.int32
            )

        # --------------------------------------------------------
        # Find rows containing an actual branch.
        # --------------------------------------------------------

        left_candidates = (
            left_extension
            >= MIN_BRANCH_PIXELS
        )

        right_candidates = (
            right_extension
            >= MIN_BRANCH_PIXELS
        )

        candidates = (
            left_candidates
            | right_candidates
        )

        ys = np.flatnonzero(candidates)

        if ys.size == 0:
            return "STRAIGHT", None

        # --------------------------------------------------------
        # Find largest group of nearby candidate rows.
        # --------------------------------------------------------

        breaks = np.flatnonzero(
            np.diff(ys) > 2
        )

        starts = np.r_[
            0,
            breaks + 1
        ]

        ends = np.r_[
            breaks,
            len(ys) - 1
        ]

        best_start = None
        best_end = None
        best_score = -1

        for start, end in zip(
            starts,
            ends
        ):

            a = int(ys[start])
            b = int(ys[end])

            score = max(
                int(
                    left_extension[
                        a:b + 1
                    ].max()
                ),
                int(
                    right_extension[
                        a:b + 1
                    ].max()
                )
            )

            if score > best_score:

                best_score = score
                best_start = a
                best_end = b

        junction_y = (
            best_start
            + best_end
        ) // 2

        # --------------------------------------------------------
        # Check several rows around junction.
        # --------------------------------------------------------

        sample_radius = 5

        y0 = max(
            0,
            junction_y - sample_radius
        )

        y1 = min(
            h,
            junction_y + sample_radius + 1
        )

        left_score = np.count_nonzero(
            left_extension[y0:y1]
            >= MIN_BRANCH_PIXELS
        )

        right_score = np.count_nonzero(
            right_extension[y0:y1]
            >= MIN_BRANCH_PIXELS
        )

        required = 3

        left = (
            left_score >= required
        )

        right = (
            right_score >= required
        )

        if left and right:
            return "CROSS", junction_y

        if left:
            return "LEFT", junction_y

        if right:
            return "RIGHT", junction_y

        return "STRAIGHT", None

    # ============================================================
    # HORIZONTAL JUNCTION
    #
    # Horizontal is ONLY used when NO vertical line exists.
    #
    #       |
    # -------+-------     UP
    #
    # -------+-------
    #       |             DOWN
    #
    #       |
    # -------+-------
    #       |             CROSS
    #
    # ============================================================

    def detect_horizontal(
        self,
        binary,
        line
    ):

        h, w = binary.shape

        y = line["center"]

        half_height = max(
            1,
            line["height"] // 2
        )

        normal_top = max(
            0,
            y - half_height
        )

        normal_bottom = min(
            h - 1,
            y + half_height
        )

        radius = min(
            JUNCTION_RADIUS,
            y,
            h - y - 1
        )

        if radius <= JUNCTION_EXTENSION:
            return "STRAIGHT", None

        local_top = y - radius
        local_bottom = y + radius + 1

        patch = (
            binary[
                local_top:local_bottom,
                :
            ] > 0
        )

        normal_t = (
            normal_top
            - local_top
        )

        normal_b = (
            normal_bottom
            - local_top
        )

        top_part = patch[:normal_t]

        bottom_part = patch[
            normal_b + 1:
        ]

        if top_part.shape[0] > 0:

            top_extension = np.count_nonzero(
                top_part,
                axis=0
            )

        else:

            top_extension = np.zeros(
                w,
                dtype=np.int32
            )

        if bottom_part.shape[0] > 0:

            bottom_extension = np.count_nonzero(
                bottom_part,
                axis=0
            )

        else:

            bottom_extension = np.zeros(
                w,
                dtype=np.int32
            )

        top_candidates = (
            top_extension
            >= MIN_BRANCH_PIXELS
        )

        bottom_candidates = (
            bottom_extension
            >= MIN_BRANCH_PIXELS
        )

        candidates = (
            top_candidates
            | bottom_candidates
        )

        xs = np.flatnonzero(candidates)

        if xs.size == 0:
            return "STRAIGHT", None

        breaks = np.flatnonzero(
            np.diff(xs) > 2
        )

        starts = np.r_[
            0,
            breaks + 1
        ]

        ends = np.r_[
            breaks,
            len(xs) - 1
        ]

        best_start = None
        best_end = None
        best_score = -1

        for start, end in zip(
            starts,
            ends
        ):

            a = int(xs[start])
            b = int(xs[end])

            score = max(
                int(
                    top_extension[
                        a:b + 1
                    ].max()
                ),
                int(
                    bottom_extension[
                        a:b + 1
                    ].max()
                )
            )

            if score > best_score:

                best_score = score
                best_start = a
                best_end = b

        junction_x = (
            best_start
            + best_end
        ) // 2

        sample_radius = 5

        x0 = max(
            0,
            junction_x - sample_radius
        )

        x1 = min(
            w,
            junction_x + sample_radius + 1
        )

        top_score = np.count_nonzero(
            top_extension[x0:x1]
            >= MIN_BRANCH_PIXELS
        )

        bottom_score = np.count_nonzero(
            bottom_extension[x0:x1]
            >= MIN_BRANCH_PIXELS
        )

        required = 3

        top = (
            top_score >= required
        )

        bottom = (
            bottom_score >= required
        )

        if top and bottom:
            return "CROSS", junction_x

        if top:
            return "UP", junction_x

        if bottom:
            return "DOWN", junction_x

        return "STRAIGHT", None

    # ============================================================
    # DEBOUNCE
    # ============================================================

    def debounce_crossing(
        self,
        crossing
    ):

        now = time.monotonic()

        if crossing in (
            None,
            "NONE",
            "STRAIGHT"
        ):

            self.pending_crossing = None
            self.pending_frames = 0

            return None

        if (
            now - self.last_confirmed_time
            < CROSSING_COOLDOWN
        ):
            return None

        if (
            crossing
            == self.pending_crossing
        ):

            self.pending_frames += 1

        else:

            self.pending_crossing = crossing
            self.pending_frames = 1

        if (
            self.pending_frames
            >= CROSSING_CONFIRM_FRAMES
        ):

            confirmed = (
                self.pending_crossing
            )

            self.pending_crossing = None
            self.pending_frames = 0

            self.last_confirmed_time = now

            return confirmed

        return None

    # ============================================================
    # MAIN PROCESS
    # ============================================================

    def process(self, frame):

        binary = self.threshold_image(frame)

        # ========================================================
        # IMPORTANT:
        #
        # VERTICAL ALWAYS HAS PRIORITY.
        #
        # We only calculate/use horizontal if vertical does NOT
        # exist.
        # ========================================================

        vertical = self.find_vertical(
            binary
        )

        horizontal = None

        if vertical is not None:

            orientation = "vertical"
            line = vertical

            crossing, junction_position = (
                self.detect_vertical(
                    binary,
                    vertical
                )
            )

        else:

            horizontal = self.find_horizontal(
                binary
            )

            if horizontal is not None:

                orientation = "horizontal"
                line = horizontal

                crossing, junction_position = (
                    self.detect_horizontal(
                        binary,
                        horizontal
                    )
                )

            else:

                orientation = None
                line = None

                crossing = "NONE"
                junction_position = None

        # ========================================================
        # CONFIRM
        # ========================================================

        confirmed = self.debounce_crossing(
            crossing
        )

        # ========================================================
        # DISPLAY
        # ========================================================

        display = frame.copy()

        if (
            orientation == "vertical"
            and line is not None
        ):

            x = line["center"]

            cv2.line(
                display,
                (x, 0),
                (
                    x,
                    display.shape[0]
                ),
                (0, 255, 0),
                2
            )

            if junction_position is not None:

                y = junction_position

                cv2.circle(
                    display,
                    (x, y),
                    12,
                    (0, 0, 255),
                    2
                )

        elif (
            orientation == "horizontal"
            and line is not None
        ):

            y = line["center"]

            cv2.line(
                display,
                (0, y),
                (
                    display.shape[1],
                    y
                ),
                (0, 255, 0),
                2
            )

            if junction_position is not None:

                x = junction_position

                cv2.circle(
                    display,
                    (x, y),
                    12,
                    (0, 0, 255),
                    2
                )

        # Text
        cv2.putText(
            display,
            "ORIENTATION: "
            + (
                orientation
                or "NONE"
            ),
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.putText(
            display,
            "DETECTION: "
            + crossing,
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

        if confirmed is not None:

            cv2.putText(
                display,
                "CONFIRMED: "
                + confirmed,
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        # ========================================================
        # RESULT
        # ========================================================

        return {
            "display": display,
            "binary": binary,

            "orientation": orientation,

            "line_center": (
                line["center"]
                if line is not None
                else None
            ),

            "line_left": (
                line["left"]
                if (
                    line is not None
                    and orientation == "vertical"
                )
                else None
            ),

            "line_right": (
                line["right"]
                if (
                    line is not None
                    and orientation == "vertical"
                )
                else None
            ),

            "error": (
                (
                    line["center"]
                    - frame.shape[1] // 2
                )
                if (
                    line is not None
                    and orientation == "vertical"
                )

                else (
                    line["center"]
                    - frame.shape[0] // 2
                )
                if (
                    line is not None
                    and orientation == "horizontal"
                )

                else None
            ),

            "crossing": crossing,

            "confirmed_crossing": confirmed,
        }


# ============================================================
# STANDALONE TEST
# ============================================================

if __name__ == "__main__":

    from camera import Camera

    tracker = LineTracker()
    camera = Camera()

    camera.start()

    print("Vision test running.")

    try:

        while True:

            frame = camera.get_frame()

            if frame is None:
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

    except KeyboardInterrupt:
        pass

    finally:
        camera.stop()
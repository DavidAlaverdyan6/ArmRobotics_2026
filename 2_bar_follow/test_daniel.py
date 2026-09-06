import time
import cv2
import numpy as np
from camera import Camera


# ============================================================
# CONFIGURATION
# ============================================================

WIDTH = 800
HEIGHT = 600

MIN_OBJECT_AREA = 50

LOWER_COLOR = np.array([0, 150, 0], dtype=np.uint8)
UPPER_COLOR = np.array([125, 255, 125], dtype=np.uint8)

BENCHMARK_FRAMES = 300


# ============================================================
# CAMERA
# ============================================================

camera = Camera(
    width=WIDTH,
    height=HEIGHT
)

camera.start()


print("==============================")
print("OBJECT DETECTOR COMPARISON")
print("==============================")
print(f"Resolution: {WIDTH}x{HEIGHT}")
print(f"Frames: {BENCHMARK_FRAMES}")
print()


# ============================================================
# TIMINGS
# ============================================================

current_times = []
candidate_times = []

current_components = []
candidate_components = []

current_green = []
candidate_green = []


try:

    # Wait for camera
    while camera.get_frame() is None:
        time.sleep(0.01)


    for _ in range(BENCHMARK_FRAMES):

        frame = camera.get_frame()

        if frame is None:
            continue


        # ====================================================
        # SAME MASK FOR BOTH
        # ====================================================

        mask = cv2.inRange(
            frame,
            LOWER_COLOR,
            UPPER_COLOR
        )


        # ====================================================
        # CURRENT VERSION
        # connectedComponentsWithStats()
        # ====================================================

        t0 = time.perf_counter()

        num_labels, labels, stats, centroids = \
            cv2.connectedComponentsWithStats(
                mask,
                connectivity=8
            )

        component_time = time.perf_counter() - t0

        t0 = time.perf_counter()

        objects = []

        for label in range(1, num_labels):

            area = stats[label, cv2.CC_STAT_AREA]

            if area < MIN_OBJECT_AREA:
                continue

            x = stats[label, cv2.CC_STAT_LEFT]
            y = stats[label, cv2.CC_STAT_TOP]
            width = stats[label, cv2.CC_STAT_WIDTH]
            height = stats[label, cv2.CC_STAT_HEIGHT]

            objects.append(
                (x, y, width, height, area)
            )

        filter_time = time.perf_counter() - t0


        t0 = time.perf_counter()

        result = frame.copy()
        result[mask != 0] = (0, 255, 0)

        for x, y, width, height, area in objects:

            cv2.rectangle(
                result,
                (x, y),
                (x + width - 1, y + height - 1),
                (27, 200, 27),
                3
            )

        green_draw_time = time.perf_counter() - t0

        current_total = (
            component_time +
            filter_time +
            green_draw_time
        )

        current_times.append(current_total)
        current_components.append(component_time)
        current_green.append(green_draw_time)


        # ====================================================
        # CANDIDATE VERSION
        # connectedComponents()
        # ====================================================

        t0 = time.perf_counter()

        num_labels, labels = cv2.connectedComponents(
            mask,
            connectivity=8
        )

        component_time = time.perf_counter() - t0


        t0 = time.perf_counter()

        objects = []

        for label in range(1, num_labels):

            ys, xs = np.where(labels == label)

            area = len(xs)

            if area < MIN_OBJECT_AREA:
                continue

            x = xs.min()
            y = ys.min()

            width = xs.max() - x + 1
            height = ys.max() - y + 1

            objects.append(
                (x, y, width, height, area)
            )

        filter_time = time.perf_counter() - t0


        t0 = time.perf_counter()

        result = frame.copy()
        result[mask != 0] = (0, 255, 0)

        for x, y, width, height, area in objects:

            cv2.rectangle(
                result,
                (x, y),
                (x + width - 1, y + height - 1),
                (27, 200, 27),
                3
            )

        green_draw_time = time.perf_counter() - t0


        candidate_total = (
            component_time +
            filter_time +
            green_draw_time
        )

        candidate_times.append(candidate_total)
        candidate_components.append(component_time)
        candidate_green.append(green_draw_time)


finally:
    camera.stop()


# ============================================================
# RESULTS
# ============================================================

current_ms = np.array(current_times) * 1000
candidate_ms = np.array(candidate_times) * 1000

current_component_ms = np.array(current_components) * 1000
candidate_component_ms = np.array(candidate_components) * 1000

current_green_ms = np.array(current_green) * 1000
candidate_green_ms = np.array(candidate_green) * 1000


print()
print("==============================")
print("CURRENT VERSION")
print("==============================")

print(
    f"COMPONENTS: {np.mean(current_component_ms):.3f} ms"
)

print(
    f"GREEN/DRAW: {np.mean(current_green_ms):.3f} ms"
)

print(
    f"TOTAL:      {np.mean(current_ms):.3f} ms"
)

print(
    f"FPS:        {1000 / np.mean(current_ms):.1f}"
)


print()
print("==============================")
print("CANDIDATE VERSION")
print("==============================")

print(
    f"COMPONENTS: {np.mean(candidate_component_ms):.3f} ms"
)

print(
    f"GREEN/DRAW: {np.mean(candidate_green_ms):.3f} ms"
)

print(
    f"TOTAL:      {np.mean(candidate_ms):.3f} ms"
)

print(
    f"FPS:        {1000 / np.mean(candidate_ms):.1f}"
)


print()
print("==============================")
print("COMPARISON")
print("==============================")

current_avg = np.mean(current_ms)
candidate_avg = np.mean(candidate_ms)

speedup = current_avg / candidate_avg

print(
    f"Current:   {current_avg:.3f} ms"
)

print(
    f"Candidate: {candidate_avg:.3f} ms"
)

print(
    f"Speedup:   {speedup:.2f}x"
)

if candidate_avg < current_avg:
    print("Candidate is FASTER.")
else:
    print("Current version is FASTER.")

print("==============================")
import time
import cv2
import numpy as np

WIDTH = 800
HEIGHT = 600

RED = 1
YELLOW = 2
GREEN = 3
BLUE = 4

MIN_OBJECT_AREA = 700


# ============================================================
# REALISTIC COLOR MAP
# ============================================================

color_map = np.zeros(
    (HEIGHT, WIDTH),
    dtype=np.uint8
)

# Realistic objects
cv2.rectangle(color_map, (80, 120), (220, 280), RED, -1)
cv2.circle(color_map, (350, 180), 70, BLUE, -1)
cv2.rectangle(color_map, (500, 100), (650, 260), GREEN, -1)
cv2.circle(color_map, (180, 450), 80, YELLOW, -1)

# Two DIFFERENT colors touching
cv2.rectangle(color_map, (400, 400), (520, 520), RED, -1)
cv2.rectangle(color_map, (520, 400), (640, 520), BLUE, -1)


# ============================================================
# DETECTOR
# ============================================================

def detect_objects(color_map):

    objects = []

    for color_id in (RED, YELLOW, GREEN, BLUE):

        mask = (color_map == color_id).astype(np.uint8) * 255

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:

            area = cv2.contourArea(contour)

            if area < MIN_OBJECT_AREA:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            center_x = x + w // 2
            center_y = y + h // 2

            objects.append((
                color_id,
                x, y, w, h,
                area,
                center_x, center_y
            ))

    return objects


# ============================================================
# WARM-UP
# ============================================================

for _ in range(10):
    detect_objects(color_map)


# ============================================================
# BENCHMARK
# ============================================================

ITERATIONS = 100

start = time.perf_counter()

for _ in range(ITERATIONS):
    detect_objects(color_map)

elapsed = time.perf_counter() - start

ms_per_frame = elapsed / ITERATIONS * 1000
fps = ITERATIONS / elapsed


print()
print("========================================")
print(" REALISTIC FOUR-COLOR BENCHMARK")
print("========================================")
print(f"Resolution:     {WIDTH}x{HEIGHT}")
print(f"Objects:        6")
print(f"Iterations:     {ITERATIONS}")
print(f"Total time:     {elapsed:.3f} s")
print(f"Time/frame:     {ms_per_frame:.3f} ms")
print(f"FPS:            {fps:.1f}")
print("========================================")
import time
import cv2
import numpy as np
from camera import Camera


# ============================================================
# CONFIG
# ============================================================

WIDTH = 800
HEIGHT = 600

FRAMES = 100


# ============================================================
# CAMERA
# ============================================================

cam = Camera(width=WIDTH, height=HEIGHT)
cam.start()

frames = []

print("Collecting frames...")

while len(frames) < FRAMES:
    frame = cam.get_frame()

    if frame is not None:
        frames.append(frame.copy())

    time.sleep(0.001)

cam.stop()

print(f"Collected {len(frames)} frames.")


# ============================================================
# 1. CURRENT NUMPY BGR CLASSIFIER
# ============================================================

def classify_numpy(frame):
    b = frame[:, :, 0].astype(np.uint16)
    g = frame[:, :, 1].astype(np.uint16)
    r = frame[:, :, 2].astype(np.uint16)

    color_map = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)

    # YELLOW = 2
    yellow = (
        (r > 120) &
        (g > 100) &
        (b * 100 < np.minimum(r, g) * 65)
    )
    color_map[yellow] = 2

    # RED = 1
    red = (
        (r * 100 > g * 135) &
        (r * 100 > b * 135)
    )
    color_map[red] = 1

    # GREEN = 3
    green = (
        (g * 100 > r * 125) &
        (g * 100 > b * 115)
    )
    color_map[green] = 3

    # BLUE = 4
    blue = (
        (b * 100 > r * 125) &
        (b * 100 > g * 115)
    )
    color_map[blue] = 4

    return color_map


# ============================================================
# 2. DOMINANT-CHANNEL-GATED NUMPY
# ============================================================

def classify_dominant(frame):
    b = frame[:, :, 0].astype(np.uint16)
    g = frame[:, :, 1].astype(np.uint16)
    r = frame[:, :, 2].astype(np.uint16)

    color_map = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)

    # --------------------------------------------------------
    # YELLOW
    # Yellow is special: R and G are both strong.
    # --------------------------------------------------------

    yellow = (
        (r > 120) &
        (g > 100) &
        (b * 100 < np.minimum(r, g) * 65)
    )
    color_map[yellow] = 2

    # --------------------------------------------------------
    # RED
    # Only evaluate red where R is dominant.
    # --------------------------------------------------------

    red_candidate = (r > g) & (r > b)

    red = red_candidate & (
        (r * 100 > g * 135) &
        (r * 100 > b * 135)
    )

    color_map[red] = 1

    # --------------------------------------------------------
    # GREEN
    # Only evaluate green where G is dominant.
    # --------------------------------------------------------

    green_candidate = (g > r) & (g > b)

    green = green_candidate & (
        (g * 100 > r * 125) &
        (g * 100 > b * 115)
    )

    color_map[green] = 3

    # --------------------------------------------------------
    # BLUE
    # Only evaluate blue where B is dominant.
    # --------------------------------------------------------

    blue_candidate = (b > r) & (b > g)

    blue = blue_candidate & (
        (b * 100 > r * 125) &
        (b * 100 > g * 115)
    )

    color_map[blue] = 4

    return color_map


# ============================================================
# 3. OPENCV BGR CLASSIFIER
# ============================================================

def classify_opencv(frame):
    b, g, r = cv2.split(frame)

    color_map = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)

    # --------------------------------------------------------
    # YELLOW = 2
    # --------------------------------------------------------

    yellow1 = cv2.inRange(
        frame,
        np.array([0, 101, 121], dtype=np.uint8),
        np.array([255, 255, 255], dtype=np.uint8)
    )

    yellow2 = (
        (b.astype(np.uint16) * 100 <
         np.minimum(
             r.astype(np.uint16),
             g.astype(np.uint16)
         ) * 65)
    ).astype(np.uint8) * 255

    yellow = cv2.bitwise_and(yellow1, yellow2)

    color_map[yellow != 0] = 2

    # --------------------------------------------------------
    # RED = 1
    # --------------------------------------------------------

    r16 = r.astype(np.uint16)
    g16 = g.astype(np.uint16)
    b16 = b.astype(np.uint16)

    red_rg = (
        (r16 * 100 > g16 * 135)
    ).astype(np.uint8) * 255

    red_rb = (
        (r16 * 100 > b16 * 135)
    ).astype(np.uint8) * 255

    red = cv2.bitwise_and(red_rg, red_rb)

    color_map[red != 0] = 1

    # --------------------------------------------------------
    # GREEN = 3
    # --------------------------------------------------------

    green_gr = (
        (g16 * 100 > r16 * 125)
    ).astype(np.uint8) * 255

    green_gb = (
        (g16 * 100 > b16 * 115)
    ).astype(np.uint8) * 255

    green = cv2.bitwise_and(green_gr, green_gb)

    color_map[green != 0] = 3

    # --------------------------------------------------------
    # BLUE = 4
    # --------------------------------------------------------

    blue_br = (
        (b16 * 100 > r16 * 125)
    ).astype(np.uint8) * 255

    blue_bg = (
        (b16 * 100 > g16 * 115)
    ).astype(np.uint8) * 255

    blue = cv2.bitwise_and(blue_br, blue_bg)

    color_map[blue != 0] = 4

    return color_map


# ============================================================
# WARMUP
# ============================================================

print("\nWarming up...")

for frame in frames[:5]:
    classify_numpy(frame)
    classify_dominant(frame)
    classify_opencv(frame)


# ============================================================
# BENCHMARK FUNCTION
# ============================================================

def benchmark(name, function):
    start = time.perf_counter()

    for frame in frames:
        function(frame)

    elapsed = time.perf_counter() - start

    avg_ms = (elapsed / FRAMES) * 1000
    fps = FRAMES / elapsed

    print(
        f"{name:<25}"
        f"{avg_ms:>10.3f} ms"
        f"{fps:>10.1f} FPS"
    )

    return avg_ms, fps


# ============================================================
# RUN
# ============================================================

print("\n==============================")
print("BGR CLASSIFIER BENCHMARK")
print("==============================")

numpy_ms, numpy_fps = benchmark(
    "CURRENT NUMPY",
    classify_numpy
)

dominant_ms, dominant_fps = benchmark(
    "DOMINANT NUMPY",
    classify_dominant
)

opencv_ms, opencv_fps = benchmark(
    "OPENCV BGR",
    classify_opencv
)

print("==============================")
print("RESULTS")
print("==============================")

print(f"CURRENT NUMPY:   {numpy_ms:.3f} ms  ({numpy_fps:.1f} FPS)")
print(f"DOMINANT NUMPY:  {dominant_ms:.3f} ms  ({dominant_fps:.1f} FPS)")
print(f"OPENCV BGR:      {opencv_ms:.3f} ms  ({opencv_fps:.1f} FPS)")

print("==============================")
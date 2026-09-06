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

# BGR
LOWER_COLOR = np.array([0, 150, 0], dtype=np.uint8)
UPPER_COLOR = np.array([125, 255, 125], dtype=np.uint8)

OUTPUT_COLOR = (0, 255, 0)
BOX_COLOR = (27, 200, 27)


# ============================================================
# CAMERA
# ============================================================

ContCam = Camera(
    width=WIDTH,
    height=HEIGHT
)

ContCam.start()

print("==============================")
print("MULTI-OBJECT COLOR DETECTOR")
print("==============================")


try:
    while True:

        frame = ContCam.get_frame()

        if frame is None:
            time.sleep(0.001)
            continue


        # ====================================================
        # COLOR MASK
        # ====================================================

        mask = cv2.inRange(
            frame,
            LOWER_COLOR,
            UPPER_COLOR
        )


        # ====================================================
        # SEPARATE OBJECTS
        # ====================================================

        num_labels, labels = cv2.connectedComponents(
            mask,
            connectivity=8
        )


        # ====================================================
        # RESULT
        # ====================================================

        result = frame.copy()

        # Turn matching pixels green
        result[mask != 0] = OUTPUT_COLOR


        # ====================================================
        # FIND EVERY OBJECT
        # ====================================================

        objects = []

        for label in range(1, num_labels):

            ys, xs = np.where(labels == label)

            area = len(xs)

            # Ignore tiny objects
            if area < MIN_OBJECT_AREA:
                continue

            x = xs.min()
            y = ys.min()

            width = xs.max() - x + 1
            height = ys.max() - y + 1


            objects.append({
                "x": int(x),
                "y": int(y),
                "width": int(width),
                "height": int(height),
                "area": int(area)
            })


            # Draw bounding box
            cv2.rectangle(
                result,
                (x, y),
                (x + width - 1, y + height - 1),
                BOX_COLOR,
                3
            )


            cv2.putText(
                result,
                str(len(objects)),
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                BOX_COLOR,
                2
            )


        # ====================================================
        # PRINT OBJECT DATA
        # ====================================================

        print(f"Objects: {len(objects)}")

        for i, obj in enumerate(objects):

            print(
                f"  Object {i + 1}: "
                f"x={obj['x']} "
                f"y={obj['y']} "
                f"size={obj['width']}x{obj['height']} "
                f"area={obj['area']}"
            )


        ContCam.set_processed_frame(result)


except KeyboardInterrupt:
    pass

finally:
    ContCam.stop()
import cv2
import numpy as np

Colors = {
    1: "R",
    2: "Y",
    3: "G",
    4: "B"
    }

class ColorDetector:

    RED = 1
    YELLOW = 2
    GREEN = 3
    BLUE = 4

    def __init__(
        self,
        min_object_area=700,
        min_brightness=60,
        min_channel_diff=25
    ):
        self.min_object_area = min_object_area
        self.min_brightness = min_brightness
        self.min_channel_diff = min_channel_diff

    def classify_colors(self, frame):
        b = frame[:, :, 0].astype(np.uint16)
        g = frame[:, :, 1].astype(np.uint16)
        r = frame[:, :, 2].astype(np.uint16)

        height, width = frame.shape[:2]
        color_map = np.zeros((height, width), dtype=np.uint8)

        max_channel = np.maximum(np.maximum(r, g), b)
        min_channel = np.minimum(np.minimum(r, g), b)

        bright = max_channel >= self.min_brightness
        colorful = (max_channel - min_channel) >= self.min_channel_diff
        valid = bright & colorful

        yellow = valid & (
            (r > 120) &
            (g > 100) &
            (b * 100 < np.minimum(r, g) * 65)
        )
        color_map[yellow] = self.YELLOW

        red = valid & (
            (r * 100 > g * 135) &
            (r * 100 > b * 135)
        )
        color_map[red] = self.RED

        green = valid & (
            (g * 100 > r * 115) &
            (g * 100 > b * 110)
        )
        color_map[green] = self.GREEN

        blue = valid & (
            (b * 100 > r * 125) &
            (b * 100 > g * 115)
        )
        color_map[blue] = self.BLUE

        return color_map

    def find_objects(self, color_map):
        mask = (color_map != 0).astype(np.uint8) * 255

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        objects = []

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < self.min_object_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            px_x, px_y = contour[0][0]
            color = int(color_map[px_y, px_x])

            if color == 0:
                continue

            center = (
                x + w // 2,
                y + h // 2
            )

            objects.append({
                "color": Colors[color],
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "center": center,
                "area": int(area)
            })

        return objects

    def draw_objects(self, frame, objects, color_map):
        result = frame.copy()

        result[color_map == self.RED] = (0, 0, 255)
        result[color_map == self.YELLOW] = (0, 255, 255)
        result[color_map == self.GREEN] = (0, 255, 0)
        result[color_map == self.BLUE] = (255, 0, 0)

        for i, obj in enumerate(objects):
            x = obj["x"]
            y = obj["y"]
            w = obj["w"]
            h = obj["h"]

            center_x, center_y = obj["center"]

            cv2.rectangle(
                result,
                (x, y),
                (x + w - 1, y + h - 1),
                (255, 255, 255),
                3
            )

            size = 6

            cv2.line(
                result,
                (center_x - size, center_y),
                (center_x + size, center_y),
                (255, 255, 255),
                2
            )

            cv2.line(
                result,
                (center_x, center_y - size),
                (center_x, center_y + size),
                (255, 255, 255),
                2
            )

            cv2.putText(
                result,
                str(i + 1),
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

        return result

    def process_frame(self, frame):
        color_map = self.classify_colors(frame)
        objects = self.find_objects(color_map)
        processed = self.draw_objects(frame, objects, color_map)

        return objects, processed
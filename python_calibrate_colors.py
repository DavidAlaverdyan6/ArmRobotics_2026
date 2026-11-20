#!/usr/bin/env python3
"""Sample the route camera and save OpenCV HSV ranges for red, green and blue."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from color_marker_detector import ContinuousCamera

COLORS = ("green", "red", "blue")
HUE_ANCHOR = {"green": 60, "red": 0, "blue": 120}
SAMPLE_FRAMES = 18
SAMPLE_BOX_SIZE = 12


def center_box(frame):
    height, width = frame.shape[:2]
    side = min(SAMPLE_BOX_SIZE, height, width)
    x = (width - side) // 2
    y = (height - side) // 2
    return x, y, x + side, y + side


def calculate_ranges(color, pixels):
    saturated = pixels[(pixels[:, 1] >= 55) & (pixels[:, 2] >= 45)]
    if len(saturated) < 0.75 * len(pixels):
        raise ValueError("Fill the center box with the colored surface.")
    anchor = HUE_ANCHOR[color]
    hue = (saturated[:, 0].astype(int) - anchor + 90) % 180 - 90
    low_h, high_h = np.percentile(hue, [5, 95])
    if abs(np.median(hue)) > 32 or high_h - low_h > 30:
        raise ValueError(f"Sample is too mixed or does not look {color}.")
    low_h = int(np.floor(low_h)) + anchor - 3
    high_h = int(np.ceil(high_h)) + anchor + 3
    low_s, low_v = np.floor(np.percentile(saturated[:, 1:], 5, axis=0)).astype(int)
    low_s = max(55, int(low_s) - 15)
    low_v = max(45, int(low_v) - 20)
    hue_ranges = [(low_h % 180, high_h % 180)]
    if hue_ranges[0][0] > hue_ranges[0][1]:
        hue_ranges = [(0, high_h % 180), (low_h % 180, 179)]
    return [
        {"lower": [start, low_s, low_v], "upper": [end, 255, 255]}
        for start, end in hue_ranges
    ]


def show_preview(camera, profiles, label):
    sequence = camera.sequence
    while True:
        result = camera.get_frame(after_sequence=sequence, timeout=2)
        if result is None:
            return
        sequence, frame = result
        x0, y0, x1, y1 = center_box(frame)
        hsv = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
        matches = []
        for color, ranges in profiles.copy().items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for bounds in ranges:
                mask |= cv2.inRange(hsv, tuple(bounds["lower"]), tuple(bounds["upper"]))
            if cv2.countNonZero(mask) * 2 > mask.size:
                matches.append(color)
        preview = frame.copy()
        cv2.rectangle(preview, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 255), 1)
        if profiles:
            result = matches[0] if len(matches) == 1 else "ambiguous" if matches else "none"
            cv2.putText(preview, f"Center: {result}", (x1 + 6, y0),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        camera.publish_task_frame(preview, label[0])


def capture_sample(camera):
    sequence = camera.sequence
    samples = []
    for _ in range(SAMPLE_FRAMES):
        result = camera.get_frame(after_sequence=sequence, timeout=2)
        if result is None:
            raise RuntimeError("No new camera frames.")
        sequence, frame = result
        x0, y0, x1, y1 = center_box(frame)
        hsv = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
        samples.append(hsv.reshape(-1, 3))
    return np.concatenate(samples)


def main():
    from threading import Thread

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--output", type=Path, default=Path("color_calibration.json"))
    args = parser.parse_args()
    camera = ContinuousCamera(port=args.port)
    profiles = {}
    label = ["Place the sample inside the center box"]
    preview = Thread(target=show_preview, args=(camera, profiles, label), daemon=True)
    camera.start()
    try:
        preview.start()
        for color in COLORS:
            label[0] = f"Place {color} in the center box"
            while True:
                input(f"Fill the box with {color}, then press Enter: ")
                try:
                    profiles[color] = calculate_ranges(color, capture_sample(camera))
                except ValueError as exc:
                    print(exc)
                    continue
                print(f"{color}: {profiles[color]}")
                break
        label[0] = "Check the color in the center box"
        input("Check Task camera. Press Enter to save, or Ctrl+C to cancel: ")
        with args.output.open("x", encoding="utf-8") as output:
            json.dump(profiles, output, indent=2)
            output.write("\n")
        print(f"Saved {args.output}")
    finally:
        camera.stop()
        preview.join(timeout=3)


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print("\nCancelled.")
    except (OSError, RuntimeError) as exc:
        raise SystemExit(str(exc)) from exc

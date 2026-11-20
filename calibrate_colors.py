#!/usr/bin/env python3
"""Sample the route camera and save OpenCV HSV ranges for red, green and blue."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from color_marker_detector import ContinuousCamera

COLORS = ("green", "red", "blue")
SAMPLE_FRAMES = 18


def find_button(hsv):
    """Find a bright connected object under the center of the preview."""
    height, width = hsv.shape[:2]
    x, y = width // 2, height // 2
    value = hsv[:, :, 2]
    center_value = int(np.median(value[y - 2:y + 3, x - 2:x + 3]))
    if center_value < 150:
        return None
    for margin in (30, 18, 8):
        bright = cv2.inRange(value, max(150, center_value - margin), 255)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(bright)
        label = labels[y, x]
        if not 0 < label < count:
            continue
        left, top, w, h, area = stats[label]
        if (area < 100 or area > 0.20 * width * height or
                left == 0 or top == 0 or left + w == width or top + h == height):
            continue
        component = (labels[top:top + h, left:left + w] == label).astype(np.uint8)
        interior = cv2.erode(component, np.ones((3, 3), np.uint8),
                             borderType=cv2.BORDER_CONSTANT, borderValue=0)
        if cv2.countNonZero(interior) >= 50:
            return left, top, w, h, interior
    return None


def calculate_ranges(color, pixels):
    usable = pixels[(pixels[:, 1] >= 8) & (pixels[:, 2] >= 45)]
    if len(usable) < max(50, 0.15 * len(pixels)):
        raise ValueError(
            "The detected area has too little color. Aim at the lit button "
            "away from the white hotspot, or change the camera angle."
        )

    angles = usable[:, 0] * (2 * np.pi / 180)
    center = np.arctan2(np.sin(angles).mean(), np.cos(angles).mean())
    anchor = int(round(center * 180 / (2 * np.pi))) % 180
    hue = (usable[:, 0].astype(int) - anchor + 90) % 180 - 90
    low_h, high_h = np.percentile(hue, [5, 95])
    if high_h - low_h > 30:
        raise ValueError(f"{color}: the detected area has mixed hues. Hold the camera still.")

    low_h = (int(np.floor(low_h)) + anchor - 2) % 180
    high_h = (int(np.ceil(high_h)) + anchor + 2) % 180
    low_s, low_v = np.floor(np.percentile(usable[:, 1:], 5, axis=0)).astype(int)
    high_s = int(np.ceil(np.percentile(usable[:, 1], 95)))
    low_s = max(5, int(low_s) - 6)
    high_s = min(255, high_s + 15)
    low_v = max(45, int(low_v) - 20)
    hue_ranges = [(low_h, high_h)] if low_h <= high_h else [(0, high_h), (low_h, 179)]
    return [
        {"lower": [start, low_s, low_v], "upper": [end, high_s, 255]}
        for start, end in hue_ranges
    ]


def profiles_overlap(first, second):
    return any(
        all(max(a, b) <= min(c, d) for a, b, c, d in zip(
            left["lower"], right["lower"], left["upper"], right["upper"]
        ))
        for left in first for right in second
    )


def show_preview(camera, profiles, label):
    sequence = camera.sequence
    while True:
        result = camera.get_frame(after_sequence=sequence, timeout=2)
        if result is None:
            return
        sequence, frame = result
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        preview = frame.copy()
        height, width = hsv.shape[:2]
        cv2.drawMarker(preview, (width // 2, height // 2), (0, 255, 255),
                       markerSize=9, thickness=1)
        button = find_button(hsv)
        if button is not None:
            x, y, w, h, interior = button
            region = hsv[y:y + h, x:x + w]
            usable = interior & (region[:, :, 1] >= 8) & (region[:, :, 2] >= 45)
            matches = []
            for color, ranges in profiles.copy().items():
                mask = np.zeros(region.shape[:2], dtype=np.uint8)
                for bounds in ranges:
                    mask |= cv2.inRange(region, tuple(bounds["lower"]), tuple(bounds["upper"]))
                if 2 * cv2.countNonZero(mask & usable) > cv2.countNonZero(usable):
                    matches.append(color)
            cv2.rectangle(preview, (x, y), (x + w - 1, y + h - 1), (0, 255, 255), 2)
            result = matches[0] if len(matches) == 1 else "Button candidate"
            cv2.putText(preview, result, (x, max(20, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        camera.publish_task_frame(preview, label[0])


def capture_sample(camera):
    sequence = camera.sequence
    samples = []
    for _ in range(SAMPLE_FRAMES):
        result = camera.get_frame(after_sequence=sequence, timeout=2)
        if result is None:
            raise RuntimeError("No new camera frames.")
        sequence, frame = result
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        button = find_button(hsv)
        if button is None:
            continue
        x, y, w, h, interior = button
        samples.append(hsv[y:y + h, x:x + w][interior != 0])
    if len(samples) < SAMPLE_FRAMES // 2:
        raise ValueError("No stable button found. Aim the cross at it and wait for the outline.")
    return np.concatenate(samples)


def main():
    from threading import Thread

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--output", type=Path, default=Path("color_calibration.json"))
    args = parser.parse_args()
    camera = ContinuousCamera(port=args.port)
    profiles = {}
    label = ["Aim the cross at the button"]
    preview = Thread(target=show_preview, args=(camera, profiles, label), daemon=True)
    camera.start()
    try:
        preview.start()
        for color in COLORS:
            label[0] = f"Aim at the {color} button"
            while True:
                input(f"When the {color} button is outlined, press Enter: ")
                try:
                    ranges = calculate_ranges(color, capture_sample(camera))
                    conflict = next((
                        name for name, previous in profiles.items()
                        if profiles_overlap(previous, ranges)
                    ), None)
                    if conflict is not None:
                        raise ValueError(
                            f"{color} overlaps {conflict}. Change the camera angle "
                            "or aim away from the white hotspot and retry."
                        )
                    profiles[color] = ranges
                except ValueError as exc:
                    print(exc)
                    continue
                print(f"{color}: {profiles[color]}")
                break
        label[0] = "Check the detected button color"
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

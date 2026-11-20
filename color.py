#!/usr/bin/env python3
"""Stand-alone camera and arm test for the COLOR task."""

import time
from contextlib import ExitStack

from telemetrix import telemetrix

from color_marker_detector import ColorMarkerDetector, ContinuousCamera
from main_test import (
    COLOR_ARM_ANGLES,
    COLOR_MARKER_MIN_AREA,
    COLOR_MARKER_STABLE_FRAMES,
    SERIAL_PORT,
    calibrated_color_detector,
    create_color_rgb_indicator,
)
from roboarm import get_shared_arm

PREVIEW_PORT = 8000
EXPOSURE_US = 2_000
ANALOGUE_GAIN = 1.0
START_DELAY_SECONDS = 0
LED_SECONDS = 3.0


def main():
    with ExitStack() as cleanup:
        camera = ContinuousCamera(port=PREVIEW_PORT)
        camera.start()
        cleanup.callback(camera.stop)
        camera.camera.set_controls({
            "AeEnable": False,
            "ExposureTime": EXPOSURE_US,
            "AnalogueGain": ANALOGUE_GAIN,
        })
        camera.start_task("COLOR: positioning the arm")

        board = telemetrix.Telemetrix(com_port=SERIAL_PORT)
        cleanup.callback(board.shutdown)
        arm, _ = get_shared_arm(board)
        indicator = create_color_rgb_indicator()
        cleanup.callback(indicator.close)
        cleanup.callback(indicator.off)

        options = {
            "min_object_area": COLOR_MARKER_MIN_AREA,
            "stable_frames": COLOR_MARKER_STABLE_FRAMES,
        }
        detector = calibrated_color_detector(**options)
        if detector is None:
            detector = ColorMarkerDetector(
                allowed_colors={"red", "green", "blue"}, **options
            )

        arm.write(**COLOR_ARM_ANGLES)
        camera.start_task(f"COLOR: searching starts in {START_DELAY_SECONDS:g} s")
        print(f"COLOR: waiting {START_DELAY_SECONDS:g} s. Open Task camera.")
        time.sleep(START_DELAY_SECONDS)
        sequence = camera.sequence
        camera.start_task("COLOR: searching")

        while True:
            captured = camera.get_frame(after_sequence=sequence, timeout=0.5)
            if captured is None:
                continue
            sequence, frame = captured
            _, annotated, color = detector.process_frame(frame)
            if color is None:
                camera.publish_task_frame(annotated, "COLOR: searching")
                continue

            camera.save_task_frame(annotated, f"COLOR: {color.upper()} detected")
            getattr(indicator, color)()
            print(f"COLOR: {color.upper()}. LED on for {LED_SECONDS:g} s.")
            time.sleep(LED_SECONDS)
            indicator.off()
            print("COLOR: result is held in Task camera. Press Ctrl+C to exit.")
            while True:
                time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCOLOR test stopped.")

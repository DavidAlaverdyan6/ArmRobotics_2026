#!/usr/bin/env python3
"""Bench test for repeated COLOR recognition without driving the robot."""

import time

from telemetrix import telemetrix

import led
import motors
import robot_pin_config as pins
from color_marker_detector import ColorMarkerDetector, ContinuousCamera
from roboarm import get_shared_arm
from stm32_reset import STM32_RESET_GPIO, reset_stm32


SERIAL_PORT = "/dev/serial0"
PREVIEW_PORT = 8000
LED_SECONDS = 1.0
COLOR_CONFIRM_SECONDS = 1.0
MIN_OBJECT_AREA = 700

# This test intentionally differs from the route COLOR pose: the gripper must
# remain fully closed while the other four joints use the task angles.
COLOR_TEST_ARM_ANGLES = {
    "base": 90.0,
    "shoulder": 160.0,
    "elbow": 180.0,
    "wrist": 0.0,
    "gripper": 180.0,
}


def create_indicator():
    """Use the measured red/blue wire swap on this robot."""
    rgb = led.RGBLED(pins.LED_BLUE, pins.LED_GREEN, pins.LED_RED)
    rgb.off()
    return rgb


def status_line(text):
    print(f"\r\033[2KCOLOR TEST: {text}", end="", flush=True)


def main():
    board = None
    camera = None
    rgb = None

    try:
        print(f"Resetting STM32 through Raspberry Pi BCM{STM32_RESET_GPIO}...")
        reset_stm32()
        board = telemetrix.Telemetrix(com_port=SERIAL_PORT)

        # Explicitly claim all drive outputs at zero. This standalone test
        # never arms or commands the chassis motors.
        motors.initialize(board)
        motors.motor_stop()

        arm, _ = get_shared_arm(board)
        arm.write(**COLOR_TEST_ARM_ANGLES)
        print(
            "Arm ready: base=90 shoulder=160 elbow=180 wrist=0 "
            "gripper=180 (closed)."
        )

        rgb = create_indicator()
        detector = ColorMarkerDetector(
            led=None,
            min_object_area=MIN_OBJECT_AREA,
            stable_frames=1,
            allowed_colors={"red", "green", "blue"},
        )

        camera = ContinuousCamera(port=PREVIEW_PORT)
        camera.start()
        print("Repeated color test ready. Press Ctrl+C to stop.")

        last_sequence = camera.sequence
        frozen_until = None
        active_color = None
        candidate_color = None
        candidate_since = None
        last_status = 0.0

        while True:
            captured = camera.get_frame(
                after_sequence=last_sequence,
                timeout=None,
            )
            if captured is None:
                continue
            last_sequence, frame = captured
            objects, annotated, _ = detector.process_frame(frame)
            now = time.monotonic()

            if frozen_until is not None and now >= frozen_until:
                rgb.off()
                frozen_until = None
                active_color = None
                candidate_color = None
                candidate_since = None
                detector.reset()

            seen_color = objects[0]["color"] if objects else None
            if frozen_until is None:
                camera.save_task_frame(
                    annotated,
                    "COLOR TEST: live center detection",
                )
                if seen_color is None:
                    candidate_color = None
                    candidate_since = None
                elif seen_color != candidate_color:
                    candidate_color = seen_color
                    candidate_since = now
                elif now - candidate_since >= COLOR_CONFIRM_SECONDS:
                    active_color = candidate_color
                    getattr(rgb, active_color)()
                    frozen_until = now + LED_SECONDS
                    candidate_color = None
                    candidate_since = None
                    detector.reset()
                    camera.save_task_frame(
                        annotated,
                        f"COLOR TEST detected {active_color.upper()}",
                    )

            if now - last_status >= 0.2:
                if frozen_until is not None:
                    remaining = max(0.0, frozen_until - now)
                    text = (
                        f"{active_color.upper()} CONFIRMED; "
                        f"LED/FRAME HOLD {remaining:.1f}s"
                    )
                elif candidate_color is not None:
                    seen_for = max(0.0, now - candidate_since)
                    text = (
                        f"SEES {candidate_color.upper()} "
                        f"{seen_for:.1f}/{COLOR_CONFIRM_SECONDS:.1f}s"
                    )
                else:
                    text = "READY"
                status_line(text)
                last_status = now

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as exc:
        print(f"\nCOLOR TEST ERROR: {exc}")
    finally:
        if camera is not None:
            camera.stop()
        if rgb is not None:
            try:
                rgb.off()
                rgb.close()
            except Exception:
                pass
        try:
            motors.motor_stop()
        except Exception:
            pass
        if board is not None:
            try:
                board.shutdown()
            except (Exception, KeyboardInterrupt):
                pass
        print("\nColor test stopped; LED and motors are off.")


if __name__ == "__main__":
    main()

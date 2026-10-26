
#!/usr/bin/env python3

import time
import cv2
import numpy as np

from picamera2 import Picamera2
from telemetrix import telemetrix

import robot_pin_config as pins


# ==================================================
# CONFIGURATION
# ==================================================

SERIAL_PORT = "/dev/serial0"

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
ROTATE_180 = True

# Servo pins from your robot_pin_config.py
SERVO_X_PIN = pins.PB13
SERVO_Y_PIN = pins.PB14
SERVO_TILT_PIN = pins.PB15
SERVO_GRIPPER_PIN = pins.PB12

# Initial servo positions -- adjust for your robot
X_HOME = 90
Y_HOME = 90
TILT_HOME = 90
GRIPPER_OPEN = 30
GRIPPER_CLOSED = 100

# Limits
SERVO_MIN = 0
SERVO_MAX = 180

# Green HSV range
GREEN_LOWER = np.array([35, 80, 60], dtype=np.uint8)
GREEN_UPPER = np.array([85, 255, 255], dtype=np.uint8)

MIN_OBJECT_AREA = 700

# Image center dead zone
CENTER_TOLERANCE_X = 35
CENTER_TOLERANCE_Y = 35

# Servo movement step per adjustment
SERVO_STEP = 2

# Servo control direction: change to -1 if reversed
X_DIRECTION = 1
Y_DIRECTION = 1

# Number of consecutive centered frames required
CENTERED_FRAMES_REQUIRED = 15

# Fixed movement positions -- tune for your arm
Y_APPROACH = 115
TILT_APPROACH = 65
Y_LIFT = 75
TILT_LIFT = 90

# Time for servos to move
SERVO_SETTLE_TIME = 0.5
GRIPPER_SETTLE_TIME = 1.0

# ==================================================


def clamp_angle(angle):
    return max(SERVO_MIN, min(SERVO_MAX, int(angle)))


def move_servo(board, pin, angle):
    board.servo_write(pin, clamp_angle(angle))


def initialize_servos(board):
    servo_pins = [
        SERVO_X_PIN,
        SERVO_Y_PIN,
        SERVO_TILT_PIN,
        SERVO_GRIPPER_PIN,
    ]

    for pin in servo_pins:
        board.set_pin_mode_servo(pin)

    time.sleep(0.5)

    move_servo(board, SERVO_X_PIN, X_HOME)
    move_servo(board, SERVO_Y_PIN, Y_HOME)
    move_servo(board, SERVO_TILT_PIN, TILT_HOME)
    move_servo(board, SERVO_GRIPPER_PIN, GRIPPER_OPEN)

    time.sleep(1.0)


def detect_green(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(
        hsv,
        GREEN_LOWER,
        GREEN_UPPER
    )

    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None, mask

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < MIN_OBJECT_AREA:
        return None, mask

    moments = cv2.moments(largest)

    if moments["m00"] == 0:
        return None, mask

    center_x = int(moments["m10"] / moments["m00"])
    center_y = int(moments["m01"] / moments["m00"])

    return {
        "x": center_x,
        "y": center_y,
        "area": area,
        "contour": largest,
    }, mask


def center_arm_on_object(board, obj, servo_angles):
    """
    Moves X/Y servos to bring the green object toward
    the center of the camera image.
    """

    image_center_x = CAMERA_WIDTH // 2
    image_center_y = CAMERA_HEIGHT // 2

    error_x = obj["x"] - image_center_x
    error_y = obj["y"] - image_center_y

    moved = False

    if abs(error_x) > CENTER_TOLERANCE_X:
        servo_angles["x"] += (
            X_DIRECTION * SERVO_STEP
            if error_x > 0
            else -X_DIRECTION * SERVO_STEP
        )

        servo_angles["x"] = clamp_angle(
            servo_angles["x"]
        )

        move_servo(
            board,
            SERVO_X_PIN,
            servo_angles["x"]
        )

        moved = True

    if abs(error_y) > CENTER_TOLERANCE_Y:
        servo_angles["y"] += (
            Y_DIRECTION * SERVO_STEP
            if error_y > 0
            else -Y_DIRECTION * SERVO_STEP
        )

        servo_angles["y"] = clamp_angle(
            servo_angles["y"]
        )

        move_servo(
            board,
            SERVO_Y_PIN,
            servo_angles["y"]
        )

        moved = True

    return not moved


def grab_object(board):
    """
    Executes a predefined grab sequence.
    The positions must be tuned for the actual arm.
    """

    print("\nԿանաչ օբյեկտը կենտրոնացված է։")
    print("Ձեռքը մոտեցնում եմ օբյեկտին...")

    move_servo(board, SERVO_Y_PIN, Y_APPROACH)
    move_servo(board, SERVO_TILT_PIN, TILT_APPROACH)

    time.sleep(SERVO_SETTLE_TIME)

    print("Փակում եմ բռնիչը...")

    move_servo(
        board,
        SERVO_GRIPPER_PIN,
        GRIPPER_CLOSED
    )

    time.sleep(GRIPPER_SETTLE_TIME)

    print("Բարձրացնում եմ իրը...")

    move_servo(board, SERVO_Y_PIN, Y_LIFT)
    move_servo(board, SERVO_TILT_PIN, TILT_LIFT)

    time.sleep(SERVO_SETTLE_TIME)

    print("Բռնելու գործողությունն ավարտվեց։")


def main():
    board = None
    camera = None

    servo_angles = {
        "x": X_HOME,
        "y": Y_HOME,
    }

    centered_frames = 0
    object_grabbed = False

    try:
        print("Միանում եմ STM32-ին...")

        board = telemetrix.Telemetrix(
            com_port=SERIAL_PORT
        )

        initialize_servos(board)

        print("Միացնում եմ Raspberry Pi Camera-ն...")

        camera = Picamera2()

        camera_config = camera.create_video_configuration(
            main={
                "size": (CAMERA_WIDTH, CAMERA_HEIGHT),
                "format": "RGB888",
            }
        )

        camera.configure(camera_config)
        camera.start()

        time.sleep(1.0)

        print("\nՀամակարգը պատրաստ է։")
        print("Փնտրում եմ կանաչ օբյեկտը։")
        print("Սեղմիր Q՝ ծրագիրը փակելու համար։")

        while True:
            frame = camera.capture_array()

            # Picamera2 RGB frame -> OpenCV BGR
            frame = cv2.cvtColor(
                frame,
                cv2.COLOR_RGB2BGR
            )

            if ROTATE_180:
                frame = cv2.rotate(
                    frame,
                    cv2.ROTATE_180
                )

            obj, mask = detect_green(frame)

            image_center_x = CAMERA_WIDTH // 2
            image_center_y = CAMERA_HEIGHT // 2

            cv2.drawMarker(
                frame,
                (image_center_x, image_center_y),
                (255, 0, 0),
                cv2.MARKER_CROSS,
                20,
                2
            )

            if obj is None:
                centered_frames = 0

                if not object_grabbed:
                    cv2.putText(
                        frame,
                        "Searching for green object",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2
                    )

            else:
                x = obj["x"]
                y = obj["y"]

                cv2.drawContours(
                    frame,
                    [obj["contour"]],
                    -1,
                    (0, 255, 0),
                    2
                )

                cv2.circle(
                    frame,
                    (x, y),
                    6,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    frame,
                    f"Green: ({x}, {y})",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                if not object_grabbed:
                    centered = center_arm_on_object(
                        board,
                        obj,
                        servo_angles
                    )

                    if centered:
                        centered_frames += 1
                    else:
                        centered_frames = 0

                    if (
                        centered_frames
                        >= CENTERED_FRAMES_REQUIRED
                    ):
                        object_grabbed = True

                        grab_object(board)

                        print(
                            "Իրը բռնելուց հետո սպասում եմ։ "
                            "Վերագործարկիր ծրագիրը՝ նոր փորձի համար։"
                        )

                else:
                    cv2.putText(
                        frame,
                        "Grab sequence completed",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )

            cv2.imshow("Green Object Robot Arm", frame)
            cv2.imshow("Green Detection Mask", mask)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\nԾրագիրը կանգնեցվում է։")

    except Exception as error:
        print(f"\nՍխալ՝ {error}")

    finally:
        if board is not None:
            try:
                board.shutdown()
            except Exception:
                pass

        if camera is not None:
            try:
                camera.stop()
            except Exception:
                pass

        cv2.destroyAllWindows()
        print("Ծրագիրն ավարտվեց։")


if __name__ == "__main__":
    main()

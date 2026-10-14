import time
import numpy as np

from picamera2 import Picamera2
from telemetrix import telemetrix

from pins import PB9, PB8, PB7, PB6


# ============================================================
# CONFIGURATION
# ============================================================

WIDTH = 416
HEIGHT = 416

# Black/white threshold.
# Pixels darker than this are considered part of the line.
THRESHOLD = 128

# Only inspect this vertical section of the image.
# This keeps the robot reacting to the line immediately
# in front of it rather than objects farther away.
ROI_TOP = 300
ROI_BOTTOM = 400

# Minimum number of black pixels required to consider
# a line detected.
MIN_LINE_PIXELS = 20

# Distance from image center where the robot drives straight.
DEAD_ZONE = 25

# Delay between control updates.
LOOP_DELAY = 0.02


# ============================================================
# L298N PIN ASSIGNMENT
# ============================================================

# pins.py translates these into the STM32duino numbers:
#
# PB9 = 25
# PB8 = 24
# PB7 = 23
# PB6 = 22

LEFT_IN1 = PB9
LEFT_IN2 = PB8

RIGHT_IN1 = PB7
RIGHT_IN2 = PB6


# ============================================================
# CAMERA INITIALIZATION
# ============================================================

print("Starting camera...")

picam2 = Picamera2()

picam2.configure(
    picam2.create_preview_configuration(
        main={
            "size": (WIDTH, HEIGHT)
        }
    )
)

picam2.start()

time.sleep(1)

print("Camera ready")


# ============================================================
# STM32 INITIALIZATION
# ============================================================

print("Connecting to STM32...")

board = telemetrix.Telemetrix(
    com_port="/dev/serial0"
)

print("STM32 connected!")


# Configure L298N direction pins

board.set_pin_mode_digital_output(LEFT_IN1)
board.set_pin_mode_digital_output(LEFT_IN2)

board.set_pin_mode_digital_output(RIGHT_IN1)
board.set_pin_mode_digital_output(RIGHT_IN2)

print("L298N initialized!")


# ============================================================
# MOTOR CONTROL
# ============================================================

def left_forward():
    board.digital_write(LEFT_IN1, 1)
    board.digital_write(LEFT_IN2, 0)


def left_backward():
    board.digital_write(LEFT_IN1, 0)
    board.digital_write(LEFT_IN2, 1)


def left_stop():
    board.digital_write(LEFT_IN1, 0)
    board.digital_write(LEFT_IN2, 0)


def right_forward():
    board.digital_write(RIGHT_IN1, 1)
    board.digital_write(RIGHT_IN2, 0)


def right_backward():
    board.digital_write(RIGHT_IN1, 0)
    board.digital_write(RIGHT_IN2, 1)


def right_stop():
    board.digital_write(RIGHT_IN1, 0)
    board.digital_write(RIGHT_IN2, 0)


def forward():
    left_forward()
    right_forward()


def stop():
    left_stop()
    right_stop()


def turn_left():
    # Stop left wheel, move right wheel
    left_stop()
    right_forward()


def turn_right():
    # Stop right wheel, move left wheel
    left_forward()
    right_stop()


# ============================================================
# LINE DETECTION
# ============================================================

def find_line(frame):
    """
    Find the horizontal center of the black line.

    Returns:
        line_x
            X coordinate of line center.

        None
            If no line is detected.
    """

    # Picamera2 may provide XBGR8888.
    # We only need grayscale brightness here.
    image = frame[:, :, :3]

    # Convert to grayscale without OpenCV.
    gray = np.mean(image, axis=2)

    # Threshold:
    # True = black/dark pixel
    black = gray < THRESHOLD

    # Select bottom ROI
    roi = black[ROI_TOP:ROI_BOTTOM, :]

    # Get coordinates of black pixels
    y, x = np.where(roi)

    if len(x) < MIN_LINE_PIXELS:
        return None

    # Average X position of detected line
    line_x = np.mean(x)

    return line_x


# ============================================================
# MAIN LOOP
# ============================================================

print()
print("====================================")
print("VISION LINE FOLLOWER")
print("====================================")
print(f"Resolution : {WIDTH}x{HEIGHT}")
print(f"Threshold  : {THRESHOLD}")
print(f"ROI        : {ROI_TOP}:{ROI_BOTTOM}")
print(f"Dead zone  : {DEAD_ZONE}")
print()
print("Starting robot...")
print("Press CTRL+C to stop.")
print()


try:

    while True:

        # ----------------------------------------------------
        # Capture frame
        # ----------------------------------------------------

        frame = picam2.capture_array()


        # ----------------------------------------------------
        # Detect line
        # ----------------------------------------------------

        line_x = find_line(frame)


        # ----------------------------------------------------
        # Line lost
        # ----------------------------------------------------

        if line_x is None:

            print("LINE LOST")

            stop()

            time.sleep(LOOP_DELAY)

            continue


        # ----------------------------------------------------
        # Calculate steering error
        # ----------------------------------------------------

        image_center = WIDTH / 2

        error = line_x - image_center


        # ----------------------------------------------------
        # Steering
        # ----------------------------------------------------

        if abs(error) <= DEAD_ZONE:

            # Line is centered
            forward()

            state = "FORWARD"


        elif error < 0:

            # Line is to the left
            turn_left()

            state = "LEFT"


        else:

            # Line is to the right
            turn_right()

            state = "RIGHT"


        # ----------------------------------------------------
        # Debug output
        # ----------------------------------------------------

        print(
            f"line={line_x:6.1f} "
            f"error={error:7.1f} "
            f"{state}"
        )


        time.sleep(LOOP_DELAY)


# ============================================================
# SAFE SHUTDOWN
# ============================================================

except KeyboardInterrupt:

    print("\nStopping robot...")


finally:

    stop()

    picam2.stop()

    board.shutdown()

    print("Robot stopped safely.")


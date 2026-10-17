#!/usr/bin/env python3
"""Non-blocking 8-channel line tracker using a shared Telemetrix board."""

import json
import os
import time

import robot_pin_config as robot_pins


# Preserve the sensor order already used by the robot.
SENSOR_PINS = list(reversed(robot_pins.LINE_SENSORS))

CALIBRATION_FILE = os.path.expanduser(
    "~/ArmRobotics/linetracker/calibration.json"
)

SENSOR_COUNT = 8

# Normalized readings represent BLACKNESS:
#   0.0 = white
#   1.0 = black
#
# Position is negative on the left, positive on the right.
WEIGHTS = [-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]

LINE_DETECTED_THRESHOLD = 0.15

# A side group is considered black when it sees <= 10% white.
CROSSING_WHITE_THRESHOLD = 0.10
CROSSING_CENTER_BLACKNESS_MIN = 0.20

LEFT_SENSORS = (0, 1)
CENTER_SENSORS = (3, 4)
RIGHT_SENSORS = (6, 7)

CALIBRATION_SAMPLES = 100

sensor_values = [0] * SENSOR_COUNT
board = None


def _make_callback(index):
    def callback(data):
        try:
            sensor_values[index] = int(data[2])
        except (IndexError, TypeError, ValueError):
            pass

    return callback


def initialize(board_instance):
    """Configure sensors on an already-connected Telemetrix board."""
    global board

    if board_instance is None:
        raise ValueError("A connected Telemetrix board is required.")

    board = board_instance

    for index, pin in enumerate(SENSOR_PINS):
        board.set_pin_mode_analog_input(
            pin,
            differential=0,
            callback=_make_callback(index),
        )


def load_calibration():
    try:
        with open(CALIBRATION_FILE, "r", encoding="utf-8") as file:
            data = json.load(file)

        if (
            len(data.get("white", [])) != SENSOR_COUNT
            or len(data.get("black", [])) != SENSOR_COUNT
        ):
            return None

        return data

    except (OSError, ValueError, TypeError):
        return None


def save_calibration(calibration):
    directory = os.path.dirname(CALIBRATION_FILE)
    os.makedirs(directory, exist_ok=True)

    temporary_path = CALIBRATION_FILE + ".tmp"

    with open(temporary_path, "w", encoding="utf-8") as file:
        json.dump(calibration, file, indent=2)

    os.replace(temporary_path, CALIBRATION_FILE)


def _read_average_samples(sample_count=CALIBRATION_SAMPLES):
    """Blocking sampling is used only during explicit calibration."""
    samples = [[] for _ in range(SENSOR_COUNT)]
    time.sleep(0.2)

    for _ in range(sample_count):
        for index, value in enumerate(sensor_values):
            samples[index].append(value)

        time.sleep(0.005)

    return [
        sum(channel) / len(channel)
        for channel in samples
    ]


def calibrate():
    if board is None:
        raise RuntimeError("Call initialize(board) before calibration.")

    input("Place all sensors over WHITE, then press ENTER...")
    white = _read_average_samples()

    input("Place all sensors over BLACK, then press ENTER...")
    black = _read_average_samples()

    calibration = {
        "white": white,
        "black": black,
    }

    save_calibration(calibration)
    return calibration


def normalize_sensor(index, raw_value, calibration):
    """Convert raw ADC values to normalized blackness in [0, 1]."""
    white = float(calibration["white"][index])
    black = float(calibration["black"][index])

    if abs(white - black) < 1:
        return 0.0

    if black < white:
        value = (white - raw_value) / (white - black)
    else:
        value = (raw_value - white) / (black - white)

    return max(0.0, min(1.0, value))


def get_normalized_values(calibration):
    return [
        normalize_sensor(i, sensor_values[i], calibration)
        for i in range(SENSOR_COUNT)
    ]


def calculate_position(values):
    """Return a signed line position: left < 0, center = 0, right > 0."""
    total = sum(values)

    if total < LINE_DETECTED_THRESHOLD:
        return None

    return sum(
        value * weight
        for value, weight in zip(values, WEIGHTS)
    ) / total


def _group_mean(values, indexes):
    return sum(values[i] for i in indexes) / len(indexes)


def _group_white_fraction(values, indexes):
    """Normalized values are blackness, so white fraction is 1 - value."""
    return sum(1.0 - values[i] for i in indexes) / len(indexes)


def detect_crossing(values):
    """
    Detect a branch from the outer sensor groups.

    Requested mapping:
      - right group <= 10% white -> LEFT
      - left group <= 10% white  -> RIGHT
      - both groups <= 10% white -> BOTH

    This intentionally follows the robot's specified branch convention.
    The center must also see enough black to reject isolated side readings.
    """
    if len(values) != SENSOR_COUNT:
        return None

    center_blackness = _group_mean(values, CENTER_SENSORS)

    if center_blackness < CROSSING_CENTER_BLACKNESS_MIN:
        return None

    left_white = _group_white_fraction(values, LEFT_SENSORS)
    right_white = _group_white_fraction(values, RIGHT_SENSORS)

    left_is_black = left_white <= CROSSING_WHITE_THRESHOLD
    right_is_black = right_white <= CROSSING_WHITE_THRESHOLD

    if left_is_black and right_is_black:
        return "BOTH"

    if right_is_black:
        return "RIGHT"

    if left_is_black:
        return "LEFT"

    return None


def update(calibration):
    """
    Return the latest readings without sleeping or printing.

    Telemetrix updates sensor_values asynchronously through callbacks.
    """
    if board is None:
        raise RuntimeError("Call initialize(board) before update().")

    raw = sensor_values.copy()

    normalized = [
        normalize_sensor(i, raw[i], calibration)
        for i in range(SENSOR_COUNT)
    ]

    position = calculate_position(normalized)

    return {
        "raw": raw,
        "normalized": normalized,
        "position": position,
        "line_detected": position is not None,
        "crossing": detect_crossing(normalized),
    }


def shutdown():
    """Release this module's board reference; the caller closes the board."""
    global board
    board = None
#!/usr/bin/env python3
"""Non-blocking line tracker using a shared Telemetrix board."""

import json
import os
import time

import robot_pin_config as robot_pins

# The physical sensor array is wired backwards.
# Reverse the centralized configuration to preserve the validated ordering.
SENSOR_PINS = list(reversed(robot_pins.LINE_SENSORS))

CALIBRATION_FILE = os.path.expanduser(
    "~/ArmRobotics/linetracker/calibration.json"
)

SENSOR_NAMES = [f"S{i}" for i in range(1, 9)]
WEIGHTS = [-3, -2, -1, 0, 1, 2, 3, 4]
SENSOR_COUNT = 8

LINE_DETECTED_THRESHOLD = 0.15
CROSSING_SIDE_THRESHOLD = 0.45
CROSSING_CENTER_THRESHOLD = 0.20

LEFT_SENSORS = [0, 1]
RIGHT_SENSORS = [6, 7]
CENTER_SENSORS = [3, 4]

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
    """Configure analog inputs on an already-connected Telemetrix board."""
    global board
    if board_instance is None:
        raise ValueError("A connected Telemetrix board is required.")

    board = board_instance
    for index, pin in enumerate(SENSOR_PINS):
        board.set_pin_mode_analog_input(
            pin, differential=0, callback=_make_callback(index)
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
    if directory:
        os.makedirs(directory, exist_ok=True)

    temporary_path = CALIBRATION_FILE + ".tmp"
    with open(temporary_path, "w", encoding="utf-8") as file:
        json.dump(calibration, file, indent=2)
    os.replace(temporary_path, CALIBRATION_FILE)


def _read_average_samples(sample_count=CALIBRATION_SAMPLES):
    """Blocking sampling used only during explicitly requested calibration."""
    samples = [[] for _ in range(SENSOR_COUNT)]
    time.sleep(0.2)

    for _ in range(sample_count):
        for index, value in enumerate(sensor_values):
            samples[index].append(value)
        time.sleep(0.005)

    return [sum(channel) / len(channel) for channel in samples]


def calibrate():
    """Interactively calibrate white and black surfaces at startup."""
    if board is None:
        raise RuntimeError("Call initialize(board) before calibration.")

    input("Place all sensors over WHITE, then press ENTER...")
    white = _read_average_samples()
    input("Place all sensors over BLACK, then press ENTER...")
    black = _read_average_samples()

    calibration = {"white": white, "black": black}
    save_calibration(calibration)
    return calibration


def normalize_sensor(index, raw_value, calibration):
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
    total = sum(values)
    if total < LINE_DETECTED_THRESHOLD:
        return None

    return sum(value * weight for value, weight in zip(values, WEIGHTS)) / total


def _group_strength(values, indexes):
    return sum(values[i] for i in indexes) / len(indexes)


def detect_crossing(values):
    center = _group_strength(values, CENTER_SENSORS)
    left = (
        _group_strength(values, LEFT_SENSORS) >= CROSSING_SIDE_THRESHOLD
        and center >= CROSSING_CENTER_THRESHOLD
    )
    right = (
        _group_strength(values, RIGHT_SENSORS) >= CROSSING_SIDE_THRESHOLD
        and center >= CROSSING_CENTER_THRESHOLD
    )

    if left and right:
        return "BOTH"
    if left:
        return "LEFT"
    if right:
        return "RIGHT"
    return None


def update(calibration):
    """
    Return the latest tracker state without sleeping or printing.

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
    """Release module references; the caller owns and closes the board."""
    global board
    board = None

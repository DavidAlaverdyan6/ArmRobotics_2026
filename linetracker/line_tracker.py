#!/usr/bin/env python3

import json
import os
import time

from telemetrix import telemetrix

import pins


# ============================================================
# CONFIGURATION
# ============================================================

CALIBRATION_FILE = os.path.expanduser(
    "~/ArmRobotics/linetracker/calibration.json"
)

# ============================================================
# 8-CHANNEL ANALOG LINE SENSOR
#
# S1   S2   S3   S4   S5   S6   S7   S8
# PA0  PA1  PA2  PA3  PA4  PA5  PA6  PA7
#
# WEIGHTS:
#
# -3   -2   -1    0   +1   +2   +3   +4
# ============================================================

SENSOR_PINS = [
    pins.PA0,
    pins.PA1,
    pins.PA2,
    pins.PA3,
    pins.PA4,
    pins.PA5,
    pins.PA6,
    pins.PA7,
]

SENSOR_NAMES = [
    "S1",
    "S2",
    "S3",
    "S4",
    "S5",
    "S6",
    "S7",
    "S8",
]

WEIGHTS = [
    -3,
    -2,
    -1,
     0,
     1,
     2,
     3,
     4,
]

SENSOR_COUNT = 8


# ============================================================
# LINE DETECTION
# ============================================================

LINE_DETECTED_THRESHOLD = 0.15


# ============================================================
# CROSSING DETECTION
# ============================================================

# Strength required for the corresponding side sensors
# to be considered seeing the crossing.
CROSSING_SIDE_THRESHOLD = 0.45

# Minimum center strength required together with a side.
CROSSING_CENTER_THRESHOLD = 0.20

# Number of sensors on each side used for crossing detection.
LEFT_SENSORS = [0, 1]
RIGHT_SENSORS = [6, 7]

# Center sensors.
CENTER_SENSORS = [3, 4]


# ============================================================
# CALIBRATION
# ============================================================

CALIBRATION_SAMPLES = 100


# ============================================================
# UPDATE RATE
# ============================================================

UPDATE_INTERVAL = 0.02


# ============================================================
# GLOBAL STATE
# ============================================================

sensor_values = [0] * SENSOR_COUNT

board = None


# ============================================================
# TELEMETRIX CALLBACKS
# ============================================================

def make_callback(index):
    def callback(data):
        try:
            sensor_values[index] = int(data[2])
        except (IndexError, TypeError, ValueError):
            pass

    return callback


# ============================================================
# CALIBRATION FILE
# ============================================================

def save_calibration(calibration):
    directory = os.path.dirname(CALIBRATION_FILE)

    if directory:
        os.makedirs(directory, exist_ok=True)

    temp_file = CALIBRATION_FILE + ".tmp"

    with open(temp_file, "w") as f:
        json.dump(calibration, f, indent=4)

    os.replace(temp_file, CALIBRATION_FILE)

    print()
    print("Calibration saved:")
    print(CALIBRATION_FILE)


def load_calibration():
    if not os.path.exists(CALIBRATION_FILE):
        return None

    try:
        with open(CALIBRATION_FILE, "r") as f:
            calibration = json.load(f)

        if "white" not in calibration or "black" not in calibration:
            return None

        if len(calibration["white"]) != SENSOR_COUNT:
            return None

        if len(calibration["black"]) != SENSOR_COUNT:
            return None

        return calibration

    except Exception as e:
        print(f"Failed to load calibration: {e}")
        return None


# ============================================================
# SENSOR SAMPLING
# ============================================================

def read_average_samples(sample_count):
    samples = [[] for _ in range(SENSOR_COUNT)]

    time.sleep(0.2)

    for _ in range(sample_count):
        for i in range(SENSOR_COUNT):
            samples[i].append(sensor_values[i])

        time.sleep(0.005)

    return [
        sum(values) / len(values)
        for values in samples
    ]


def print_sensor_values(values):
    for name, value in zip(SENSOR_NAMES, values):
        print(f"{name}: {value:.1f}", end="    ")

    print()


# ============================================================
# CALIBRATION
# ============================================================

def calibrate():
    print()
    print("=" * 60)
    print("                 LINE SENSOR CALIBRATION")
    print("=" * 60)

    print()
    print("WHITE CALIBRATION")
    print("-----------------")
    print("Place all 8 sensors over the WHITE surface.")

    input("Press ENTER when ready...")

    print()
    print("Reading white values...")

    white = read_average_samples(CALIBRATION_SAMPLES)

    print()
    print("White values:")
    print_sensor_values(white)

    print()
    print("BLACK CALIBRATION")
    print("-----------------")
    print("Place all 8 sensors over the BLACK line.")

    input("Press ENTER when ready...")

    print()
    print("Reading black values...")

    black = read_average_samples(CALIBRATION_SAMPLES)

    print()
    print("Black values:")
    print_sensor_values(black)

    calibration = {
        "white": white,
        "black": black,
    }

    save_calibration(calibration)

    print()
    print("=" * 60)
    print("Calibration complete.")
    print("=" * 60)


# ============================================================
# NORMALIZATION
# ============================================================

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


# ============================================================
# WEIGHTED POSITION
# ============================================================

def calculate_position(values):
    total = sum(values)

    if total < LINE_DETECTED_THRESHOLD:
        return None

    weighted_sum = 0.0

    for value, weight in zip(values, WEIGHTS):
        weighted_sum += value * weight

    return weighted_sum / total


# ============================================================
# CROSSING HELPERS
# ============================================================

def get_group_strength(values, indexes):
    if not indexes:
        return 0.0

    return sum(values[i] for i in indexes) / len(indexes)


# ============================================================
# CROSSING DETECTION
# ============================================================

def detect_crossing(values):
    left_strength = get_group_strength(values, LEFT_SENSORS)
    right_strength = get_group_strength(values, RIGHT_SENSORS)
    center_strength = get_group_strength(values, CENTER_SENSORS)

    left = (
        left_strength >= CROSSING_SIDE_THRESHOLD
        and center_strength >= CROSSING_CENTER_THRESHOLD
    )

    right = (
        right_strength >= CROSSING_SIDE_THRESHOLD
        and center_strength >= CROSSING_CENTER_THRESHOLD
    )

    if left and right:
        return "BOTH"

    if left:
        return "LEFT"

    if right:
        return "RIGHT"

    return None


# ============================================================
# DISPLAY
# ============================================================

def print_status(normalized, position, crossing):
    values = " ".join(
        f"{value:.2f}"
        for value in normalized
    )

    if position is None:
        position_text = "LOST"
    else:
        position_text = f"{position:+.2f}"

    if crossing is None:
        crossing_text = "NONE"
    else:
        crossing_text = crossing

    print(
        f"\r{values}    "
        f"Position: {position_text:<8}    "
        f"Crossing: {crossing_text:<5}",
        end="",
        flush=True
    )


# ============================================================
# TELEMETRIX INITIALIZATION
# ============================================================

def initialize_board():
    global board

    print()
    print("Connecting to STM32...")
    print("Telemetrix: auto-detect serial port")

    board = telemetrix.Telemetrix()

    time.sleep(1)

    print("STM32 connected.")

    for i, pin in enumerate(SENSOR_PINS):
        board.set_pin_mode_analog_input(
            pin,
            differential=0,
            callback=make_callback(i)
        )

        time.sleep(0.05)

    print("8 analog sensors initialized.")


# ============================================================
# LINE TRACKER
# ============================================================

def run_tracker(calibration):
    print()
    print("=" * 60)
    print("                    LINE TRACKER")
    print("=" * 60)

    print()
    print("Sensors:")
    print("S1    S2    S3    S4    S5    S6    S7    S8")

    print("Weights:")
    print("-3    -2    -1     0    +1    +2    +3    +4")

    print()
    print("Crossing detection:")
    print("LEFT  = S1/S2 + center")
    print("RIGHT = S7/S8 + center")
    print("BOTH  = left + right")

    print()
    print("Running...")
    print("Press CTRL+C to stop.")
    print()

    try:
        while True:
            normalized = get_normalized_values(calibration)
            position = calculate_position(normalized)
            crossing = detect_crossing(normalized)

            print_status(normalized, position, crossing)

            time.sleep(UPDATE_INTERVAL)

    except KeyboardInterrupt:
        print()
        print()
        print("Line tracker stopped.")


# ============================================================
# CLEANUP
# ============================================================

def shutdown():
    global board

    if board is not None:
        try:
            board.shutdown()
        except Exception:
            pass

        board = None


# ============================================================
# MAIN
# ============================================================

def main():
    try:
        initialize_board()

        calibration = load_calibration()

        if calibration is None:
            print()
            print("No valid calibration found.")
            print("Starting calibration...")

            calibrate()

            calibration = load_calibration()

        if calibration is None:
            print("ERROR: Calibration could not be loaded.")
            return

        run_tracker(calibration)

    finally:
        shutdown()


if __name__ == "__main__":
    main()
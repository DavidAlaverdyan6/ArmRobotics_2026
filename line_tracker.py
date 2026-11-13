#!/usr/bin/env python3
"""ADC acquisition, line interpretation, and differential tracking engine."""

import json
import os
import time
import threading

import robot_pin_config as robot_pins
import motors


# Preserve the sensor order already used by the robot.
SENSOR_PINS = list(reversed(robot_pins.LINE_SENSORS))

CALIBRATION_FILE = os.path.expanduser(
    "~/ArmRobotics/linetracker/calibration.local.json"
)
LEGACY_CALIBRATION_FILE = os.path.expanduser(
    "~/ArmRobotics/linetracker/calibration.json"
)

SENSOR_COUNT = 8

# Normalized readings represent BLACKNESS:
#   0.0 = white
#   1.0 = black
#
# Position is negative on the left, positive on the right.
WEIGHTS = [-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]
# The symmetric half-step weights place the array center exactly between S4
# and S5. Both sensors contribute equally there, producing position 0.0.
CENTER_POSITION = (WEIGHTS[0] + WEIGHTS[-1]) / 2.0

# Reject faint marker residue and floor shading. A usable line must either
# cover enough of the array with visible contrast, or be genuinely black on
# one sensor (which still allows a narrow real line between sensors).
LINE_NOISE_FLOOR = 0.20
LINE_DETECTED_PEAK_MIN = 0.45
LINE_DETECTED_TOTAL_MIN = 0.70
LINE_DETECTED_STRONG_PEAK = 0.80
# When black tape and a gray remnant appear in the same ADC frame, steer by
# the black tape. Keep the ordinary threshold when no dark anchor is present:
# a partially covered real line can read only about 0.5 on several sensors.
LINE_DOMINANT_PEAK_MIN = 0.80
LINE_DOMINANT_FRACTION = 0.70

# A real branch normally covers the extreme sensor and its inward neighbor.
# This is less calibration-sensitive than demanding near-perfect black (0.95)
# and rejects isolated dark spots on an uneven floor.
CROSSING_EDGE_ON = 0.85
CROSSING_SIDE_SUPPORT_MIN = 0.55
CROSSING_CENTER_BLACKNESS_MIN = 0.45

LEFT_SENSORS = (0, 1)
CENTER_SENSORS = (3, 4)
RIGHT_SENSORS = (6, 7)

CALIBRATION_SAMPLES = 100

sensor_values = [0] * SENSOR_COUNT
sample_times = [None] * SENSOR_COUNT
sample_ids = [0] * SENSOR_COUNT
callback_received = [False] * SENSOR_COUNT
_sample_lock = threading.Lock()
board = None


def _make_callback(index):
    pin = SENSOR_PINS[index]

    def callback(data):
        # Match ADC.py: set_pin_mode_analog_input binds each callback to a
        # sensor index, and data[2] is the ADC value. Do not compare data[1]
        # with the configured pin: Telemetrix's reported pin identifier may
        # differ from this project's STM32 pin constants.
        try:
            value = int(data[2])
            if not 0 <= value <= 4095:
                return
        except (IndexError, TypeError, ValueError):
            return
        with _sample_lock:
            sensor_values[index] = value
            sample_times[index] = time.monotonic()
            sample_ids[index] += 1
            callback_received[index] = True

    return callback


def initialize(board_instance):
    """Configure sensors on an already-connected Telemetrix board."""
    global board

    if board_instance is None:
        raise ValueError("A connected Telemetrix board is required.")

    board = board_instance
    with _sample_lock:
        sensor_values[:] = [0] * SENSOR_COUNT
        sample_times[:] = [None] * SENSOR_COUNT
        sample_ids[:] = [0] * SENSOR_COUNT
        callback_received[:] = [False] * SENSOR_COUNT

    for index, pin in enumerate(SENSOR_PINS):
        board.set_pin_mode_analog_input(
            pin,
            callback=_make_callback(index),
        )


def callback_status():
    """Return which ADC channels have produced at least one callback."""
    with _sample_lock:
        return callback_received.copy()


def all_callbacks_ready():
    with _sample_lock:
        return all(callback_received)


def wait_for_callbacks(timeout=6.0, poll_interval=0.04):
    """Wait until all eight ADC subscriptions have reported once."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if all_callbacks_ready():
            return True
        time.sleep(poll_interval)
    return all_callbacks_ready()


def load_calibration():
    """Load gitignored local calibration, migrating legacy values once.

    Never overwrite an existing local calibration with repository defaults.
    """
    for path in (CALIBRATION_FILE, LEGACY_CALIBRATION_FILE):
        try:
            with open(path, "r", encoding="utf-8") as file:
                data = json.load(file)
            if (
                not isinstance(data, dict)
                or len(data.get("white", [])) != SENSOR_COUNT
                or len(data.get("gray", [])) != SENSOR_COUNT
                or len(data.get("black", [])) != SENSOR_COUNT
                or data.get("pins") != SENSOR_PINS
            ):
                return None
        except FileNotFoundError:
            continue
        except (OSError, ValueError, TypeError):
            return None

        if path == LEGACY_CALIBRATION_FILE:
            # First run after upgrading: retain the old sensor readings in a
            # private file that future git pulls cannot overwrite.
            save_calibration(data)
        return data
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

    input("Place all sensors over GRAY, then press ENTER...")
    gray = _read_average_samples()

    input("Place all sensors over BLACK, then press ENTER...")
    black = _read_average_samples()

    calibration = {
        "white": white,
        "gray": gray,
        "black": black,
        "pins": SENSOR_PINS.copy(),
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
    if len(values) != SENSOR_COUNT:
        return None

    peak = max(values, default=0.0)
    filtered = [
        value if value >= LINE_NOISE_FLOOR else 0.0
        for value in values
    ]
    total = sum(filtered)

    visible_patch = (
        peak >= LINE_DETECTED_PEAK_MIN
        and total >= LINE_DETECTED_TOTAL_MIN
    )
    narrow_black_line = peak >= LINE_DETECTED_STRONG_PEAK
    if not (visible_patch or narrow_black_line):
        return None

    if peak >= LINE_DOMINANT_PEAK_MIN:
        dominant_min = max(LINE_NOISE_FLOOR, peak * LINE_DOMINANT_FRACTION)
        filtered = [
            value if value >= dominant_min else 0.0
            for value in values
        ]
        total = sum(filtered)

    return sum(
        value * weight
        for value, weight in zip(filtered, WEIGHTS)
    ) / total


def clamp(value, low, high):
    return max(low, min(high, value))


class LineTrackingEngine:
    """Convert this tracker's calibrated position into differential PWM."""

    def __init__(
        self,
        base_speed=110,
        max_speed=180,
        min_speed=-80,
        kp=30.0,
        target_position=CENTER_POSITION,
        motor_driver=motors,
        kd=2.0,
    ):
        self.base_speed = base_speed
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.kp = kp
        self.target_position = target_position
        self.motor_driver = motor_driver
        self.kd = kd
        self.reset_steering()

    def reset_steering(self):
        self.previous_error = None
        self.previous_time = None
        self.filtered_derivative = 0.0

    def stop(self):
        self.reset_steering()
        self.motor_driver.motor_stop()

    def correction(self, position):
        return self.kp * (position - self.target_position)

    def steering_correction(self, position):
        """Return the shared P+D line correction from all eight sensors."""
        correction = self.correction(position)
        error = position - self.target_position
        now = time.monotonic()
        if self.previous_time is not None:
            dt = now - self.previous_time
            if 0.005 <= dt <= 0.25:
                # The same damping is used forward and in reverse.  It makes
                # the robot ease into the centre instead of zig-zagging past
                # the line whenever the sensor position changes rapidly.
                alpha = dt / (0.06 + dt)
                rate = (error - self.previous_error) / dt
                self.filtered_derivative += alpha * (
                    rate - self.filtered_derivative
                )
            else:
                self.filtered_derivative = 0.0
        correction += clamp(self.kd * self.filtered_derivative, -20, 20)
        self.previous_error = error
        self.previous_time = now
        return correction

    def drive(self, state, speed=None, max_speed=None):
        """Follow the line forward. Return False when no line is visible."""
        if state is None or state.get("position") is None:
            self.stop()
            return False

        speed = self.base_speed if speed is None else speed
        max_speed = self.max_speed if max_speed is None else max_speed
        correction = self.steering_correction(state["position"])
        left = clamp(speed + correction, self.min_speed, max_speed)
        right = clamp(speed - correction, self.min_speed, max_speed)
        self.motor_driver.set_speeds(round(left), round(right))
        return True

    def drive_straight(self, speed=None, max_speed=None):
        """Continue forward without a usable line position."""
        self.reset_steering()
        speed = self.base_speed if speed is None else speed
        max_speed = self.max_speed if max_speed is None else max_speed
        pwm = round(clamp(speed, self.min_speed, max_speed))
        self.motor_driver.set_speeds(pwm, pwm)
        return True

    def reverse(
        self, state, speed, heading_correction=0.0, max_delta=85,
        line_gain=1.0,
    ):
        """Follow a visible line backwards by braking one wheel only."""
        if state is None or state.get("position") is None:
            self.stop()
            return False

        correction = clamp(
            self.steering_correction(state["position"]) * line_gain
            + heading_correction,
            -max_delta,
            max_delta,
        )
        # The sensor's forward steering polarity must be inverted in reverse.
        # Keep one wheel at the requested reverse speed and *brake* the other
        # one.  Do not accelerate either wheel above base: on this robot that
        # made reverse return pull to the left.  PWM 80 is the physical floor.
        base = max(abs(speed), 80)
        if correction >= 0:
            left = -base
            right = -clamp(base - correction, 80, base)
        else:
            left = -clamp(base + correction, 80, base)
            right = -base
        self.motor_driver.set_speeds(round(left), round(right))
        return True

    def reverse_straight(self, speed, heading_correction=0.0, max_delta=85):
        """Continue reversing from gyro heading when line position is absent."""
        self.reset_steering()
        correction = clamp(heading_correction, -max_delta, max_delta)
        base = max(abs(speed), 80)
        if correction >= 0:
            left = -base
            right = -clamp(base - correction, 80, base)
        else:
            left = -clamp(base + correction, 80, base)
            right = -base
        self.motor_driver.set_speeds(round(left), round(right))
        return True


def _group_mean(values, indexes):
    return sum(values[i] for i in indexes) / len(indexes)


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

    left_is_black = (
        values[LEFT_SENSORS[0]] >= CROSSING_EDGE_ON
        and values[LEFT_SENSORS[1]] >= CROSSING_SIDE_SUPPORT_MIN
    )
    right_is_black = (
        values[RIGHT_SENSORS[-1]] >= CROSSING_EDGE_ON
        and values[RIGHT_SENSORS[0]] >= CROSSING_SIDE_SUPPORT_MIN
    )

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

    with _sample_lock:
        raw, times, ids = (sensor_values.copy(), sample_times.copy(),
                           sample_ids.copy())

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
        "sample_times": times,
        "sample_ids": ids,
    }


def all_channels_fresh(max_age=1.0):
    now = time.monotonic()
    with _sample_lock:
        times = sample_times.copy()
    return (all(t is not None and 0 <= now - t <= max_age for t in times)
            and max(times) - min(times) <= 0.15)


def shutdown():
    """Release this module's board reference; the caller closes the board."""
    global board
    board = None

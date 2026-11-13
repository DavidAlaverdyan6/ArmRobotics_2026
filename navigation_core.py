"""Hardware-independent safety and junction decisions for the route controller."""

import json
import math
from collections import deque
from pathlib import Path

SENSOR_TO_AXLE_MM = 70.0
SENSOR_SPAN_MM = 60.0
WHEEL_DIAMETER_MM = 168.0
LINE_WIDTH_MM = 30.0
CROSSBAR_WIDTH_MM = 30.0
MIN_PIVOT_PWM = 70
# 226 mm was measured between inner edges; it is not the wheel-centre track.
INNER_WHEEL_GAP_MM = 226.0
WHEEL_CIRCUMFERENCE_MM = math.pi * WHEEL_DIAMETER_MM

MAX_SAMPLE_AGE_S = 1.0
MAX_FRAME_SPREAD_S = 0.15
BLACK_ON = 0.85
BLACK_OFF = 0.65
CENTER_MIN = 0.55
JUNCTION_CONFIRM_FRAMES = 3
JUNCTION_CLEAR_FRAMES = 4


def turn_stop_angle(direction, target_deg, rate_dps):
    """Provisional coast lead; measured excess was at PWM 70 only."""
    lead = 5.6425 if direction == "L" else 6.125
    return target_deg - max(lead, min(12.0, abs(rate_dps) * 0.04))


class LineMonitor:
    """Track recent valid frames and confirm loss without reacting to one glitch."""

    def __init__(self):
        self.history = deque(maxlen=12)
        self.missing = 0
        self.last_error = 0.0
        self.derivative = 0.0

    def observe(self, state, now, target, left_pwm=0, right_pwm=0,
                direction="FORWARD", route_index=0, gyro_angle=None):
        position = state["position"]
        if position is None:
            self.missing += 1
            return self.missing >= 3, False
        self.missing = 0
        error = position - target
        if self.history:
            dt = now - self.history[-1]["time"]
            if dt > 0:
                self.derivative = (error - self.last_error) / dt
        self.last_error = error
        self.history.append({
            "time": now, "position": position, "error": error,
            "derivative": self.derivative, "gyro_angle": gyro_angle,
            "left_pwm": left_pwm, "right_pwm": right_pwm,
            "direction": direction, "route_index": route_index,
        })
        values = state["normalized"]
        outer = max(values[0], values[7]) >= 0.65
        moving_outward = error * self.derivative > 0.5
        predicted = (abs(error) > 2.0 and (moving_outward or outer))
        return False, predicted


class FreshFrames:
    """Accept a complete, coherent eight-channel snapshot only once."""

    def __init__(self):
        self.last_ids = None

    def accept(self, state, now):
        ids = state.get("sample_ids")
        times = state.get("sample_times")
        if (not isinstance(ids, list) or not isinstance(times, list)
                or len(ids) != 8 or len(times) != 8
                or any(not isinstance(i, int) or i <= 0 for i in ids)
                or any(t is None or not math.isfinite(t) for t in times)):
            return False
        if (max(times) - min(times) > MAX_FRAME_SPREAD_S
                or any(t > now or now - t > MAX_SAMPLE_AGE_S for t in times)):
            return False
        # Telemetrix may suppress a callback for an unchanged channel even
        # with continuous analog reporting. A complete frame therefore needs
        # every channel fresh, but only one channel must advance since the
        # previous accepted frame; requiring all eight to advance can make
        # startup fail on a valid stationary line.
        if self.last_ids is not None and not any(
            new > old for new, old in zip(ids, self.last_ids)
        ):
            return False
        self.last_ids = ids.copy()
        return True


class JunctionTracker:
    """Hysteresis, full fresh-frame confirmation and one event per bar."""

    def __init__(self):
        self.left = self.right = False
        self.candidate = None
        self.count = self.clear_count = 0
        self.latched = False
        self.first_seen = None
        self.last_seen = None

    def observe(self, state, now, forward=True):
        values = state["normalized"]
        if len(values) != 8:
            raise ValueError("Expected eight normalized sensors")
        self.left = values[0] > (BLACK_OFF if self.left else BLACK_ON)
        self.right = values[7] > (BLACK_OFF if self.right else BLACK_ON)
        center = max(values[2:6]) >= CENTER_MIN
        branch = center and (self.left or self.right)
        if not branch:
            previous_kind = self.candidate
            self.clear_count += 1
            self.candidate = None
            self.count = 0
            if self.clear_count >= JUNCTION_CLEAR_FRAMES:
                self.latched = False
            if self.last_seen is not None and self.latched:
                resolved = ("T" if not center else "CROSS") if previous_kind == "CROSS_OR_T" else previous_kind
                self.last_seen = None
                return ("EXIT", resolved) if forward else (None, None)
            self.last_seen = None
            return None, None
        self.clear_count = 0
        self.last_seen = now
        kind = ("CROSS_OR_T" if self.left and self.right else
                "LEFT" if self.left else "RIGHT")
        if self.latched and kind == "CROSS_OR_T":
            self.candidate = kind
        if not forward or self.latched:
            return None, None
        # S1 and S8 may assert on successive frames. Retain the first edge.
        if self.candidate is None:
            self.first_seen = now
            self.count = 1
        else:
            self.count += 1
        self.candidate = kind
        if self.count >= JUNCTION_CONFIRM_FRAMES:
            self.latched = True
            return kind, self.first_seen
        return None, None

    def read(self, state):
        """Compatibility for the existing B/F reverse task helpers."""
        values = state["normalized"]
        if len(values) != 8:
            raise ValueError("Expected eight normalized sensors")
        left = values[0] >= BLACK_ON
        right = values[7] >= BLACK_ON
        center = max(values[2:6]) >= CENTER_MIN
        return left, right, center and (left or right)


class MotionCalibration:
    """Measured speed and stop/restart travel for the exact alignment maneuver."""

    def __init__(self, speeds=None, stop_mm=None, launch_mm=None):
        self.speeds = speeds or {}
        self.stop_mm = stop_mm or {}
        self.launch_mm = launch_mm or {}

    @classmethod
    def load(cls, path):
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            speeds = {int(k): float(v) for k, v in data["forward_mm_s"].items()}
            stops = {int(k): float(v) for k, v in data["stop_mm"].items()}
            launch = {int(k): float(v) for k, v in data["launch_deficit_mm"].items()}
            if any(p not in (80, 100, 120, 140) or not math.isfinite(v)
                   or v <= 0 for p, v in speeds.items()):
                raise ValueError("Invalid forward speed calibration")
            if any(p not in (80, 100, 120, 140) or not math.isfinite(v)
                   or v < 0 for p, v in stops.items()):
                raise ValueError("Invalid stopping distance calibration")
            if any(p not in (80, 100, 120, 140) or not math.isfinite(v)
                   or v < 0 for p, v in launch.items()):
                raise ValueError("Invalid launch calibration")
            return cls(speeds, stops, launch)
        except (OSError, KeyError, TypeError, ValueError):
            return cls()

    def duration_after_trailing_edge(self, pwm=100):
        """At bar exit the sensor is +15 mm; axle remains 55 mm behind centre.

        stop_mm covers travel from both stops in the stop/restart/stop sequence.
        launch_deficit_mm measures the restart acceleration loss versus steady speed.
        """
        speed = self.speeds.get(pwm)
        stop = self.stop_mm.get(pwm)
        launch = self.launch_mm.get(pwm)
        if (speed is None or stop is None or launch is None
                or stop >= SENSOR_TO_AXLE_MM - CROSSBAR_WIDTH_MM / 2):
            return None
        return (SENSOR_TO_AXLE_MM - CROSSBAR_WIDTH_MM / 2 - stop + launch) / speed

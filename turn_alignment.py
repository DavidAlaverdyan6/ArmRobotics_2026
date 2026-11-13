"""Forward alignment after crossing a junction's trailing edge.

This is a best-effort OPEN-LOOP geometry alignment. It must be verified on the
physical course. Fail closed if sensor frames or calibration are unavailable.
"""
import math
import time

import line_tracker
import motors
from navigation_core import FreshFrames
from junction_drive_config import load as load_junction_drive

EDGE_DRIVE_TIMEOUT_S = 3.0
EDGE_POLL_S = 0.02
EDGE_EXIT_CONFIRM_FRAMES = 4
EDGE_BLACK_ON = 0.75
EDGE_BLACK_OFF = 0.55
CENTER_PRESENT = 0.45
MAX_PULSE_S = 2.0


def _side_on(values):
    return ((values[0] >= EDGE_BLACK_ON and values[1] >= EDGE_BLACK_OFF)
            or (values[7] >= EDGE_BLACK_ON and values[6] >= EDGE_BLACK_OFF))


def align_to_crossbar_center(calibration):
    """From a confirmed junction, move until trailing edge, then pulse 55 mm.

    Returns False (with motor stop) on any sensor, calibration or timing error.
    The fixed-duration pulse is configured by calibrate.py from a measured run.
    Stopping drift at the trailing edge still requires physical validation.
    """
    try:
        duration, pwm = load_junction_drive()
    except ValueError as error:
        print(f"ALIGN: {error}; motors stay stopped.")
        motors.motor_stop()
        return False
    if not math.isfinite(duration) or not 0 < duration <= MAX_PULSE_S:
        motors.motor_stop()
        return False
    fresh = FreshFrames()
    seen_crossbar = False
    cleared = 0
    deadline = time.monotonic()+EDGE_DRIVE_TIMEOUT_S
    try:
        while time.monotonic() < deadline:
            state = line_tracker.update(calibration)
            now = time.monotonic()
            if not line_tracker.all_channels_fresh():
                print('ALIGN: stale ADC; stopping.')
                return False
            if not fresh.accept(state, now):
                time.sleep(EDGE_POLL_S)
                continue
            values = state.get('normalized', [])
            if len(values) != 8:
                return False
            crossbar = _side_on(values)
            if crossbar:
                seen_crossbar = True
                cleared = 0
            elif seen_crossbar and state.get('position') is not None and max(values[2:6]) >= CENTER_PRESENT:
                cleared += 1
            else:
                cleared = 0
            if cleared >= EDGE_EXIT_CONFIRM_FRAMES:
                motors.motor_stop()
                time.sleep(.1)
                break
            motors.set_speeds(pwm,pwm)
            time.sleep(EDGE_POLL_S)
        else:
            print('ALIGN: junction trailing edge not confirmed within timeout.')
            return False
        print(f'ALIGN: trailing edge confirmed; moving {duration:.3f}s at PWM {pwm}.')
        start = time.monotonic()
        motors.set_speeds(pwm,pwm)
        remaining = duration-(time.monotonic()-start)
        if remaining>0:
            time.sleep(remaining)
        return True
    finally:
        motors.motor_stop()

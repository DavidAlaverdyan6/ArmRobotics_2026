"""Validated forward pulse configuration for alignment at a junction exit.

This file is only a small data store; it does not control motors.
"""
import json
import math
import os
from pathlib import Path

CONFIG_PATH = Path(__file__).with_name("junction_drive.json")
MIN_PWM = 70
MAX_PWM = 255
MIN_SECONDS = 0.01
MAX_SECONDS = 2.0


def validate(seconds, pwm):
    if isinstance(seconds, bool) or isinstance(pwm, bool):
        raise ValueError("Boolean values are not valid drive parameters")
    try:
        seconds = float(seconds)
        pwm_value = float(pwm)
    except (TypeError, ValueError) as exc:
        raise ValueError("Time and PWM must be numbers") from exc
    if not math.isfinite(seconds) or not MIN_SECONDS <= seconds <= MAX_SECONDS:
        raise ValueError(f"Drive time must be {MIN_SECONDS}..{MAX_SECONDS} seconds")
    if not math.isfinite(pwm_value) or not pwm_value.is_integer():
        raise ValueError("PWM must be an integer")
    pwm = int(pwm_value)
    if not MIN_PWM <= pwm <= MAX_PWM:
        raise ValueError(f"PWM must be {MIN_PWM}..{MAX_PWM}")
    return seconds, pwm


def save(seconds, pwm, path=CONFIG_PATH):
    seconds, pwm = validate(seconds, pwm)
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps({"seconds": seconds, "pwm": pwm}, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)
    return seconds, pwm


def load(path=CONFIG_PATH):
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return validate(data["seconds"], data["pwm"])
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(
            "Junction drive not configured. Run python3 calibrate.py "
            "and enter the tested duration and PWM."
        ) from exc

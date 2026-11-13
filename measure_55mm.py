#!/usr/bin/env python3
"""Calculate 55 mm timing from measured, marked runs; never drives motors.

Measure the complete stop/start/stop movement at the same PWM used for L90/R90.
For two different commanded durations, mark the final axle positions in mm.
The difference gives steady speed; the intercept includes launch and braking.
"""

import argparse
import json
import math
import time
from pathlib import Path

TARGET_MM = 55.0
WHEEL_DIAMETER_MM = 67.0


def estimate(t1, d1, t2, d2):
    if not 0 < t1 < t2 or d2 <= d1:
        raise ValueError("Use two increasing times and distances")
    speed = (d2 - d1) / (t2 - t1)
    intercept = d1 - speed * t1
    duration = (TARGET_MM - intercept) / speed
    return speed, intercept, duration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pwm", type=int, default=80)
    parser.add_argument("--first", nargs=2, type=float, metavar=("SECONDS", "MM"))
    parser.add_argument("--second", nargs=2, type=float, metavar=("SECONDS", "MM"))
    parser.add_argument("--run", type=float, metavar="SECONDS", help="Execute one straight measurement run")
    parser.add_argument("--execute", action="store_true", help="Enable actual motor movement")
    parser.add_argument("--save", action="store_true", help="Save the computed profile next to this script")
    args = parser.parse_args()
    if args.run is not None:
        if not args.execute:
            parser.error("A motor run requires --execute")
        if not 0 < args.run <= 2 or not 70 <= args.pwm <= 140:
            parser.error("Use duration 0..2 s and PWM 70..140")
        from telemetrix import telemetrix
        import motors
        from stm32_reset import reset_stm32
        board = None
        try:
            reset_stm32()
            board = telemetrix.Telemetrix(com_port="/dev/serial0")
            motors.initialize(board)
            input("Mark axle position, clear the path, then press Enter: ")
            started = time.monotonic()
            while time.monotonic() - started < args.run:
                motors.set_speeds(args.pwm, args.pwm)
                time.sleep(0.005)
            motors.motor_stop()
            print(f"PWM={args.pwm}, commanded={args.run:.3f} s. Measure final axle travel in mm.")
        finally:
            if board is not None:
                try:
                    motors.motor_stop()
                finally:
                    board.shutdown()
        return
    if args.first is None or args.second is None:
        parser.error("Supply --first SECONDS MM and --second SECONDS MM, or --run SECONDS --execute")
    speed, intercept, duration = estimate(*args.first, *args.second)
    if not all(math.isfinite(x) for x in (speed, intercept, duration)) or duration <= 0:
        parser.error("Measurements do not yield a positive duration")
    circumference = math.pi * WHEEL_DIAMETER_MM
    print(f"PWM {args.pwm}: speed {speed:.1f} mm/s; start/stop offset {intercept:+.1f} mm")
    print(f"55 mm = {TARGET_MM / circumference:.3f} wheel revolutions")
    print(f"Commanded duration for 55 mm: {duration:.3f} s")
    print("motion_calibration.json values for this PWM (combine with existing PWM entries):")
    print(f'  "forward_mm_s": {{"{args.pwm}": {speed:.4f}}},')
    print(f'  "stop_mm": {{"{args.pwm}": {max(0.0, intercept):.4f}}},')
    print(f'  "launch_deficit_mm": {{"{args.pwm}": {max(0.0, -intercept):.4f}}}')
    print("Verify the result on the robot before using it for automatic turns.")
    if args.save:
        if args.pwm not in (80, 100, 120, 140) or not 0 < duration <= 2:
            parser.error("Profile requires PWM 80/100/120/140 and a 0..2 s alignment")
        path = Path(__file__).with_name("motion_calibration.json")
        profile = json.loads(path.read_text()) if path.exists() else {}
        for key, value in (("forward_mm_s", speed), ("stop_mm", max(0.0, intercept)),
                           ("launch_deficit_mm", max(0.0, -intercept))):
            profile.setdefault(key, {})[str(args.pwm)] = value
        path.write_text(json.dumps(profile, indent=2) + "\n")
        print(f"Saved: {path}")


if __name__ == "__main__":
    main()

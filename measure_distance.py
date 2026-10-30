#!/usr/bin/env python3
"""Read the same VL53L0X distance used by the B task in main_ggs_interactive.py.

Run from the project root: python3 measure_distance.py
This program does not start the motors, servos, or line follower.
"""

import time

from sensors.vl53l0x_sensor import VL53L0XSensor

READ_INTERVAL_SECONDS = 0.2


def main():
    sensor = VL53L0XSensor()
    try:
        print("VL53L0X: initializing...")
        sensor.initialize()
        print("Measure from the distance sensor to the obstacle/button.")
        print("Values are corrected with the SAME 25 mm offset as main GGS.")
        print("Move the robot by hand (motors stay off). Press Ctrl+C to stop.\n")

        while True:
            distance_mm = sensor.read_distance_mm()
            print(
                f"Distance: {distance_mm:4d} mm | {distance_mm / 10:5.1f} cm",
                flush=True,
            )
            time.sleep(READ_INTERVAL_SECONDS)

    except KeyboardInterrupt:
        print("\nMeasurement stopped.")
    except Exception as exc:
        print(f"Sensor error: {exc}")
        raise
    finally:
        sensor.close()


if __name__ == "__main__":
    main()

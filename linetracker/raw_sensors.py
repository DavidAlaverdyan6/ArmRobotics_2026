#!/usr/bin/env python3

import time

from telemetrix import telemetrix
import pins


SERIAL_PORT = "/dev/serial0"

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

sensor_values = [0] * 8


def make_callback(index):
    def callback(data):
        try:
            sensor_values[index] = int(data[2])
        except (IndexError, TypeError, ValueError):
            pass

    return callback


board = telemetrix.Telemetrix(SERIAL_PORT)

for i, pin in enumerate(SENSOR_PINS):
    board.set_pin_mode_analog_input(
        pin,
        differential=0,
        callback=make_callback(i)
    )
    time.sleep(0.05)


print("8-channel raw ADC monitor")
print("Press CTRL+C to stop.")
print()

try:
    while True:
        values = " | ".join(
            f"{name}: {value:4d}"
            for name, value in zip(SENSOR_NAMES, sensor_values)
        )

        print(f"\r{values}", end="", flush=True)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    board.shutdown()

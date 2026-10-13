import time

from telemetrix import telemetrix

from pins import (
    PA0,
    PA1,
    PA2,
    PA3,
    PA4,
    PA5,
    PA6,
    PA7
)


# ============================================================
# 8-CHANNEL LINE TRACKER
# ============================================================

SENSOR_PINS = [
    PA7,
    PA6,
    PA5,
    PA4,
    PA3,
    PA2,
    PA1,
    PA0
]

sensor_values = [0] * 8


# ============================================================
# CALLBACK
# ============================================================

def make_callback(index):

    def callback(data):

        try:
            # Telemetrix analog callback:
            # data[2] contains the ADC value

            sensor_values[index] = data[2]

        except Exception as e:

            print(
                f"\nSensor {index + 1} error:",
                data,
                e
            )

    return callback


# ============================================================
# CONNECT TO STM32
# ============================================================

print("Connecting to STM32...")

board = telemetrix.Telemetrix(
    com_port="/dev/serial0"
)

print("STM32 connected!")


# ============================================================
# INITIALIZE SENSORS
# ============================================================

print()
print("Initializing 8-channel line tracker...")
print()

for i, pin in enumerate(SENSOR_PINS):

    print(
        f"S{i + 1} -> PA{i} -> {pin}"
    )

    board.set_pin_mode_analog_input(
        pin,
        callback=make_callback(i)
    )

print()
print("All sensors initialized!")
print()


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while True:

        print(
            "\r"
            + " | ".join(
                f"S{i + 1}: {value:4d}"
                for i, value
                in enumerate(sensor_values)
            ),
            end="",
            flush=True
        )

        time.sleep(0.1)


# ============================================================
# SAFE SHUTDOWN
# ============================================================

except KeyboardInterrupt:

    print()
    print()
    print("Stopping...")


finally:

    board.shutdown()

    print("STM32 disconnected.")
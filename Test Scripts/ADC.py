import time

from telemetrix import telemetrix
from pins import PB1


def analog_callback(data):
    print("GOT:", data)


print("Connecting to STM32...")

board = telemetrix.Telemetrix(
    com_port="/dev/serial0"
)

print("STM32 connected!")
print(f"Reading PB1 = {PB1}")

board.set_pin_mode_analog_input(
    PB1,
    callback=analog_callback
)

print("Analog input initialized!")

try:
    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    board.shutdown()
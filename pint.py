import time

from telemetrix import telemetrix
from pins import PB0
board = telemetrix.Telemetrix(
    com_port="/dev/serial0"
)
def analog_callback(data):
    print("GOT:", data)
    
print("ABOUT TO CONFIGURE PB0")

board.set_pin_mode_analog_input(
    PB0,
    callback=analog_callback
)

print("PB0 CONFIGURED")
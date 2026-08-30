import time

from telemetrix import telemetrix
from pins import PA0
board = telemetrix.Telemetrix(
    com_port="/dev/serial0"
)
def analog_callback(data):
    print("GOT:", data)
    
print("ABOUT TO CONFIGURE PA0")

board.set_pin_mode_analog_input(
    PA0,
    callback=analog_callback
)

print("PA0 CONFIGURED")
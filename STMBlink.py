import time
from telemetrix import telemetrix

PINS = {'PC13': 31}

board = telemetrix.Telemetrix()
board.set_pin_mode_digital_output(PINS['PC13'])

# Flash PC13 LED 3 times
for i in range(3):
    board.digital_write(PINS['PC13'], 0) # Active LOW
    time.sleep(0.3)
    board.digital_write(PINS['PC13'], 1)
    time.sleep(0.3)

board.shutdown()

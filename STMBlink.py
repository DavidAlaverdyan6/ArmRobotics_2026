import time
from telemetrix import telemetrix
from pins import PC13

board = telemetrix.Telemetrix()
board.set_pin_mode_digital_output(PC13)

for _ in range(3):
    board.digital_write(PC13, 0)
    time.sleep(0.3)
    board.digital_write(PC13, 1)
    time.sleep(0.3)

board.shutdown()

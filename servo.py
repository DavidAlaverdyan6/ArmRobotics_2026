import time
from telemetrix import telemetrix

from pins import PA8, PB13, PB14, PB15, PA8, PA11

board = telemetrix.Telemetrix()

board.set_pin_mode_servo(PB13)
board.set_pin_mode_servo(PB14)
board.set_pin_mode_servo(PB15)
board.set_pin_mode_servo(PA8)
board.set_pin_mode_servo(PA11)


def servo():
    board.servo_write(PB13, 0)
    time.sleep(1)
    board.servo_write(PB13, 90)
    time.sleep(1)
    board.servo_write(PB13, 180)
    time.sleep(1)

try:
    servo()


finally:
    board.shutdown()
import time
from telemetrix import telemetrix
from pins import PB10, PB2


board = telemetrix.Telemetrix(com_port="/dev/serial0")

board.set_pin_mode_digital_output(PB10)
board.set_pin_mode_digital_output(PB2)


def motor_forward():
    board.digital_write(PB10, 1)
    board.digital_write(PB2, 0)


def motor_reverse():
    board.digital_write(PB10, 0)
    board.digital_write(PB2, 1)


def motor_stop():
    board.digital_write(PB10, 0)
    board.digital_write(PB2, 0)


try:
    print("Motor forward")
    motor_forward()
    time.sleep(2)

    print("Stop")
    motor_stop()
    time.sleep(1)

    print("Motor reverse")
    motor_reverse()
    time.sleep(2)

    print("Stop")
    motor_stop()

finally:
    motor_stop()
    board.shutdown()
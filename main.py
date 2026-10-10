import time
from telemetrix import telemetrix
from pins import PB12, PB9, PB8

board = telemetrix.Telemetrix("/dev/serial0")

board.set_pin_mode_digital_output(PB9)
board.set_pin_mode_digital_output(PB8)
board.set_pin_mode_servo(PB12)

def motor_forward():
    board.digital_write(PB9, 1)
    board.digital_write(PB8, 0)


def motor_reverse():
    board.digital_write(PB9, 0)
    board.digital_write(PB8, 1)


def motor_stop():
    board.digital_write(PB9, 0)
    board.digital_write(PB8, 0)

def servo():
    board.servo_write(PB12, 90)
    time.sleep(1)
    board.servo_write(PB12, 0)
    time.sleep(1)
    board.servo_write(PB12, 180)
    time.sleep(3)
    board.servo_write(PB12, 90)

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
    time.sleep(1)
    servo()


finally:
    motor_stop()
    board.shutdown()

import time
from telemetrix import telemetrix
from pins import PB13

print("Connecting to STM32...")

board = telemetrix.Telemetrix(
    com_port="/dev/serial0"
)

print("STM32 connected!")
print(f"Using PB15 = {PB13}")

print("Configuring servo...")
board.set_pin_mode_servo(PB13)

print("Servo initialized!")

time.sleep(2)

print("Moving to 90°...")
board.servo_write(PB13, 90)
time.sleep(1)
board.servo_write(PB13, 0)
print("Moving to 0°...")
time.sleep(1)
board.servo_write(PB13, 180)
time.sleep(3)

board.shutdown()
print("Done.")
#=======
import robot_pin_config as pins


class Servo:
    def __init__(self, board, pin):
        self.board = board
        self.pin = pin
        self.board.set_pin_mode_servo(pin)

    def write(self, angle):
        self.board.servo_write(self.pin, max(0, min(180, int(angle))))


def initialize(board):
    global servo1, servo2, servo3, servo4, servo5

    servo1 = Servo(board, pins.servopan)
    servo2 = Servo(board, pins.servotilt)
    servo3 = Servo(board, pins.servo3)
    servo4 = Servo(board, pins.servo4)
    servo5 = Servo(board, pins.servo_gripper)
#>>>>>>> 2995d10e65a6ee1ae4b1e77f9e92fa7c1ba59a4b

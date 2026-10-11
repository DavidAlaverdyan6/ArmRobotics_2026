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

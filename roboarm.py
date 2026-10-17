
import time

import robot_pin_config as pins


class Servo:
    """Small Telemetrix servo wrapper with a servo.write(angle) API."""

    def __init__(self, board, pin, initial_angle=90):
        self.board = board
        self.pin = pin
        self.angle = initial_angle

        self.board.set_pin_mode_servo(self.pin)

    def write(self, angle):
        angle = int(angle)

        if not 0 <= angle <= 180:
            raise ValueError("Servo angle must be between 0 and 180")

        self.board.servo_write(self.pin, angle)
        self.angle = angle


class RoboArm:
    """Four-axis robot arm with an independently controlled gripper."""

    def __init__(self, board):
        self.board = board

        self.base = Servo(board, pins.servopan)
        self.shoulder = Servo(board, pins.servotilt)
        self.elbow = Servo(board, pins.servo3)
        self.wrist = Servo(board, pins.servo4)
        self.gripper = Servo(board, pins.servo_gripper)

    def write(self, base=None, shoulder=None, elbow=None,
              wrist=None, gripper=None):
        """Command any combination of joints."""
        commands = (
            ("base", base),
            ("shoulder", shoulder),
            ("elbow", elbow),
            ("wrist", wrist),
            ("gripper", gripper),
        )

        for name, angle in commands:
            if angle is not None:
                getattr(self, name).write(angle)

    def close_gripper(self):
        self.gripper.write(180)

    def open_gripper(self):
        self.gripper.write(0)

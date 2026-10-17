
import math
import robot_pin_config as pins


class Servo:
    """Telemetrix servo wrapper. Position is unknown until referenced."""

    def __init__(self, board, pin):
        self.board = board
        self.pin = pin
        self.angle = None
        self.board.set_pin_mode_servo(pin)

    def set_reference(self, angle):
        """Record an observed position without moving the servo."""
        angle = float(angle)
        if not math.isfinite(angle) or not 0 <= angle <= 180:
            raise ValueError("Servo reference must be between 0 and 180")
        self.angle = angle

    def write(self, angle):
        angle = float(angle)
        if not math.isfinite(angle) or not 0 <= angle <= 180:
            raise ValueError("Servo angle must be between 0 and 180")

        self.board.servo_write(self.pin, int(round(angle)))
        self.angle = angle


class RoboArm:
    """Four arm joints and an independent gripper."""

    def __init__(self, board):
        self.board = board
        self.base = Servo(board, pins.servopan)
        self.shoulder = Servo(board, pins.servotilt)
        self.elbow = Servo(board, pins.servo3)
        self.wrist = Servo(board, pins.servo4)
        self.gripper = Servo(board, pins.servo_gripper)

    def set_reference(self, base, shoulder, elbow, wrist, gripper):
        """Record approximate physical positions; sends no servo commands."""
        for name, angle in (
            ("base", base),
            ("shoulder", shoulder),
            ("elbow", elbow),
            ("wrist", wrist),
            ("gripper", gripper),
        ):
            getattr(self, name).set_reference(angle)

    def is_referenced(self):
        return all(
            getattr(self, name).angle is not None
            for name in ("base", "shoulder", "elbow", "wrist", "gripper")
        )

    def write(self, base=None, shoulder=None, elbow=None,
              wrist=None, gripper=None):
        for name, angle in (
            ("base", base),
            ("shoulder", shoulder),
            ("elbow", elbow),
            ("wrist", wrist),
            ("gripper", gripper),
        ):
            if angle is not None:
                getattr(self, name).write(angle)

    def open_gripper(self):
        self.gripper.write(0)

    def close_gripper(self):
        self.gripper.write(180)

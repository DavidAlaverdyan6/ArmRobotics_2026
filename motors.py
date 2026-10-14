
import robot_pin_config as pins

_board = None


def initialize(board):
    global _board
    _board = board

    for pin in (*pins.LEFT_MOTOR, *pins.RIGHT_MOTOR):
        _board.set_pin_mode_analog_output(pin)
        _board.analog_write(pin, 0)


def _write_motor(motor_pins, speed):
    """Signed speed: -255 reverse, 0 stop, +255 forward."""
    speed = max(-255, min(255, int(speed)))
    pin_forward, pin_reverse = motor_pins

    if speed >= 0:
        _board.analog_write(pin_reverse, 0)
        _board.analog_write(pin_forward, speed)
    else:
        _board.analog_write(pin_forward, 0)
        _board.analog_write(pin_reverse, -speed)


def set_speeds(left, right):
    """Independently control left and right motor speeds."""
    if _board is None:
        raise RuntimeError("Call motors.initialize(board) first")

    _write_motor(pins.LEFT_MOTOR, left)
    _write_motor(pins.RIGHT_MOTOR, right)


def motor_forward(speed=180):
    set_speeds(speed, speed)


def motor_reverse(speed=150):
    set_speeds(-speed, -speed)


def motor_left(speed=150):
    """Pivot left."""
    set_speeds(-speed, speed)


def motor_right(speed=150):
    """Pivot right."""
    set_speeds(speed, -speed)


def motor_stop():
    set_speeds(0, 0)

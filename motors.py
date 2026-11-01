
import robot_pin_config as pins
import threading

_board = None
_guarded = False
_armed = False
_owner_thread = None
_freshness_check = None


def initialize(board):
    global _board, _armed
    _board = board
    _armed = False

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
    if _guarded and (left != 0 or right != 0):
        if not _armed or threading.get_ident() != _owner_thread:
            raise RuntimeError("Motor command rejected: drive is not armed for this task")
        if _freshness_check is not None and not _freshness_check():
            disarm()
            raise RuntimeError("Motor command rejected: ADC data are stale")

    try:
        _write_motor(pins.LEFT_MOTOR, left)
        _write_motor(pins.RIGHT_MOTOR, right)
    except Exception:
        try:
            _write_motor(pins.LEFT_MOTOR, 0)
            _write_motor(pins.RIGHT_MOTOR, 0)
        finally:
            raise


def enable_guard(freshness_check=None):
    """Main opts in; legacy standalone motor diagnostics keep their API."""
    global _guarded, _armed, _owner_thread, _freshness_check
    motor_stop()
    _guarded = True
    _armed = False
    _owner_thread = threading.get_ident()
    _freshness_check = freshness_check


def arm():
    global _armed
    if not _guarded or threading.get_ident() != _owner_thread:
        raise RuntimeError("Drive guard is not owned by this task")
    _armed = True


def disarm():
    global _armed
    _armed = False
    motor_stop()


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

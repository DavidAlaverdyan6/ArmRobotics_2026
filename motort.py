import time
from telemetrix import telemetrix
import motors

board = telemetrix.Telemetrix("/dev/serial0")

try:
    motors.initialize(board)

    for name, action in [
        ("Forward", lambda: motors.motor_forward(150)),
        ("Reverse", lambda: motors.motor_reverse(150)),
        ("Pivot left", lambda: motors.motor_left(150)),
        ("Pivot right", lambda: motors.motor_right(150)),
    ]:
        input(f"Press Enter to test {name}...")
        action()
        time.sleep(1)
        motors.motor_stop()

finally:
    motors.motor_stop()
    board.shutdown()

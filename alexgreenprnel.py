
import time
from telemetrix import telemetrix

SERVO_PIN = 12  # STM32 PB12

board = None

try:
    board = telemetrix.Telemetrix(
        com_port="/dev/serial0"
    )

    time.sleep(1)

    board.set_pin_mode_servo(SERVO_PIN)

    time.sleep(1)

    print("Servo test started")

    while True:
        for angle in [90, 60, 120, 90]:
            print(f"Angle: {angle}")
            board.servo_write(SERVO_PIN, angle)
            time.sleep(2)

except KeyboardInterrupt:
    print("Stopped")

finally:
    if board is not None:
        board.shutdown()

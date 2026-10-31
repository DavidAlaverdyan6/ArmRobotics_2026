import time
from telemetrix import telemetrix


PORT = "/dev/serial0"
B5 = 21  # Based on your pins.py mapping

board = telemetrix.Telemetrix(com_port=PORT)

try:
    board.set_pin_mode_digital_output(B5)
    time.sleep(0.5)

    print("Blinking B5. Press Ctrl+C to stop.")

    while True:
        board.digital_write(B5, 1)
        time.sleep(4)

        board.digital_write(B5, 0)
        time.sleep(3)

except KeyboardInterrupt:
    print("\nStopping.")

finally:
    board.digital_write(B5, 0)
    board.shutdown()
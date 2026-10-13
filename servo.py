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
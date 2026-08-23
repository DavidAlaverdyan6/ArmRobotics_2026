import time
from telemetrix import telemetrix
from pins import PA8

print("Connecting to STM32...")

board = telemetrix.Telemetrix(
    com_port="/dev/serial0"
)

print("STM32 connected!")
print(f"Using PA8 = {PA8}")

print("Configuring servo...")
board.set_pin_mode_servo(PA8)

print("Servo initialized!")

time.sleep(2)

print("Moving to 90°...")
board.servo_write(PA8, 90)
time.sleep(1)
board.servo_write(PA8, 0)
time.sleep(1)
board.servo_write(PA8, 180)
time.sleep(3)

board.shutdown()
print("Done.")
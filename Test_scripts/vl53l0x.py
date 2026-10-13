import time
import board
import busio
import adafruit_vl53l0x

i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

print("VL53L0X connected")

while True:
    d = vl53.range
    print(f"Raw: {d:4d} mm | Corrected: {max(0, d - 25):4d} mm")

    time.sleep(0.05)
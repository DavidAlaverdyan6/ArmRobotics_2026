import time
import board
import busio
import adafruit_vl53l0x


i2c = busio.I2C(board.SCL, board.SDA)

vl53 = adafruit_vl53l0x.VL53L0X(i2c)

print("VL53L0X connected!")
print("Reading distance...\n")

while True:
    distance = vl53.range

    print(f"Distance: {distance:4d} mm")

    time.sleep(0.05)
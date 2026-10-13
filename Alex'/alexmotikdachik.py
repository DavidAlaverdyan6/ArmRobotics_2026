import RPi.GPIO as GPIO
import time

IR_PIN = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_PIN, GPIO.IN)

try:
    while True:
        if GPIO.input(IR_PIN):
            print("Object detected")
        else:
            print("No object detected")

        time.sleep(0.1)

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
import led
import robot_pin_config as pins
import time

rgb = led.RGBLED(
    pins.LED_RED,
    pins.LED_GREEN,
    pins.LED_BLUE,
)

rgb.green()
time.sleep(1)
rgb.red()
time.sleep(1)
rgb.blue()
time.sleep(1)

# During shutdown:
rgb.off()
rgb.close()
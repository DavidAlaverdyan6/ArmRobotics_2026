
# led.py
from gpiozero import RGBLED as _RGBLED
from gpiozero import OutputDevice


class RGBLED:
    """Common-anode RGB LED controlled by Raspberry Pi GPIO."""

    def __init__(self, red_pin, green_pin, blue_pin):
        # Common-anode LEDs are active-low.
        self.led = _RGBLED(
            red=red_pin,
            green=green_pin,
            blue=blue_pin,
            active_high=False,
        )

    def set_color(self, red=False, green=False, blue=False):
        self.led.color = (
            1.0 if red else 0.0,
            1.0 if green else 0.0,
            1.0 if blue else 0.0,
        )

    def off(self):
        self.led.off()

    def red(self):
        self.set_color(red=True)

    def green(self):
        self.set_color(green=True)

    def blue(self):
        self.set_color(blue=True)

    def yellow(self):
        self.set_color(red=True, green=True)

    def cyan(self):
        self.set_color(green=True, blue=True)

    def magenta(self):
        self.set_color(red=True, blue=True)

    def white(self):
        self.set_color(red=True, green=True, blue=True)

    def close(self):
        self.led.close()

import unittest

import stm32_reset


class FakeGPIO:
    BCM = "BCM"
    OUT = "OUT"
    IN = "IN"
    LOW = 0
    PUD_OFF = "OFF"

    def __init__(self):
        self.calls = []

    def setwarnings(self, value):
        self.calls.append(("setwarnings", value))

    def setmode(self, mode):
        self.calls.append(("setmode", mode))

    def setup(self, pin, direction, **kwargs):
        self.calls.append(("setup", pin, direction, kwargs))


class STM32ResetTests(unittest.TestCase):
    def test_reset_pulls_nrst_low_then_releases_it(self):
        gpio = FakeGPIO()
        sleeps = []

        stm32_reset.reset_stm32(
            gpio=gpio,
            sleep=sleeps.append,
            pulse_seconds=0.12,
            boot_seconds=1.5,
        )

        self.assertEqual(gpio.calls, [
            ("setwarnings", False),
            ("setmode", "BCM"),
            ("setup", 23, "OUT", {"initial": 0}),
            ("setup", 23, "IN", {"pull_up_down": "OFF"}),
        ])
        self.assertEqual(sleeps, [0.12, 1.5])

    def test_reset_releases_nrst_if_sleep_fails(self):
        gpio = FakeGPIO()

        def fail(_seconds):
            raise RuntimeError("interrupted")

        with self.assertRaises(RuntimeError):
            stm32_reset.reset_stm32(gpio=gpio, sleep=fail)

        self.assertEqual(
            gpio.calls[-1],
            ("setup", 23, "IN", {"pull_up_down": "OFF"}),
        )


if __name__ == "__main__":
    unittest.main()


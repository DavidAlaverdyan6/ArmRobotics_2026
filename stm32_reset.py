"""Hardware reset for the STM32 connected to the Raspberry Pi.

The STM32 NRST input is active-low and already has a pull-up on the board.
BCM23 only ever drives the line low; changing it back to an input releases the
line instead of driving it high against another reset source.
"""

import time


STM32_RESET_GPIO = 23
RESET_PULSE_SECONDS = 0.10
STM32_BOOT_SECONDS = 2.0


def reset_stm32(
    pin=STM32_RESET_GPIO,
    pulse_seconds=RESET_PULSE_SECONDS,
    boot_seconds=STM32_BOOT_SECONDS,
    gpio=None,
    sleep=time.sleep,
):
    """Pulse STM32 NRST low and wait for its firmware to become ready."""
    if gpio is None:
        import RPi.GPIO as gpio

    if pulse_seconds <= 0:
        raise ValueError("reset pulse must be longer than zero")
    if boot_seconds < 0:
        raise ValueError("boot delay cannot be negative")

    gpio.setwarnings(False)
    gpio.setmode(gpio.BCM)
    try:
        # Set the output value atomically with the direction. Never drive NRST
        # high: its board pull-up releases reset when this pin becomes input.
        gpio.setup(pin, gpio.OUT, initial=gpio.LOW)
        sleep(pulse_seconds)
    finally:
        gpio.setup(pin, gpio.IN, pull_up_down=gpio.PUD_OFF)

    sleep(boot_seconds)


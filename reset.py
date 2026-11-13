#!/usr/bin/env python3
"""Reset the STM32 board through Raspberry Pi BCM23."""

from stm32_reset import (
    RESET_PULSE_SECONDS,
    STM32_BOOT_SECONDS,
    STM32_RESET_GPIO,
    reset_stm32,
)


def main():
    print(
        f"Resetting STM32 via Raspberry Pi BCM{STM32_RESET_GPIO} "
        f"({RESET_PULSE_SECONDS:.2f}s pulse)..."
    )
    reset_stm32()
    print(f"STM32 reset complete; boot wait was {STM32_BOOT_SECONDS:.1f}s.")


if __name__ == "__main__":
    main()

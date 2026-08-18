import time

from telemetrix import telemetrix
from vl53l0x import VL53L0X


board = telemetrix.Telemetrix()

sensor = None

try:
    print("Initializing I2C...")
    board.set_pin_mode_i2c()

    time.sleep(0.1)

    sensor = VL53L0X(
        board,
        address=0x29,
        io_timeout_ms=1000,
    )

    sensor.begin()

    print("\nVL53L0X diagnostic registers:")

    registers = [
        0x00,
        0x01,
        0x06,
        0x0A,
        0x0B,
        0x13,
        0x14,
        0x44,
        0x50,
        0x51,
        0x60,
        0x70,
        0x71,
        0x89,
        0x91,
    ]

    for reg in registers:
        try:
            value = sensor._read8(reg)
            print(f"0x{reg:02X}: 0x{value:02X}")
        except Exception as e:
            print(f"0x{reg:02X}: ERROR: {e}")

    print("\nStarting measurement...")

    sensor._write8(0x00, 0x01)

    time.sleep(0.05)

    for _ in range(20):
        interrupt = sensor._read8(0x13)
        status = sensor._read8(0x14)

        print(
            f"INT=0x{interrupt:02X} "
            f"STATUS=0x{status:02X}"
        )

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    if sensor is not None:
        try:
            sensor.stop_continuous()
        except Exception:
            pass

    board.shutdown()
#!/usr/bin/env python3
"""One-turn MPU-6050 diagnostic. Runs motors only when invoked with --execute."""

import argparse

def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('direction', choices=('L', 'R'))
    parser.add_argument('--execute', action='store_true',
                        help='perform exactly one physical 90 degree turn')
    args = parser.parse_args(argv)
    if not args.execute:
        print('Diagnostic prepared. Add --execute only with robot on a safe test area.')
        return
    from telemetrix import telemetrix
    import smbus
    import motors
    from main import (SERIAL_PORT, I2C_BUS, gyro_setup, gyro_calibrate,
                      pivot)
    board = bus = None
    try:
        board = telemetrix.Telemetrix(com_port=SERIAL_PORT)
        motors.initialize(board)
        motors.enable_guard()
        bus = smbus.SMBus(I2C_BUS)
        gyro_setup(bus)
        offset = gyro_calibrate(bus)
        motors.arm()
        ok, _ = pivot(bus, offset, args.direction, angle_target=90.0)
        motors.disarm()
        measured_text = input('External measured angle in degrees (Enter if unavailable): ').strip()
        measured = float(measured_text) if measured_text else None
        message = (f'Diagnostic direction={args.direction}, target=90.0 deg, '
              f'gyro status={"within 8 deg" if ok else "outside 8 deg"}')
        if measured is not None:
            message += f', external={measured:.2f} deg, external error={measured-90:+.2f} deg'
        else:
            message += ', external angle not recorded'
        print(message)
    finally:
        if board is not None:
            try:
                motors.disarm()
            finally:
                board.shutdown()
        if bus is not None:
            bus.close()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import line_tracker


def main():
    try:
        line_tracker.initialize("/dev/serial0")
        line_tracker.calibrate()

    except KeyboardInterrupt:
        print()
        print("Calibration cancelled.")

    finally:
        line_tracker.shutdown()


if __name__ == "__main__":
    main()

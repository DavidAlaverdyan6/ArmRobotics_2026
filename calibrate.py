#!/usr/bin/env python3

from telemetrix import telemetrix
import line_tracker

PORT = "/dev/serial0"


def main():
    board = None

    try:
        print("Միացվում է սենսորների համակարգը...")

        board = telemetrix.Telemetrix(com_port=PORT)
        line_tracker.initialize(board)

        print("\nՍկսվում է կալիբրովկան։")
        print("Սենսորները շարժիր սև և սպիտակ մակերեսների վրայով։")
        print("Մի գործարկիր մատոռները։\n")

        line_tracker.calibrate()

        print("\nԿալիբրովկան ավարտված է։")

    except KeyboardInterrupt:
        print("\nԿալիբրովկան դադարեցվեց։")

    except Exception as e:
        print(f"\nՍխալ՝ {e}")

    finally:
        try:
            line_tracker.shutdown()
        except Exception:
            pass

        if board is not None:
            board.shutdown()


if __name__ == "__main__":
    main()
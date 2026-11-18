import time

from color_marker_detector import ContinuousCamera


def main():
    camera = ContinuousCamera(port=8000)
    camera.start()

    try:
        print("Камера запущена на порту 8000. Для остановки нажми Ctrl+C.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()


if __name__ == "__main__":
    main()

import cv2
import time
import threading
import logging
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from picamera2 import Picamera2
from telemetrix import telemetrix


# ==================== SETTINGS ====================

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8000

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
JPEG_QUALITY = 80

# L298N motor pins
IN1 = 4
IN2 = 17
IN3 = 14
IN4 = 15

# Rotate image 180 degrees
ROTATE_CAMERA_180 = True


# ==================== GLOBALS ====================

latest_frame = None
frame_lock = threading.Lock()

camera = None
board = None

motors_ready = False
running = True


# ==================== CAMERA ====================

def initialize_camera():
    global camera

    print("Starting camera...")

    camera = Picamera2()

    config = camera.create_video_configuration(
        main={
            "size": (CAMERA_WIDTH, CAMERA_HEIGHT),
            "format": "RGB888"
        },
        buffer_count=4
    )

    camera.configure(config)
    camera.start()

    time.sleep(2)

    print("Camera ready!")


def camera_worker():
    global latest_frame, running

    while running:
        try:
            frame = camera.capture_array()

            # Convert RGB to BGR for OpenCV
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame = cv2.flip(frame, -1)
            # Rotate the image 180 degrees.
            # The rotated frame is also the one sent to the server.
            if ROTATE_CAMERA_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)

            with frame_lock:
                latest_frame = frame

        except Exception as error:
            print("Camera error:", error)
            time.sleep(0.1)


# ==================== HTTP STREAM ====================

class CameraHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path not in ("/", "/video", "/stream"):
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame"
        )
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        try:
            while running:
                with frame_lock:
                    frame = (
                        latest_frame.copy()
                        if latest_frame is not None
                        else None
                    )

                if frame is None:
                    time.sleep(0.05)
                    continue

                success, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
                )

                if not success:
                    continue

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(
                    b"Content-Type: image/jpeg\r\n\r\n"
                )
                self.wfile.write(encoded.tobytes())
                self.wfile.write(b"\r\n")

                time.sleep(0.02)

        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def log_message(self, format_string, *args):
        return


def start_server():
    server = ThreadingHTTPServer(
        (SERVER_HOST, SERVER_PORT),
        CameraHandler
    )

    server.daemon_threads = True

    print(
        f"CAMERA SERVER STARTED: "
        f"http://<RASPBERRY_PI_IP>:{SERVER_PORT}/video"
    )

    server.serve_forever()


# ==================== STM32 / TELEMETRIX ====================

def initialize_motors():
    global board, motors_ready

    print("Connecting to STM32...")

    board = telemetrix.Telemetrix(
        com_port="/dev/serial0"
    )

    time.sleep(4)

    for pin in (IN1, IN2, IN3, IN4):
        board.set_pin_mode_digital_output(pin)

    time.sleep(0.5)

    # Stop motors at startup
    stop_motors()

    motors_ready = True
    print("STM32 connected!")
    print("L298N initialized!")


def set_pin(pin, value):
    if board is not None and motors_ready:
        board.digital_write(pin, value)


# ==================== MOTOR CONTROL ====================

def stop_motors():
    if board is None:
        return

    for pin in (IN1, IN2, IN3, IN4):
        try:
            board.digital_write(pin, 0)
        except Exception:
            pass


def forward():
    set_pin(IN1, 1)
    set_pin(IN2, 0)
    set_pin(IN3, 1)
    set_pin(IN4, 0)


def backward():
    set_pin(IN1, 0)
    set_pin(IN2, 1)
    set_pin(IN3, 0)
    set_pin(IN4, 1)


def turn_left():
    set_pin(IN1, 0)
    set_pin(IN2, 1)
    set_pin(IN3, 1)
    set_pin(IN4, 0)


def turn_right():
    set_pin(IN1, 1)
    set_pin(IN2, 0)
    set_pin(IN3, 0)
    set_pin(IN4, 1)


# ==================== MAIN ====================

def main():
    global running

    try:
        initialize_camera()

        threading.Thread(
            target=camera_worker,
            daemon=True
        ).start()

        # Start HTTP server in the background
        threading.Thread(
            target=start_server,
            daemon=True
        ).start()

        initialize_motors()

        print("System ready!")
        print("Open the camera stream at:")
        print("http://<RASPBERRY_PI_IP>:8000/video")
        print("Motors are stopped until a command is issued.")
        print("Press Ctrl+C to exit.")

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping system...")

    except Exception as error:
        logging.exception("Startup/runtime error: %s", error)

    finally:
        running = False

        try:
            stop_motors()
        except Exception:
            pass

        if camera is not None:
            try:
                camera.stop()
                camera.close()
            except Exception:
                pass

        if board is not None:
            try:
                board.shutdown()
            except Exception:
                pass

        print("System stopped.")


if __name__ == "__main__":
    main()
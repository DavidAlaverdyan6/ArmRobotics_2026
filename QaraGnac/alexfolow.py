
import time
import threading
import io
import numpy as np
import smbus

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from picamera2 import Picamera2
from telemetrix import telemetrix
from pins import PB9, PB8, PB7, PB6

# ============================================================
# CONFIGURATION
# ============================================================

WIDTH = 416
HEIGHT = 416
THRESHOLD = 128

ROI_TOP = 300
ROI_BOTTOM = 400
MIN_LINE_PIXELS = 20
DEAD_ZONE = 25
LOOP_DELAY = 0.02

# THREE-POINT INTERSECTION DETECTOR
DOT_CENTER_X = WIDTH // 2
DOT_CENTER_Y = 300
DOT_HORIZONTAL_SPACING = 150
DOT_VERTICAL_OFFSET = 100
DOT_RADIUS = 5
DOT_BLACK_RATIO = 0.50

CROSSING_CONFIRM_FRAMES = 3
CROSSING_COOLDOWN = 0.8

CROSSING_POINTS = {
    "FORWARD": (
        DOT_CENTER_X,
        DOT_CENTER_Y - DOT_VERTICAL_OFFSET
    ),
    "LEFT": (
        DOT_CENTER_X - DOT_HORIZONTAL_SPACING,
        DOT_CENTER_Y
    ),
    "RIGHT": (
        DOT_CENTER_X + DOT_HORIZONTAL_SPACING,
        DOT_CENTER_Y
    )
}

# L298N PINS
LEFT_IN1 = PB9
LEFT_IN2 = PB8
RIGHT_IN1 = PB7
RIGHT_IN2 = PB6

# TURN CONFIGURATION
TURN_ANGLE = 60.0
TURN_CORRECTION = 1.0
GYRO_TURN_TARGET = TURN_ANGLE * TURN_CORRECTION
TURN_UPDATE_DELAY = 0.005
MAX_TURN_TIME = 4.0
EXIT_TIME = 0.15

# LINE LOST RECOVERY
BACKUP_TIME = 0.30
BACKUP_DELAY = 0.05

# MPU6050
MPU6050 = 0x68
GYRO_SCALE = 131.0
CALIBRATION_SAMPLES = 500
CALIBRATION_DELAY = 0.002

# ============================================================
# GLOBAL STATE
# ============================================================

server_running = True
latest_jpeg = None
jpeg_lock = threading.Lock()

picam2 = None
board = None
bus = None

# ============================================================
# CAMERA INITIALIZATION
# ============================================================

print("Starting camera...")

picam2 = Picamera2()

picam2.configure(
    picam2.create_preview_configuration(
        main={
            "size": (WIDTH, HEIGHT),
            "format": "RGB888"
        },
        buffer_count=4
    )
)

picam2.start()
time.sleep(1)

print("Camera ready")

# ============================================================
# CAMERA SERVER
# ============================================================

def camera_server_frame():
    global latest_jpeg

    from PIL import Image

    while server_running:
        try:
            frame = picam2.capture_array()

            # Picamera2 RGB888 output
            rgb = frame[:, :, :3]

            image = Image.fromarray(rgb)

            buffer = io.BytesIO()
            image.save(buffer, format="JPEG", quality=70)

            jpeg = buffer.getvalue()

            with jpeg_lock:
                latest_jpeg = jpeg

        except Exception as exc:
            if server_running:
                print("Camera server error:", exc)
            time.sleep(0.1)


class CameraHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path == "/":
            page = """
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Camera</title>
<style>
body {
    background: #111;
    color: white;
    font-family: Arial, sans-serif;
    text-align: center;
    margin: 0;
    padding: 20px;
}
img {
    width: 832px;
    max-width: 95vw;
    height: auto;
}
</style>
</head>
<body>
<h1>Robot Camera</h1>
<img src="/stream" alt="Robot camera stream">
</body>
</html>
"""
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(page.encode("utf-8"))
            return

        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, no-store")
            self.send_header("Pragma", "no-cache")
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame"
            )
            self.end_headers()

            try:
                while server_running:
                    with jpeg_lock:
                        jpeg = latest_jpeg

                    if jpeg is None:
                        time.sleep(0.02)
                        continue

                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(
                        b"Content-Type: image/jpeg\r\n"
                    )
                    self.wfile.write(
                        f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                    )
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()

                    time.sleep(0.03)

            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

            return

        self.send_error(404)

    def log_message(self, format, *args):
        pass


def start_web_server():
    try:
        server = ThreadingHTTPServer(
            ("0.0.0.0", 8000),
            CameraHandler
        )

        server.daemon_threads = True

        print("\n========================================")
        print("CAMERA SERVER STARTED")
        print("Open: http://<PI-IP>:8000")
        print("========================================\n")

        while server_running:
            server.handle_request()

        server.server_close()

    except Exception as exc:
        if server_running:
            print("Web server error:", exc)


camera_thread = threading.Thread(
    target=camera_server_frame,
    daemon=True
)
camera_thread.start()

web_thread = threading.Thread(
    target=start_web_server,
    daemon=True
)
web_thread.start()

# ============================================================
# STM32 INITIALIZATION
# ============================================================

print("Connecting to STM32...")

board = telemetrix.Telemetrix(
    com_port="/dev/serial0"
)

print("STM32 connected!")

# ============================================================
# MOTOR INITIALIZATION
# ============================================================

board.set_pin_mode_digital_output(LEFT_IN1)
board.set_pin_mode_digital_output(LEFT_IN2)
board.set_pin_mode_digital_output(RIGHT_IN1)
board.set_pin_mode_digital_output(RIGHT_IN2)

print("L298N initialized!")

# ============================================================
# MOTOR CONTROL
# ============================================================

def left_forward():
    board.digital_write(LEFT_IN1, 1)
    board.digital_write(LEFT_IN2, 0)


def left_backward():
    board.digital_write(LEFT_IN1, 0)
    board.digital_write(LEFT_IN2, 1)


def left_stop():
    board.digital_write(LEFT_IN1, 0)
    board.digital_write(LEFT_IN2, 0)


def right_forward():
    board.digital_write(RIGHT_IN1, 1)
    board.digital_write(RIGHT_IN2, 0)


def right_backward():
    board.digital_write(RIGHT_IN1, 0)
    board.digital_write(RIGHT_IN2, 1)


def right_stop():
    board.digital_write(RIGHT_IN1, 0)
    board.digital_write(RIGHT_IN2, 0)


def forward():
    left_forward()
    right_forward()


def backward():
    left_backward()
    right_backward()


def stop():
    left_stop()
    right_stop()


def line_turn_left():
    left_stop()
    right_forward()


def line_turn_right():
    left_forward()
    right_stop()


# ============================================================
# MPU6050 INITIALIZATION
# ============================================================

print("\nInitializing MPU-6050...")

bus = smbus.SMBus(1)
bus.write_byte_data(MPU6050, 0x6B, 0)
time.sleep(0.1)

print("MPU-6050 initialized.")


def read_word(reg):
    high = bus.read_byte_data(MPU6050, reg)
    low = bus.read_byte_data(MPU6050, reg + 1)

    value = (high << 8) | low

    if value >= 32768:
        value -= 65536

    return value


# ============================================================
# GYROSCOPE CALIBRATION
# ============================================================

print("\n========================================")
print("CALIBRATING MPU-6050")
print("DO NOT MOVE THE ROBOT!")
print("========================================")

gx_offset = 0.0
gy_offset = 0.0
gz_offset = 0.0

for i in range(CALIBRATION_SAMPLES):
    gx_offset += read_word(0x43)
    gy_offset += read_word(0x45)
    gz_offset += read_word(0x47)
    time.sleep(CALIBRATION_DELAY)

gx_offset /= CALIBRATION_SAMPLES
gy_offset /= CALIBRATION_SAMPLES
gz_offset /= CALIBRATION_SAMPLES

print("Gyro calibration complete.")
print(f"GX offset: {gx_offset:.2f}")
print(f"GY offset: {gy_offset:.2f}")
print(f"GZ offset: {gz_offset:.2f}")

angle_z = 0.0
last_gyro_time = time.monotonic()


def update_gyro():
    global angle_z, last_gyro_time

    current_time = time.monotonic()
    dt = current_time - last_gyro_time
    last_gyro_time = current_time

    if dt <= 0:
        return angle_z

    gz = (read_word(0x47) - gz_offset) / GYRO_SCALE
    angle_z += gz * dt

    return angle_z


def get_relative_angle(start_angle):
    return update_gyro() - start_angle


# ============================================================
# GYRO TURN FUNCTIONS
# ============================================================

def gyro_turn_left():
    print("\nGYRO TURN: LEFT")
    print(f"Target: {GYRO_TURN_TARGET:.1f} degrees")

    stop()
    time.sleep(0.05)

    start_angle = update_gyro()
    start_time = time.monotonic()

    while True:
        relative_angle = get_relative_angle(start_angle)
        elapsed = time.monotonic() - start_time

        if abs(relative_angle) >= GYRO_TURN_TARGET:
            break

        if elapsed >= MAX_TURN_TIME:
            print("WARNING: LEFT turn timeout!")
            break

        left_stop()
        right_forward()

        time.sleep(TURN_UPDATE_DELAY)

    stop()

    final_angle = abs(get_relative_angle(start_angle))

    print("LEFT turn finished.")
    print(f"Measured rotation: {final_angle:.2f} degrees")


def gyro_turn_right():
    print("\nGYRO TURN: RIGHT")
    print(f"Target: {GYRO_TURN_TARGET:.1f} degrees")

    stop()
    time.sleep(0.05)

    start_angle = update_gyro()
    start_time = time.monotonic()

    while True:
        relative_angle = get_relative_angle(start_angle)
        elapsed = time.monotonic() - start_time

        if abs(relative_angle) >= GYRO_TURN_TARGET:
            break

        if elapsed >= MAX_TURN_TIME:
            print("WARNING: RIGHT turn timeout!")
            break

        left_forward()
        right_stop()

        time.sleep(TURN_UPDATE_DELAY)

    stop()

    final_angle = abs(get_relative_angle(start_angle))

    print("RIGHT turn finished.")
    print(f"Measured rotation: {final_angle:.2f} degrees")


# ============================================================
# IMAGE PROCESSING
# ============================================================

def threshold_image(frame):
    image = frame[:, :, :3]

    gray = np.mean(image, axis=2)

    return gray < THRESHOLD


def find_line(frame):
    image = frame[:, :, :3]
    gray = np.mean(image, axis=2)
    black = gray < THRESHOLD

    roi = black[ROI_TOP:ROI_BOTTOM, :]

    y, x = np.where(roi)

    if len(x) < MIN_LINE_PIXELS:
        return None

    return float(np.mean(x))


def sample_point(black, x, y):
    x1 = max(0, x - DOT_RADIUS)
    x2 = min(WIDTH, x + DOT_RADIUS + 1)

    y1 = max(0, y - DOT_RADIUS)
    y2 = min(HEIGHT, y + DOT_RADIUS + 1)

    region = black[y1:y2, x1:x2]

    if region.size == 0:
        return False

    black_pixels = np.count_nonzero(region)
    black_ratio = black_pixels / region.size

    return black_ratio >= DOT_BLACK_RATIO


def read_crossing_points(black):
    states = {}

    for name, (x, y) in CROSSING_POINTS.items():
        states[name] = sample_point(black, x, y)

    return states


def print_pattern(states):
    print()
    print(f"              {'●' if states['FORWARD'] else '○'}")
    print()
    print(
        f"      {'●' if states['LEFT'] else '○'}"
        f"                 "
        f"{'●' if states['RIGHT'] else '○'}"
    )
    print()


def is_intersection(states):
    detected = (
        int(states["LEFT"])
        + int(states["RIGHT"])
        + int(states["FORWARD"])
    )

    return detected >= 2


# ============================================================
# DIRECTION SELECTION
# ============================================================

def ask_direction():
    while True:
        direction = input(
            "Choose direction [L/R/F]: "
        ).strip().upper()

        if direction in ("L", "R", "F"):
            return direction

        print("Invalid direction. Enter L, R or F.")


def execute_direction(direction):
    print(f"\nExecuting direction: {direction}")

    if direction == "L":
        gyro_turn_left()
        stop()
        time.sleep(0.05)
        forward()
        time.sleep(EXIT_TIME)
        stop()

    elif direction == "R":
        gyro_turn_right()
        stop()
        time.sleep(0.05)
        forward()
        time.sleep(EXIT_TIME)
        stop()

    elif direction == "F":
        print("Going STRAIGHT...")
        forward()


# ============================================================
# LINE LOST RECOVERY
# ============================================================

def recover_line():
    print("\nLINE LOST - REVERSING")

    stop()
    time.sleep(0.05)

    backward()
    time.sleep(BACKUP_TIME)

    stop()
    time.sleep(BACKUP_DELAY)


# ============================================================
# MAIN PROGRAM
# ============================================================

print("\n========================================")
print(" THREE-POINT GYRO LINE FOLLOWER")
print("========================================")

print(f"Resolution       : {WIDTH}x{HEIGHT}")
print(f"Threshold        : {THRESHOLD}")
print(f"Line ROI         : {ROI_TOP}:{ROI_BOTTOM}")
print(f"Dead zone        : {DEAD_ZONE}")
print(f"Turn angle       : {TURN_ANGLE} degrees")
print(f"Gyro target      : {GYRO_TURN_TARGET:.1f} degrees")
print(f"Turn timeout     : {MAX_TURN_TIME}s")

print("\nIntersection points:")
print("\n              ●")
print("\n      ●                 ●")

print("\nCamera server:")
print("http://<PI-IP>:8000")

print("\nStarting robot...")
print("Press CTRL+C to stop.\n")

crossing_frames = 0
crossing_locked = False
crossing_cooldown_until = 0.0

try:
    while True:
        frame = picam2.capture_array()

        line_x = find_line(frame)

        if line_x is None:
            print("LINE LOST")
            crossing_frames = 0
            recover_line()
            continue

        black = threshold_image(frame)
        states = read_crossing_points(black)
        intersection = is_intersection(states)

        now = time.monotonic()
        cooldown = now < crossing_cooldown_until

        if intersection and not crossing_locked and not cooldown:
            crossing_frames += 1
        else:
            crossing_frames = 0

        if (
            crossing_frames >= CROSSING_CONFIRM_FRAMES
            and not crossing_locked
            and not cooldown
        ):
            crossing_locked = True
            crossing_frames = 0

            stop()

            print("\n========================================")
            print("       INTERSECTION DETECTED!")
            print("========================================\n")

            print_pattern(states)

            direction = ask_direction()
            execute_direction(direction)

            crossing_cooldown_until = (
                time.monotonic() + CROSSING_COOLDOWN
            )

            continue

        if crossing_locked and not intersection:
            crossing_locked = False

        image_center = WIDTH / 2
        error = line_x - image_center

        if abs(error) <= DEAD_ZONE:
            forward()
            state = "FORWARD"

        elif error < 0:
            line_turn_left()
            state = "LEFT"

        else:
            line_turn_right()
            state = "RIGHT"

        pattern = (
            f"{int(states['FORWARD'])}"
            f"{int(states['LEFT'])}"
            f"{int(states['RIGHT'])}"
        )

        print(
            f"line={line_x:6.1f} "
            f"error={error:7.1f} "
            f"dots={pattern} "
            f"{state}"
        )

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("\nStopping robot...")

finally:
    server_running = False

    try:
        if board is not None:
            stop()
    except Exception as exc:
        print("Motor stop error:", exc)

    try:
        if picam2 is not None:
            picam2.stop()
    except Exception as exc:
        print("Camera stop error:", exc)

    try:
        if board is not None:
            board.shutdown()
    except Exception as exc:
        print("STM32 shutdown error:", exc)

    try:
        if bus is not None:
            bus.close()
    except Exception as exc:
        print("MPU6050 shutdown error:", exc)

    print("Robot stopped.")

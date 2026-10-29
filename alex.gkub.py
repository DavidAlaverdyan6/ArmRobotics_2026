
import time
import threading

import cv2
import numpy as np

from flask import Flask, Response, jsonify, render_template_string
from picamera2 import Picamera2
from telemetrix import telemetrix

from roboarm import RoboArm


HOST = "0.0.0.0"
PORT = 8000
STM32_PORT = "/dev/serial0"

WIDTH = 640
HEIGHT = 480

CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2

FLIP_CAMERA_180 = True

GREEN_LOWER = np.array([40, 100, 80], dtype=np.uint8)
GREEN_UPPER = np.array([85, 255, 255], dtype=np.uint8)

MIN_GREEN_AREA = 1200

BASE_START = 90
SHOULDER_START = 75
ELBOW_START = 90

BASE_MIN = 0
BASE_MAX = 180

SHOULDER_MIN = 0
SHOULDER_MAX = 180

ELBOW_MIN = 0
ELBOW_MAX = 60
ELBOW_STEP = 20

BASE_DIRECTION = -1
SHOULDER_DIRECTION = -1

DEADZONE_X = 25
DEADZONE_Y = 25

KP_BASE = 0.025
KP_SHOULDER = 0.025

MAX_STEP_DEG = 2.0
CONTROL_INTERVAL = 0.12

CENTER_HOLD_SEC = 0.5
ELBOW_SETTLE_SEC = 0.8

app = Flask(__name__)

camera = None
controller = None
arm = None

running = True
tracking_enabled = False

latest_jpeg = None

frame_lock = threading.Lock()
state_lock = threading.RLock()
servo_lock = threading.Lock()

base_angle = float(BASE_START)
shoulder_angle = float(SHOULDER_START)
elbow_angle = float(ELBOW_START)

center_since = None
last_elbow_move = 0.0
last_control = 0.0

state = {
    "found": False,
    "x": None,
    "y": None,
    "error_x": None,
    "error_y": None,
    "centered": False,
    "tracking": False,
    "base": BASE_START,
    "shoulder": SHOULDER_START,
    "elbow": ELBOW_START,
    "message": "Starting"
}


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def update_state(**values):
    with state_lock:
        state.update(values)


def detect_green(frame_rgb):

    frame = cv2.cvtColor(
        frame_rgb,
        cv2.COLOR_RGB2BGR
    )

    if FLIP_CAMERA_180:
        frame = cv2.rotate(
            frame,
            cv2.ROTATE_180
        )

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )

    mask = cv2.inRange(
        hsv,
        GREEN_LOWER,
        GREEN_UPPER
    )

    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel,
        iterations=2
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=2
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    result = {
        "found": False,
        "x": None,
        "y": None,
        "error_x": None,
        "error_y": None,
        "centered": False
    }

    cv2.line(
        frame,
        (CENTER_X, 0),
        (CENTER_X, HEIGHT),
        (255, 255, 0),
        1
    )

    cv2.line(
        frame,
        (0, CENTER_Y),
        (WIDTH, CENTER_Y),
        (255, 255, 0),
        1
    )

    cv2.rectangle(
        frame,
        (CENTER_X - DEADZONE_X, CENTER_Y - DEADZONE_Y),
        (CENTER_X + DEADZONE_X, CENTER_Y + DEADZONE_Y),
        (255, 255, 0),
        2
    )

    if not contours:
        return frame, result

    contour = max(contours, key=cv2.contourArea)

    if cv2.contourArea(contour) < MIN_GREEN_AREA:
        return frame, result

    moments = cv2.moments(contour)

    if moments["m00"] == 0:
        return frame, result

    x = int(moments["m10"] / moments["m00"])
    y = int(moments["m01"] / moments["m00"])

    error_x = x - CENTER_X
    error_y = y - CENTER_Y

    centered = (
        abs(error_x) <= DEADZONE_X
        and abs(error_y) <= DEADZONE_Y
    )

    cv2.drawContours(
        frame,
        [contour],
        -1,
        (0, 255, 0),
        2
    )

    cv2.circle(
        frame,
        (x, y),
        8,
        (0, 0, 255),
        -1
    )

    cv2.line(
        frame,
        (CENTER_X, CENTER_Y),
        (x, y),
        (0, 0, 255),
        2
    )

    cv2.putText(
        frame,
        f"X:{x} Y:{y}",
        (10, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2
    )

    if centered:
        cv2.putText(
            frame,
            "CENTERED",
            (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

    result.update({
        "found": True,
        "x": x,
        "y": y,
        "error_x": error_x,
        "error_y": error_y,
        "centered": centered
    })

    return frame, result


def move_tracking_servos(error_x, error_y):

    global base_angle
    global shoulder_angle

    with servo_lock:

        new_base = base_angle
        new_shoulder = shoulder_angle

        if abs(error_x) > DEADZONE_X:

            step = clamp(
                error_x * KP_BASE,
                -MAX_STEP_DEG,
                MAX_STEP_DEG
            )

            new_base += BASE_DIRECTION * step

        if abs(error_y) > DEADZONE_Y:

            step = clamp(
                error_y * KP_SHOULDER,
                -MAX_STEP_DEG,
                MAX_STEP_DEG
            )

            new_shoulder += SHOULDER_DIRECTION * step

        new_base = clamp(
            new_base,
            BASE_MIN,
            BASE_MAX
        )

        new_shoulder = clamp(
            new_shoulder,
            SHOULDER_MIN,
            SHOULDER_MAX
        )

        base_command = int(round(new_base))
        shoulder_command = int(round(new_shoulder))

        if (
            base_command != int(round(base_angle))
            or shoulder_command != int(round(shoulder_angle))
        ):

            arm.write(
                base=base_command,
                shoulder=shoulder_command
            )

        base_angle = new_base
        shoulder_angle = new_shoulder

    update_state(
        base=round(base_angle, 1),
        shoulder=round(shoulder_angle, 1)
    )


def move_elbow_step():

    global elbow_angle
    global last_elbow_move

    with servo_lock:

        if elbow_angle >= ELBOW_MAX:
            return False

        new_angle = min(
            elbow_angle + ELBOW_STEP,
            ELBOW_MAX
        )

        arm.write(
            elbow=int(round(new_angle))
        )

        elbow_angle = new_angle
        last_elbow_move = time.monotonic()

    update_state(
        elbow=round(elbow_angle, 1),
        message=f"ELBOW moved to {elbow_angle:.0f}"
    )

    return True


def camera_worker():

    global latest_jpeg
    global center_since
    global last_control

    while running:

        try:
            frame_rgb = camera.capture_array()

            frame, detection = detect_green(frame_rgb)

            now = time.monotonic()

            with state_lock:
                state.update(detection)
                enabled = tracking_enabled

            if not enabled:

                center_since = None

                update_state(
                    message="Tracking stopped"
                )

            elif not detection["found"]:

                center_since = None

                update_state(
                    message="Green object lost"
                )

            elif now - last_elbow_move < ELBOW_SETTLE_SEC:

                center_since = None

                update_state(
                    message="Waiting for elbow movement"
                )

            elif detection["centered"]:

                if center_since is None:
                    center_since = now

                if elbow_angle >= ELBOW_MAX:

                    update_state(
                        message="ELBOW limit reached"
                    )

                elif now - center_since >= CENTER_HOLD_SEC:

                    moved = move_elbow_step()

                    if moved:
                        center_since = None

                else:

                    update_state(
                        message="Object centered"
                    )

            else:

                center_since = None

                if now - last_control >= CONTROL_INTERVAL:

                    move_tracking_servos(
                        detection["error_x"],
                        detection["error_y"]
                    )

                    last_control = now

                update_state(
                    message="Centering green object"
                )

            cv2.putText(
                frame,
                f"BASE: {base_angle:.0f}",
                (10, HEIGHT - 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2
            )

            cv2.putText(
                frame,
                f"SHOULDER: {shoulder_angle:.0f}",
                (10, HEIGHT - 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2
            )

            cv2.putText(
                frame,
                f"ELBOW: {elbow_angle:.0f}",
                (10, HEIGHT - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2
            )

            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, 80]
            )

            if ok:
                with frame_lock:
                    latest_jpeg = encoded.tobytes()

            time.sleep(0.02)

        except Exception as error:

            update_state(
                message=f"Camera error: {error}"
            )

            time.sleep(0.2)


HTML = """
<!DOCTYPE html>
<html lang="en">

<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">

<title>Green Object Tracking</title>

<style>
* {
    box-sizing: border-box;
}

body {
    margin: 0;
    padding: 20px;
    background: #10151d;
    color: #e9eff8;
    font-family: Arial, sans-serif;
}

main {
    max-width: 900px;
    margin: auto;
}

h1 {
    color: #62d7a4;
}

.panel {
    background: #1b2430;
    border: 1px solid #354354;
    border-radius: 12px;
    padding: 18px;
    margin-bottom: 16px;
}

.camera {
    width: 100%;
    border-radius: 8px;
}

.grid {
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: 12px;
}

.value {
    color: #62d7a4;
    font-weight: bold;
}

button {
    padding: 13px 20px;
    border: none;
    border-radius: 8px;
    color: white;
    font-size: 16px;
    cursor: pointer;
    margin: 5px;
}

.start {
    background: #238653;
}

.stop {
    background: #bd3535;
}

.status {
    background: #101923;
    padding: 12px;
    border-radius: 8px;
}
</style>
</head>

<body>
<main>

<h1>Green Object Tracking</h1>

<div class="panel">

<h2>Live Camera</h2>

<img class="camera" src="/video_feed">

</div>

<div class="panel">

<h2>Tracking Control</h2>

<button class="start" onclick="startTracking()">
START TRACKING
</button>

<button class="stop" onclick="stopTracking()">
STOP TRACKING
</button>

<p class="status" id="message">Connecting...</p>

</div>

<div class="panel">

<h2>Object Coordinates</h2>

<div class="grid">

<p>X: <span class="value" id="x">--</span></p>
<p>Y: <span class="value" id="y">--</span></p>

<p>Error X: <span class="value" id="errorX">--</span></p>
<p>Error Y: <span class="value" id="errorY">--</span></p>

<p>Centered: <span class="value" id="centered">--</span></p>
<p>Tracking: <span class="value" id="tracking">--</span></p>

</div>

</div>

<div class="panel">

<h2>Servo Angles</h2>

<p>BASE:
<span class="value" id="base">--</span> degrees</p>

<p>SHOULDER:
<span class="value" id="shoulder">--</span> degrees</p>

<p>ELBOW:
<span class="value" id="elbow">--</span> degrees</p>

</div>

</main>

<script>

async function startTracking() {

    const response = await fetch(
        "/tracking/start",
        {method: "POST"}
    );

    const result = await response.json();

    document.getElementById("message").textContent =
        result.message || result.error;
}

async function stopTracking() {

    const response = await fetch(
        "/tracking/stop",
        {method: "POST"}
    );

    const result = await response.json();

    document.getElementById("message").textContent =
        result.message || result.error;
}

function setText(id, value) {

    document.getElementById(id).textContent =
        value === null || value === undefined
        ? "--"
        : value;
}

async function refresh() {

    try {

        const response = await fetch("/status");
        const data = await response.json();

        setText("x", data.x);
        setText("y", data.y);

        setText("errorX", data.error_x);
        setText("errorY", data.error_y);

        setText("base", data.base);
        setText("shoulder", data.shoulder);
        setText("elbow", data.elbow);

        setText("centered", data.centered);
        setText("tracking", data.tracking);

        setText("message", data.message);

    } catch (error) {

        setText("message", "Connection lost");
    }
}

setInterval(refresh, 300);
refresh();

</script>

</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/status")
def get_status():

    with state_lock:
        return jsonify(state.copy())


@app.route("/tracking/start", methods=["POST"])
def start_tracking():

    global tracking_enabled
    global center_since

    if arm is None:
        return jsonify(
            error="Robot not connected"
        ), 503

    with state_lock:
        tracking_enabled = True
        center_since = None
        state["tracking"] = True
        state["message"] = "Tracking started"

    return jsonify(
        ok=True,
        message="Tracking started"
    )


@app.route("/tracking/stop", methods=["POST"])
def stop_tracking():

    global tracking_enabled
    global center_since

    with state_lock:
        tracking_enabled = False
        center_since = None
        state["tracking"] = False
        state["message"] = "Tracking stopped"

    return jsonify(
        ok=True,
        message="Tracking stopped"
    )


@app.route("/video_feed")
def video_feed():

    def generate():

        while running:

            with frame_lock:
                frame = latest_jpeg

            if frame is not None:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + frame
                    + b"\r\n"
                )

            time.sleep(0.05)

    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


def main():

    global camera
    global controller
    global arm
    global running

    try:

        print("Starting camera...")

        camera = Picamera2()

        camera.configure(
            camera.create_video_configuration(
                main={
                    "size": (WIDTH, HEIGHT),
                    "format": "RGB888"
                }
            )
        )

        camera.start()
        time.sleep(2)

        print("Connecting STM32...")

        controller = telemetrix.Telemetrix(
            com_port=STM32_PORT
        )

        arm = RoboArm(controller)

        print("Tracking server ready")

        threading.Thread(
            target=camera_worker,
            daemon=True
        ).start()

        app.run(
            host=HOST,
            port=PORT,
            debug=False,
            threaded=True,
            use_reloader=False
        )

    except KeyboardInterrupt:
        print("Stopping")

    finally:

        running = False

        if camera is not None:
            camera.stop()
            camera.close()

        if controller is not None:
            controller.shutdown()


if __name__ == "__main__":
    main()

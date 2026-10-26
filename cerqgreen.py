
import cv2
import time
import threading
import numpy as np

from flask import Flask, Response, render_template_string, jsonify
from picamera2 import Picamera2


# =============== CONFIGURATION ===============

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8000

# Strict green HSV filter
GREEN_LOWER = np.array([40, 100, 80], dtype=np.uint8)
GREEN_UPPER = np.array([85, 255, 255], dtype=np.uint8)

# Ignore small green areas
MIN_CONTOUR_AREA = 1200

# Rotate camera image 180 degrees
FLIP_CAMERA_180 = True

# =============================================

app = Flask(__name__)

camera = None
frame_lock = threading.Lock()

latest_jpeg = None
latest_info = {
    "found": False,
    "x": None,
    "y": None,
    "area": 0
}

running = True


# =============== CAMERA INITIALIZATION ===============

def initialize_camera():
    global camera

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

    print("Camera started successfully.")


# =============== IMAGE PROCESSING ===============

def process_frame(frame_rgb):

    # Convert camera image to BGR
    frame = cv2.cvtColor(
        frame_rgb,
        cv2.COLOR_RGB2BGR
    )

    # Rotate image 180 degrees
    if FLIP_CAMERA_180:
        frame = cv2.rotate(
            frame,
            cv2.ROTATE_180
        )

    height, width = frame.shape[:2]

    # Convert image to HSV
    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )

    # Detect green pixels
    mask = cv2.inRange(
        hsv,
        GREEN_LOWER,
        GREEN_UPPER
    )

    # Remove noise and small isolated regions
    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel,
        iterations=2
    )

    # Fill small holes inside detected objects
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=2
    )

    # Find external contours
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    info = {
        "found": False,
        "x": None,
        "y": None,
        "area": 0
    }

    # Image center marker
    cv2.drawMarker(
        frame,
        (width // 2, height // 2),
        (255, 255, 0),
        cv2.MARKER_CROSS,
        20,
        2
    )

    # Select largest green region
    if contours:

        largest = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(largest)

        if area >= MIN_CONTOUR_AREA:

            moments = cv2.moments(largest)

            if moments["m00"] != 0:

                cx = int(
                    moments["m10"] / moments["m00"]
                )

                cy = int(
                    moments["m01"] / moments["m00"]
                )

                # Draw object outline
                cv2.drawContours(
                    frame,
                    [largest],
                    -1,
                    (0, 255, 0),
                    3
                )

                # Draw center point
                cv2.circle(
                    frame,
                    (cx, cy),
                    8,
                    (0, 0, 255),
                    -1
                )

                cv2.circle(
                    frame,
                    (cx, cy),
                    14,
                    (255, 255, 255),
                    2
                )

                # Draw coordinate lines
                cv2.line(
                    frame,
                    (cx, 0),
                    (cx, height),
                    (255, 0, 255),
                    1
                )

                cv2.line(
                    frame,
                    (0, cy),
                    (width, cy),
                    (255, 0, 255),
                    1
                )

                # Display coordinates
                cv2.putText(
                    frame,
                    f"X: {cx}  Y: {cy}",
                    (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"Area: {int(area)} px",
                    (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2
                )

                cv2.putText(
                    frame,
                    "GREEN OBJECT: FOUND",
                    (10, height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                info = {
                    "found": True,
                    "x": cx,
                    "y": cy,
                    "area": int(area)
                }

    # No object detected
    if not info["found"]:

        cv2.putText(
            frame,
            "GREEN OBJECT: NOT FOUND",
            (10, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

    # Display image resolution
    cv2.putText(
        frame,
        f"{width} x {height}",
        (width - 160, height - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1
    )

    return frame, info


# =============== CAMERA WORKER ===============

def camera_worker():
    global latest_jpeg, latest_info, running

    while running:

        try:
            frame_rgb = camera.capture_array("main")

            frame, info = process_frame(frame_rgb)

            success, encoded = cv2.imencode(
                ".jpg",
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, 80]
            )

            if success:
                with frame_lock:
                    latest_jpeg = encoded.tobytes()
                    latest_info = info

        except Exception as error:
            print("Camera processing error:", error)
            time.sleep(0.1)


# =============== WEB PAGE ===============

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">

    <meta name="viewport"
          content="width=device-width, initial-scale=1">

    <title>Green Object Tracker</title>

    <style>
        body {
            background: #171717;
            color: white;
            font-family: Arial, sans-serif;
            text-align: center;
            margin: 0;
            padding: 16px;
        }

        h2 {
            color: #55ee77;
        }

        img {
            width: 100%;
            max-width: 800px;
            height: auto;
            border: 2px solid #55ee77;
            border-radius: 8px;
        }

        .info {
            margin: 14px auto;
            padding: 14px;
            max-width: 500px;
            background: #292929;
            border-radius: 8px;
            font-size: 20px;
            line-height: 1.8;
        }

        .note {
            color: #bbbbbb;
            font-size: 14px;
        }
    </style>
</head>

<body>

    <h2>Green Object Tracker</h2>

    <img src="/video_feed" alt="Camera stream">

    <div class="info">
        <div id="status">Connecting...</div>
        <div id="coords">X: -- | Y: --</div>
        <div id="area">Area: --</div>
    </div>

    <p class="note">
        Coordinates are pixels from the top-left corner
        of the rotated image.
    </p>

    <script>
        async function updateInfo() {
            try {
                const response = await fetch('/coordinates');
                const data = await response.json();

                document.getElementById('status').textContent =
                    data.found
                    ? 'GREEN OBJECT FOUND'
                    : 'GREEN OBJECT NOT FOUND';

                document.getElementById('coords').textContent =
                    data.found
                    ? `X: ${data.x} | Y: ${data.y}`
                    : 'X: -- | Y: --';

                document.getElementById('area').textContent =
                    data.found
                    ? `Area: ${data.area} px`
                    : 'Area: --';

            } catch (error) {
                document.getElementById('status').textContent =
                    'Waiting for camera...';
            }
        }

        setInterval(updateInfo, 200);
        updateInfo();
    </script>

</body>
</html>
"""


# =============== WEB ROUTES ===============

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)


@app.route("/coordinates")
def coordinates():
    with frame_lock:
        info = latest_info.copy()

    return jsonify(info)


@app.route("/video_feed")
def video_feed():

    def generate():
        while True:

            with frame_lock:
                frame = latest_jpeg

            if frame is None:
                time.sleep(0.05)
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + frame
                + b"\r\n"
            )

            time.sleep(0.01)

    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


# =============== MAIN ===============

if __name__ == "__main__":

    worker = None

    try:
        initialize_camera()

        worker = threading.Thread(
            target=camera_worker,
            daemon=True
        )

        worker.start()

        print()
        print("Green Object Tracker server started.")
        print("Find Raspberry Pi IP using: hostname -I")
        print(f"Open: http://RASPBERRY_PI_IP:{SERVER_PORT}")
        print("Press Ctrl+C to stop.")

        app.run(
            host=SERVER_HOST,
            port=SERVER_PORT,
            debug=False,
            threaded=True,
            use_reloader=False
        )

    except KeyboardInterrupt:
        print("\nStopping server...")

    finally:
        running = False

        if camera is not None:
            try:
                camera.stop()
                camera.close()
            except Exception:
                pass

        print("Server stopped.")


from picamera2 import Picamera2
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import io
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


WIDTH = 416
HEIGHT = 416

# Distance between the three sampling points
POINT_SPACING = 80

POINTS = [
    (WIDTH // 2 - POINT_SPACING, HEIGHT // 2),
    (WIDTH // 2,                 HEIGHT // 2),
    (WIDTH // 2 + POINT_SPACING, HEIGHT // 2),
]


picam2 = Picamera2()

picam2.configure(
    picam2.create_preview_configuration(
        main={"size": (WIDTH, HEIGHT)}
    )
)

picam2.start()

time.sleep(1)


latest_frame = None
frame_lock = threading.Lock()


def classify_color(r, g, b):

    # Brightness
    brightness = (int(r) + int(g) + int(b)) / 3

    # BLACK
    if brightness < 50:
        return "BLACK", (0, 0, 0)

    # WHITE
    if brightness > 200:
        return "WHITE", (255, 255, 255)

    # YELLOW
    if r > 120 and g > 100 and b < min(r, g) * 0.65:
        return "YELLOW", (255, 255, 0)

    # RED
    if r > g * 1.35 and r > b * 1.35:
        return "RED", (255, 0, 0)

    # GREEN
    if g > r * 1.25 and g > b * 1.15:
        return "GREEN", (0, 255, 0)

    # BLUE
    if b > r * 1.25 and b > g * 1.15:
        return "BLUE", (0, 0, 255)

    # If nothing matches strongly enough
    # classify by dominant channel
    if r >= g and r >= b:
        return "RED", (255, 0, 0)

    if g >= r and g >= b:
        return "GREEN", (0, 255, 0)

    return "BLUE", (0, 0, 255)


def capture_loop():

    global latest_frame

    while True:

        frame = picam2.capture_array()

        # Camera is XBGR8888, so ignore alpha.
        # Adjust this if your colors appear swapped.
        rgb = frame[:, :, :3]

        image = Image.fromarray(rgb, "RGB")

        draw = ImageDraw.Draw(image)

        for x, y in POINTS:

            r, g, b = rgb[y, x]

            label, dot_color = classify_color(
                r, g, b
            )

            radius = 9

            # Draw sampled/classified color
            draw.ellipse(
                (
                    x - radius,
                    y - radius,
                    x + radius,
                    y + radius,
                ),
                fill=dot_color,
                outline="white",
                width=2,
            )

            # Label
            draw.text(
                (
                    x - 25,
                    y + 12
                ),
                label,
                fill="white"
            )

        # JPEG
        buffer = io.BytesIO()

        image.save(
            buffer,
            format="JPEG",
            quality=80
        )

        with frame_lock:
            latest_frame = buffer.getvalue()


class Handler(BaseHTTPRequestHandler):

    def do_GET(self):

        if self.path == "/" or self.path == "/index.html":

            html = """
            <!DOCTYPE html>

            <html>

            <head>
                <title>Pi Camera Color Test</title>
            </head>

            <body>

                <h1>Three Point Color Test</h1>

                <img src="/stream.mjpg">

            </body>

            </html>
            """

            data = html.encode()

            self.send_response(200)

            self.send_header(
                "Content-Type",
                "text/html"
            )

            self.send_header(
                "Content-Length",
                len(data)
            )

            self.end_headers()

            self.wfile.write(data)

        elif self.path == "/stream.mjpg":

            self.send_response(200)

            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame"
            )

            self.send_header(
                "Cache-Control",
                "no-cache"
            )

            self.end_headers()

            try:

                while True:

                    with frame_lock:
                        frame = latest_frame

                    if frame is None:
                        time.sleep(0.01)
                        continue

                    self.wfile.write(
                        b"--frame\r\n"
                    )

                    self.wfile.write(
                        b"Content-Type: image/jpeg\r\n"
                    )

                    self.wfile.write(
                        f"Content-Length: {len(frame)}\r\n\r\n".encode()
                    )

                    self.wfile.write(frame)

                    self.wfile.write(
                        b"\r\n"
                    )

                    time.sleep(0.03)

            except (
                BrokenPipeError,
                ConnectionResetError
            ):
                pass

        else:

            self.send_error(404)


threading.Thread(
    target=capture_loop,
    daemon=True
).start()


server = ThreadingHTTPServer(
    ("0.0.0.0", 8000),
    Handler
)

print("Color classifier running")
print("http://<PI-IP>:8000")

try:

    server.serve_forever()

except KeyboardInterrupt:

    pass

finally:

    picam2.stop()
    server.server_close()


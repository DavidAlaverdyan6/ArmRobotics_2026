from picamera2 import Picamera2
from PIL import Image, ImageDraw
import numpy as np
import io
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

WIDTH = 416
HEIGHT = 416
THRESHOLD = 128

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


def capture_loop():
    global latest_frame

    while True:
        frame = picam2.capture_array()

        # Grayscale
        gray = np.mean(frame[:, :, :3], axis=2)

        # B&W threshold
        binary = np.where(
            gray > THRESHOLD,
            255,
            0
        ).astype(np.uint8)

        image = Image.fromarray(binary, "L")

        # Labels
        draw = ImageDraw.Draw(image)

        draw.text((10, 10), f"THRESHOLD: {THRESHOLD}", fill=255)
        draw.text((10, 30), "WHITE = ABOVE", fill=255)
        draw.text((10, 50), "BLACK = BELOW", fill=0)

        # JPEG
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG", quality=80)

        with frame_lock:
            latest_frame = buffer.getvalue()


class Handler(BaseHTTPRequestHandler):

    def do_GET(self):

        # Browser page
        if self.path == "/" or self.path == "/index.html":

            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>Pi Camera B&W</title>
            </head>
            <body>
                <h1>Pi Camera B&W Threshold</h1>
                <img src="/stream.mjpg">
            </body>
            </html>
            """

            data = html.encode()

            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(data))
            self.end_headers()

            self.wfile.write(data)

        # MJPEG stream
        elif self.path == "/stream.mjpg":

            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame"
            )
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()

            try:
                while True:

                    with frame_lock:
                        frame = latest_frame

                    if frame is None:
                        time.sleep(0.01)
                        continue

                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(
                        b"Content-Type: image/jpeg\r\n"
                    )
                    self.wfile.write(
                        f"Content-Length: {len(frame)}\r\n\r\n".encode()
                    )
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")

                    time.sleep(0.03)

            except (BrokenPipeError, ConnectionResetError):
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

print("B&W camera server running")
print("http://<PI-IP>:8000")

try:
    server.serve_forever()

except KeyboardInterrupt:
    pass

finally:
    picam2.stop()
    server.server_close()
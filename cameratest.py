from picamera2 import Picamera2, Preview
import time

print("Initializing Pi Camera v1.2...")

picam2 = Picamera2()

picam2.configure(
    picam2.create_preview_configuration(
        main={"size": (416, 416)}
    )
)

print("Starting preview...")
picam2.start_preview(Preview.QTGL)

print("Starting camera...")
picam2.start()

print("Warming up...")
time.sleep(1)

print("Camera running. Press Ctrl+C to stop.")

try:
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopping camera...")

finally:
    picam2.stop()
    picam2.stop_preview()
    picam2.close()
    print("Camera stopped.")
import time

import vision
from camera import Camera

Tracker = vision.LineTracker()
PiCam = Camera()

PiCam.start()

print("Vision test running.")

try:
    while True:
        frame = PiCam.get_frame()

        if frame is None:
            time.sleep(0.01)
            continue

        result = Tracker.process(frame)


        PiCam.set_processed_frame(result["display"])


        if result["confirmed_crossing"] is not None:
            print("CROSSING:", result["confirmed_crossing"])
            time.sleep(0.01)


except KeyboardInterrupt:
    pass
finally:
    PiCam.stop()
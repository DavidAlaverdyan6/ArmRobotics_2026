from camera import Camera
from Color_Detector_Daniel import ColorDetector


camera = Camera(width=800, height=600)
detector = ColorDetector(
    min_object_area=700,
    min_brightness=60,
    min_channel_diff=25
)

camera.start()

try:
    while True:
        frame = camera.get_frame()

        if frame is None:
            continue

        objects, processed = detector.process_frame(frame)

        camera.set_processed_frame(processed)

        # Your detected objects are here
        for i, obj in enumerate(objects):
            print(f'Object {i+1}: Color:{obj["color"]}, X:{obj["x"]}, Y:{obj["y"]}, W:{obj["w"]}, H:{obj["h"]}, Center:{obj["center"]}, Area:{obj["area"]}')

finally:
    camera.stop()
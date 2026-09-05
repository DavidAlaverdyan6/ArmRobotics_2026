import time
import numpy as np
from camera import Camera

ContCam = Camera()
ContCam.start()

print("====================\nCONTRAST TEST\n====================")

try:
    #detect = input("What color to detect: ")
    while True:
        frame = ContCam.get_frame()
        if frame is None:
            time.sleep(0.01)
            continue
        else:
            blue = frame[:,:,0]
            green = frame[:,:,1]
            red = frame[:,:,2]

            GreenCondition =  (red < 125) & (green > 150) & (blue < 125)
            GreenPixels = np.argwhere(GreenCondition)
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            LeftUpMost = [np.min(GreenPixels[:,1]), np.min(GreenPixels[:,0])]
            RightDownMost = [np.max(GreenPixels[:,1]), np.max(GreenPixels[:,0])]
            print("LeftUpMost:", LeftUpMost, "RightDownMost:", RightDownMost)
            mask[GreenCondition] = 255

        
        result = mask
        ContCam.set_processed_frame(result)
except KeyboardInterrupt:
    pass
finally:
    ContCam.stop()

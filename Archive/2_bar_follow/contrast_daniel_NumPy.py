import time
import numpy as np
from cv2 import rectangle
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
            result = frame.copy()

            GreenCondition =  (frame[:,:,2] < 125) & (frame[:,:,1] > 150) & (frame[:,:,0] < 125)
            GreenPixels = np.argwhere(GreenCondition)
            LeftUpMost = [np.min(GreenPixels[:,1]), np.min(GreenPixels[:,0])]
            RightDownMost = [np.max(GreenPixels[:,1]), np.max(GreenPixels[:,0])]
            SquarePosition = [RightDownMost[0]-LeftUpMost[0], RightDownMost[1]-LeftUpMost[1]]
            rectangle(result, LeftUpMost, RightDownMost, (27, 200, 27), 3)

            result[GreenCondition] = (0, 255, 0)

        
        ContCam.set_processed_frame(result)
except KeyboardInterrupt:
    pass
finally:
    ContCam.stop()

import cv2
from picamera2 import Picamera2

robocam = Picamera2()
robocam.options["quality"] = 95
robocam.configure(robocam.create_preview_configuration(main={"format": 'RGB888',"size": (640, 480)}))

robocam.start()
cnt = 0
while cnt < 5:
    frame = robocam.capture_array()
    f_name = f'frame_{cnt}.jpg'
    cv2.imwrite(f_name, frame)
    cnt += 1
    print(f"✅ Image saved to {f_name}")

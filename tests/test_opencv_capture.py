import sys
import os
import cv2

# Add src directory to sys.path for module import
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
from vision import OpencvCapture

VIDEO_PATH = 'input_video.mp4'  # Update with your video file path

cap = OpencvCapture(VIDEO_PATH)

for i, frame in enumerate(cap):
    if i >= 5:
        break
    filename = f'opencv_frame_{i+1}.jpg'
    cv2.imwrite(filename, frame)
    print(f'Saved {filename}')

cap.release()
print('Done saving 5 frames from video.')

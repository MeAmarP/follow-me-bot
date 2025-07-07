import sys
import os
import cv2

# Add src directory to sys.path for module import
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
from vision import PicameraCapture

try:
    cap = PicameraCapture()
except ImportError as e:
    print(e)
    sys.exit(1)

for i in range(5):
    frame = cap.read()
    filename = f'picamera_frame_{i+1}.jpg'
    cv2.imwrite(filename, frame)
    print(f'Saved {filename}')

cap.release()
print('Done saving 5 frames from PiCamera.')

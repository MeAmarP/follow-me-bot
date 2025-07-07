import sys
import os
import cv2
import argparse

# Add src directory to sys.path for module import
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
from vision import OpencvCapture

def main():
    parser = argparse.ArgumentParser(description='Save 5 frames from a video file using OpencvCapture.')
    parser.add_argument('--video', type=str, required=True, help='Path to input video file')
    args = parser.parse_args()

    cap = OpencvCapture(args.video)

    for i, frame in enumerate(cap):
        if i >= 5:
            break
        filename = f'opencv_frame_{i+1}.jpg'
        cv2.imwrite(filename, frame)
        print(f'Saved {filename}')

    cap.release()
    print('Done saving 5 frames from video.')

if __name__ == "__main__":
    main()

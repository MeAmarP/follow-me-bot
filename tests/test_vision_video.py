import cv2
import numpy as np
import sys
import os
import argparse
import time

# Add src directory to sys.path for module import
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
from vision import ObjectDetector


def main():
    parser = argparse.ArgumentParser(description="Run object detection on a video and save the output.")
    parser.add_argument('--model', type=str, required=True, help='Path to TFLite model file')
    parser.add_argument('--video', type=str, required=True, help='Path to input video file')
    parser.add_argument('--output', type=str, default='output_detected_video.mp4', help='Path to save output video')
    args = parser.parse_args()

    detector = ObjectDetector(args.model)
    cap = cv2.VideoCapture(args.video)

    if not cap.isOpened():
        print(f"❌ Error: Cannot open video {args.video}")
        return

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(args.output, fourcc, fps, (width, height))

    print("Processing video...")
    frame_count = 0
    avg_infer_time = []
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # Convert BGR to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        start_time = time.time()
        # Detect objects
        detections = detector.detect_objects(rgb_frame)
        end_time = time.time()
        avg_infer_time.append((end_time - start_time) * 1000)
        print(f"Inference latency: {(end_time - start_time)} ms")
        # Draw detections
        for det in detections:
            x1, y1, x2, y2 = det['box']
            label = f"{det['class_name']} {det['confidence']:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        out.write(frame)
        frame_count += 1
        if frame_count % 30 == 0:
            print(f"Processed {frame_count} frames...")

    cap.release()
    out.release()
    print(f"✅ Output saved to {args.output}")
    if avg_infer_time:
        print(f"Average inference time: {np.mean(avg_infer_time):.2f} ms")
    else:
        print("No frames processed.")

if __name__ == "__main__":
    main()

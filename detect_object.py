# detect_object.py
"""
Object detection using MediaPipe and TFLite models.
Usage:
    python detect_object.py --model <model_path> --image <image_path>
"""
import argparse
import cv2
import mediapipe as mp
import numpy as np

# Argument parser for CLI
parser = argparse.ArgumentParser(description="Object detection using MediaPipe and TFLite model.")
parser.add_argument('--model', type=str, required=True, help='Path to TFLite model file')
parser.add_argument('--image', type=str, required=True, help='Path to input image file')
args = parser.parse_args()

# Load image
image = cv2.imread(args.image)
if image is None:
    raise FileNotFoundError(f"Image not found: {args.image}")

# Convert image to RGB
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Load MediaPipe Object Detector
BaseOptions = mp.tasks.BaseOptions
ObjectDetector = mp.tasks.vision.ObjectDetector
ObjectDetectorOptions = mp.tasks.vision.ObjectDetectorOptions
VisionRunningMode = mp.tasks.vision.RunningMode

options = ObjectDetectorOptions(
    base_options=BaseOptions(model_asset_path=args.model),
    score_threshold=0.3,
    max_results=5,
    running_mode=VisionRunningMode.IMAGE
)

def print_detections(detection_result):
    for det in detection_result.detections:
        bbox = det.bounding_box
        print(f"Detected: {det.categories[0].category_name} (score: {det.categories[0].score:.2f}) at [{bbox.origin_x}, {bbox.origin_y}, {bbox.width}, {bbox.height}]")

with ObjectDetector.create_from_options(options) as detector:
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image_rgb)
    detection_result = detector.detect(mp_image)
    print_detections(detection_result)
    # Draw detections on image
    for det in detection_result.detections:
        bbox = det.bounding_box
        x, y, w, h = bbox.origin_x, bbox.origin_y, bbox.width, bbox.height
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        label = det.categories[0].category_name or "Object"
        score = det.categories[0].score
        cv2.putText(image, f"{label} {score:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    # Show result
    cv2.imshow('Detection', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Follow Me Bot

A Raspberry Pi-based robot project featuring object detection and autonomous movement.

## Features
- Object detection
- Real-time camera streaming
- Motor control for forward, reverse, left, and right movement

## Directory Structure
```
follow-me-bot/
├── data/                # Data and sample images
├── models/              # Pretrained TFLite models (e.g., efficientdet_lite0_int8.tflite)
├── src/                 # Source code
│   ├── motor_driver.py  # Motor and driver classes
│   ├── vision.py        # Object detection class using MediaPipe
│   └── config/          # Tracking and config modules
├── tests/               # Test scripts
│   ├── test_camera.py   # Camera stream test
│   ├── test_motor.py    # Motor test
│   ├── test_robo_motion.py # Robot movement test
│   └── test_detect_object.py # Object detection test
├── detect_object.py     # CLI script for object detection
└── README.md            # Project documentation
```

## Setup
1. **Install dependencies:**
   ```bash
   pip install mediapipe opencv-python numpy gpiozero
   ```
   - For Raspberry Pi, ensure you have the correct camera drivers and permissions.

2. **Download TFLite models:**
   Place your `.tflite` models in the `models/` directory.

## Usage

### Camera Stream Test
```bash
python tests/test_camera.py
```

### Motor Test
```bash
python tests/test_motor.py
```

### Robot Motion Test
```bash
python tests/test_robo_motion.py
```

### Object Detection Test
```bash
python tests/test_detect_object.py --model models/efficientdet_lite0_int8.tflite --image data/sample.jpg
```

## Notes
- Update GPIO pin numbers in `motor_driver.py` if your wiring differs.
- The `vision.py` module provides a reusable `ObjectDetector` class for integration in your own scripts.
- For best results, use models compatible with MediaPipe's object detection API.

## License
See [LICENSE](LICENSE) for details.

## References
- https://ai.google.dev/edge/mediapipe/solutions/vision/object_detector/python#video_1
- https://randomnerdtutorials.com/raspberry-pi-pico-dc-motor-micropython/
- https://github.com/google-ai-edge/mediapipe-samples
- https://www.raspberrypi.com/documentation/computers/camera_software.html#rpicam-apps
- https://gpiozero.readthedocs.io/en/stable/recipes.html#
- Video Sample - https://www.freepik.com/free-video/long-path-middle-forest_2867137#fromView=search&page=1&position=43&uuid=3ef1b4a0-d8eb-407f-8def-44461006fca5

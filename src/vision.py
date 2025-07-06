import numpy as np
import mediapipe as mp

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

BaseOptions = python.BaseOptions
MpObjectDetector = vision.ObjectDetector
ObjectDetectorOptions = vision.ObjectDetectorOptions
VisionRunningMode = vision.RunningMode

class ObjectDetector:
    def __init__(self, model_path):
        if not model_path:
            raise ValueError("Model path is required for ObjectDetector")

        options = ObjectDetectorOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            max_results=5,
            running_mode=VisionRunningMode.IMAGE
        )
        self.detector = MpObjectDetector.create_from_options(options)

    def detect_objects(self, image_np):
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image_np)
        detection_result = self.detector.detect(mp_image)

        detections = []
        for detection in detection_result.detections:
            category = detection.categories[0]
            category_name = category.category_name
            class_id = category.index
            confidence = round(category.score, 2)
            bbox = detection.bounding_box
            x, y, w, h = bbox.origin_x, bbox.origin_y, bbox.width, bbox.height

            detections.append({
                "class_id": class_id,
                "class_name": category_name ,
                "confidence": confidence,
                "box": [x, y, x + w, y + h]  # tlbr format
            })

        return detections

    def visualize(
        image,
        detection_result
    ) -> np.ndarray:
        """Draws bounding boxes on the input image and return it.
        Args:
            image: The input RGB image.
            detection_result: The list of all "Detection" entities to be visualize.
        Returns:
            Image with bounding boxes.
        """
        for detection in detection_result.detections:
            # Draw bounding_box
            bbox = detection.bounding_box
            start_point = bbox.origin_x, bbox.origin_y
            end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
            cv2.rectangle(image, start_point, end_point, TEXT_COLOR, 3)

            # Draw label and score
            category = detection.categories[0]
            category_name = category.category_name
            probability = round(category.score, 2)
            result_text = category_name + ' (' + str(probability) + ')'
            text_location = (MARGIN + bbox.origin_x,
                             MARGIN + ROW_SIZE + bbox.origin_y)
            cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                        FONT_SIZE, TEXT_COLOR, FONT_THICKNESS)

        return image
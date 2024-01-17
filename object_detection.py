import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import cv2
from picamera2 import Picamera2
from libcamera import Transform
import os

MODEL_PATH = "/home/mo/PigeonPunisher/efficientdet_lite0-detection-metadata.tflite"

class Detector():
    picam2 = None
    detector = None
    headless = None
    config = None
    x_mid = None
    y_mid = None

    def __init__(self, headless=True) -> None:
        self.headless = headless
        base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
        options = vision.ObjectDetectorOptions(base_options=base_options, score_threshold=0.5, category_allowlist=["person"])
        self.detector = vision.ObjectDetector.create_from_options(options)

        Picamera2.set_logging(Picamera2.WARNING) 
        os.environ["LIBCAMERA_LOG_LEVELS"] = "2" # "0" - DEBUG, "1" - INFO, "2" - WARN, "3" - ERROR, "4" - FATAL
        self.picam2 = Picamera2()
        self.config = self.picam2.create_still_configuration(transform=Transform(hflip=True, vflip=True))
        self.x_mid = int(self.config['main']['size'][0]/2)
        self.y_mid = int(self.config['main']['size'][1]/2)
        self.picam2.configure(self.config)
        self.picam2.start(show_preview=False)
    
    def __del__(self):
        if not self.headless:
            cv2.destroyAllWindows()

    def capture_and_detect(self):
        pil_image = self.picam2.capture_image("main").convert('RGB')
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.asarray(pil_image))
        detection_result = self.detector.detect(image)
        for det in detection_result.detections:
            print(f"{det.categories[0].category_name} detected with {det.categories[0].score} score")
        if not self.headless:
            image_copy = np.copy(image.numpy_view())
            annotated_image = self.visualize(image_copy, detection_result)
            rgb_annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
            cv2.imshow("test", rgb_annotated_image)
            cv2.waitKey(1)
        return detection_result.detections

    def visualize(self, image, detection_result) -> np.ndarray:
        MARGIN = 10  # pixels
        ROW_SIZE = 10  # pixels
        FONT_SIZE = 5
        FONT_THICKNESS = 5
        TEXT_COLOR = (255, 0, 0)  # red
        """Draws bounding boxes on the input image and return it.
        Args:
            image: The input RGB image.
            detection_result: The list of all "Detection" entities to be visualize.
        Returns:
            Image with bounding boxes.
        """
        cv2.circle(image, (self.x_mid, self.y_mid), 30, (0, 255, 0), 10)

        for detection in detection_result.detections:
            # Draw bounding_box
            bbox = detection.bounding_box
            start_point = bbox.origin_x, bbox.origin_y
            end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
            cv2.rectangle(image, start_point, end_point, TEXT_COLOR, 3)
            cv2.circle(image, (bbox.origin_x, bbox.origin_y), 1, TEXT_COLOR, 30)

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
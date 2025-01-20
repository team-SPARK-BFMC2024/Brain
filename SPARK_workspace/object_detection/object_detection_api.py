from ultralytics import YOLO
import cv2
import numpy as np
import time
from copy import deepcopy

class ObjectDetectionAPI():
    def __init__(self, weight_path):
        self.weight_path = weight_path
        self.detection_model = YOLO(self.weight_path)
        self.class_label = self.detection_model.names

    def detect(self, image):
        # Run detection
        results = self.detection_model.predict(image, conf=0.3, iou=0.5)

        # Visualize
        viz_image = deepcopy(image)

        # Process results
        for result in results:
            boxes = result.boxes.cpu().numpy()
            
            for box in boxes:
                # Get box coordinates
                x1, y1, x2, y2 = box.xyxy[0].astype(int)
                
                # Get class id and confidence
                class_id = int(box.cls[0])
                conf = box.conf[0]
                
                # Draw bounding box
                cv2.rectangle(viz_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                
                # Get label size
                (w, h), _ = cv2.getTextSize(self.class_label[class_id], cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                
                # Draw label background
                cv2.rectangle(viz_image, (x1, y1 - 20), (x1 + w, y1), (0, 255, 0), -1)
                
                # Draw label text
                cv2.putText(viz_image, self.class_label[class_id], (x1, y1 - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        return {"result": results, "image": viz_image}


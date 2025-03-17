import torch
from ultralytics import YOLO
import numpy as np

class ObjectDetector:
    def __init__(self, weights='object_detector/weights/best.engine', conf_thres=0.5, iou_thres=0.45):

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        try:
            self.model = YOLO(weights)
            print(f"Model loaded successfully on {self.device}")

            if torch.cuda.is_available():
                print(f"Using GPU: {torch.cuda.get_device_name(0)}")
            else:
                print("CUDA not available, using CPU")
        except Exception as e:
            print(f"Error loading model: {e}")
            raise
        self.model.conf = conf_thres
        self.model.iou = iou_thres

        if not torch.cuda.is_available():
            torch.set_num_threads(4)
            print("Set CPU threads to 4")

    def detect(self, frame):
        try:

            with torch.no_grad():
                results = self.model(frame, verbose=False, device=self.device)
                if len(results) > 0 and len(results[0].boxes) > 0:
                    boxes = results[0].boxes
                    return boxes.data.cpu().numpy()
            return np.array([])
        except Exception as e:
            print(f"Detection error: {e}")
            return np.array([])

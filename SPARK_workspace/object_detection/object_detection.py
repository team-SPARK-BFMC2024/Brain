from object_detection_api import ObjectDetectionAPI
import sys
import cv2
import yaml
sys.path.append("../")
from camera_loader.camera_loader_api import CameraLoaderAPI

if __name__ == "__main__":
    # Load config
    config_file = "../config/camera_config.yaml"
    with open(config_file, "r") as file:
       cam_config = yaml.safe_load(file)        

    # Init Pi camera
    pi_cam = CameraLoaderAPI(cam_config["Camera"])
    pi_cam.start_camera()

    # Init object detection
    object_detection = ObjectDetectionAPI("weights/best.pt")

    # Define the codec and create VideoWriter object
    fps = cam_config['Camera']['fps']
    resolution = (cam_config['Camera']['width'], cam_config['Camera']['height'])
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec for AVI format
    out = cv2.VideoWriter('output.avi', fourcc, fps, resolution)

    try:
        while True:
            frame = pi_cam.read_image()
            result = object_detection.detect(frame)
            viz_image = result["image"]
            out.write(viz_image)

    finally:
        # Cleanup resources
        cv2.destroyAllWindows()
        out.release()
        pi_cam.close_camera()


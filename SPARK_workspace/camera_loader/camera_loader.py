import cv2
import yaml
from camera_loader_api import CameraLoaderAPI

if __name__ == "__main__":
    # Load config
    config_file = "camera_config.yaml"
    with open(config_file, "r") as file:
        config = yaml.safe_load(file)        
    print(config)

    # Init Pi camera
    pi_cam = CameraLoaderAPI(config["Camera"])
    pi_cam.start_camera()

    # Define the codec and create VideoWriter object
    fps = config['Camera']['fps']
    resolution = (config['Camera']['width'], config['Camera']['height'])
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec for AVI format
    out = cv2.VideoWriter('output.avi', fourcc, fps, resolution)

    try:
        while True:
            frame = pi_cam.read_image()
            out.write(frame)

    finally:
        # Cleanup resources
        cv2.destroyAllWindows()
        out.release()
        pi_cam.close_camera()

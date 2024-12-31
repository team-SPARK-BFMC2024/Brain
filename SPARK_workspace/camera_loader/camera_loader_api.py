from picamera2 import Picamera2
import time
import cv2

class CameraLoaderAPI():
    def __init__(self, config):
        # Initialize the Picamera2 object
        self.__pi_cam = Picamera2()

        # Set the resolution and FPS (frame rate)
        self.__config = config
        self.__fps = self.__config['fps']
        self.__resolution = (self.__config['width'], self.__config['height'])

        # Configure the camera for preview
        self.__cam_config = self.__pi_cam.create_preview_configuration(main={"size": self.__resolution})
        self.__pi_cam.configure(self.__cam_config)

    def start_camera(self):
        self.__pi_cam.start()

    def close_camera(self):
        self.__pi_cam.close()

    def read_image(self):
        # Capture a frame
        frame = self.__pi_cam.capture_array()
        # Convert the frame from RGBA to BGR
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

        return frame_bgr
    

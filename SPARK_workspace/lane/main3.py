import configparser
import logging
import time
import cv2
import serial
import torch
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform
from threading import Thread, Lock
from queue import Queue
from ultralytics import YOLO
from src.LaneKeeping.lanekeeping import LaneKeeping
from src.LaneDetection.detect import LaneDetection

class ObjectDetector:
    def __init__(self, weights='best.pt', conf_thres=0.35, iou_thres=0.45):
        self.device = torch.device('cpu')
        try:
            self.model = YOLO(weights)
            print("Model loaded successfully")
        except Exception as e:
            print(f"Error loading model: {e}")
            raise
        self.model.conf = conf_thres
        self.model.iou = iou_thres
        if not torch.cuda.is_available():
            torch.set_num_threads(4)
            
    def detect(self, frame):
        try:
            with torch.no_grad():
                results = self.model(frame, verbose=False)
                if len(results) > 0:
                    boxes = results[0].boxes
                    return boxes.data.cpu().numpy()
            return np.array([])
        except Exception as e:
            print(f"Detection error: {e}")
            return np.array([])

class VehicleState:
    def __init__(self):
        self.zone_type = 'city'
        self.in_roundabout = False
        self.parking_mode = False
        self.stop_timer = 0
        self.last_state = None
        self.obstacle_vehicles = []
        self.detected_pedestrians = []
        self.current_lane_type = 'dashed'
        self.current_speed = 0
        self.target_speed = 0
        self.steering_angle = 0
        self.last_detection_time = time.time()

class CarController:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        time.sleep(2)
        self.state = VehicleState()
        
        # Initialize speed parameters
        self.min_speed = 50  # Default city minimum
        self.max_speed = 100  # Default city maximum
        self.base_speed = 80 # Default city base speed
        self.state.target_speed = self.base_speed
        
    def set_zone(self, zone_type='city'):
        """Update speed parameters based on zone type."""
        if zone_type == 'highway':
            self.min_speed = 40
            self.max_speed = 50
            self.base_speed = 45
        else:  # city
            self.min_speed = 20
            self.max_speed = 30
            self.base_speed = 25
        self.state.target_speed = self.base_speed
        self.state.zone_type = zone_type
            
    def send_command(self, command: str):
        self.serial.write(command.encode('ascii'))
        response = self.serial.readline().decode().strip()
        return response if response else None

    def set_power_state(self, state: int):
        command = f"#kl:{state};;\r\n"
        return self.send_command(command)

    def set_speed(self, speed: int):
        # Handle None or invalid speed values
        if speed is None:
            speed = self.base_speed
        try:
            speed = int(speed)
            speed = max(min(speed, self.max_speed), self.min_speed)
            self.state.current_speed = speed
            command = f"#speed:{speed};;\r\n"
            return self.send_command(command)
        except (TypeError, ValueError):
            print(f"Invalid speed value: {speed}, using base speed")
            return self.set_speed(self.base_speed)

    def set_steering(self, angle: int):
        self.state.steering_angle = angle
        steering_command = int(angle * 20)
        steering_command = max(min(steering_command, 230), -230)
        command = f"#steer:{steering_command};;\r\n"
        return self.send_command(command)

    def brake(self):
        self.state.current_speed = 0
        command = "#brake:0;;\r\n"
        return self.send_command(command)

    def close(self):
        self.brake()
        time.sleep(0.5)
        self.set_power_state(0)
        self.serial.close()

class LaneAnalyzer:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.bus_lane_region = None
        self.last_lane_type = 'dashed'
        
    def detect_lane_type(self, frame, lane_detection_results):
        # Implement lane type detection based on your lane detection results
        # This is a placeholder - implement based on your specific setup
        return self.last_lane_type
        
    def detect_bus_lane(self, frame):
        # Implement bus lane detection
        # This is a placeholder - implement based on your specific setup
        return False
        
    def is_safe_to_overtake(self, obstacle_vehicles, lane_type):
        if lane_type == 'continuous':
            return False
        
        # Check if there's enough space to overtake
        # This is a placeholder - implement based on your specific setup
        return len(obstacle_vehicles) == 0

class TrafficRuleProcessor:
    def __init__(self):
        self.CLASS_NAMES = [
            'car', 'closed-road-stand', 'crosswalk-sign', 'highway-entry-sign',
            'highway-exit-sign', 'no-entry-road-sign', 'one-way-road-sign',
            'parking-sign', 'parking-spot', 'pedestrian', 'priority-sign',
            'round-about-sign', 'stop-line', 'stop-sign', 'traffic-light',
            'traffic-green', 'traffic-red', 'traffic-yellow'
        ]
        
        # Default speeds for different scenarios (in cm/s)
        self.SPEEDS = {
            'highway': {
                'normal': 50,
                'caution': 45,
                'slow': 40
            },
            'city': {
                'normal': 30,
                'caution': 25,
                'slow': 20
            }
        }

    def get_default_speed(self, vehicle_state, speed_type='normal'):
        """Get default speed based on zone and speed type."""
        zone = vehicle_state.zone_type
        return self.SPEEDS[zone][speed_type]

    def process_traffic_signal(self, signal_class, vehicle_state):
        """Process traffic light signals."""
        try:
            if signal_class == self.CLASS_NAMES.index('traffic-red'):
                return 'stop', 0
            elif signal_class == self.CLASS_NAMES.index('traffic-yellow'):
                return 'caution', self.get_default_speed(vehicle_state, 'caution')
            elif signal_class == self.CLASS_NAMES.index('traffic-green'):
                return 'proceed', self.get_default_speed(vehicle_state, 'normal')
            return None, self.get_default_speed(vehicle_state, 'normal')
        except ValueError as e:
            print(f"Error processing traffic signal: {e}")
            return None, self.get_default_speed(vehicle_state, 'normal')

    def process_traffic_sign(self, sign_class, vehicle_state):
        """Process traffic signs with appropriate speeds and behaviors."""
        try:
            current_time = time.time()
            
            if sign_class == self.CLASS_NAMES.index('stop-sign'):
                if vehicle_state.last_state != 'stop':
                    vehicle_state.stop_timer = current_time
                    vehicle_state.last_state = 'stop'
                    return 'stop', 0
                elif current_time - vehicle_state.stop_timer >= 3:
                    vehicle_state.last_state = None
                    return 'proceed', self.get_default_speed(vehicle_state, 'normal')
                return 'stop', 0
                
            elif sign_class == self.CLASS_NAMES.index('parking-sign'):
                vehicle_state.parking_mode = True
                return 'parking', self.get_default_speed(vehicle_state, 'slow')
                
            elif sign_class == self.CLASS_NAMES.index('crosswalk-sign'):
                return 'caution', self.get_default_speed(vehicle_state, 'caution')
                
            elif sign_class == self.CLASS_NAMES.index('highway-entry-sign'):
                vehicle_state.zone_type = 'highway'
                return 'proceed', self.SPEEDS['highway']['normal']
                
            elif sign_class == self.CLASS_NAMES.index('highway-exit-sign'):
                vehicle_state.zone_type = 'city'
                return 'proceed', self.SPEEDS['city']['normal']
                
            elif sign_class == self.CLASS_NAMES.index('round-about-sign'):
                vehicle_state.in_roundabout = True
                return 'caution', self.get_default_speed(vehicle_state, 'caution')
                
            elif sign_class == self.CLASS_NAMES.index('priority-sign'):
                return 'proceed', self.get_default_speed(vehicle_state, 'normal')
                
            elif sign_class == self.CLASS_NAMES.index('no-entry-road-sign'):
                return 'stop', 0
                
            elif sign_class == self.CLASS_NAMES.index('one-way-road-sign'):
                return 'proceed', self.get_default_speed(vehicle_state, 'normal')
                
            return None, self.get_default_speed(vehicle_state, 'normal')
            
        except ValueError as e:
            print(f"Error processing traffic sign: {e}")
            return None, self.get_default_speed(vehicle_state, 'normal')

    def process_pedestrian(self, detection, frame_height):
        """Process pedestrian detections with distance-based behavior."""
        try:
            x1, y1, x2, y2 = detection[:4]
            
            # Calculate relative distance based on vertical position
            relative_pos = y2 / frame_height
            
            # Pedestrian is very close to vehicle
            if relative_pos > 0.8:
                return 'stop', 0
            # Pedestrian is in warning zone
            elif relative_pos > 0.6:
                return 'caution', 20  # Slow speed
            # Pedestrian is detected but at a safe distance
            else:
                return 'caution', 25  # Moderate speed
                
        except Exception as e:
            print(f"Error processing pedestrian detection: {e}")
            return 'caution', 20  # Default to cautious behavior

    def process_vehicle(self, detection, lane_type, frame_width, frame_height):
        """Process vehicle detections with lane-based behavior."""
        try:
            x1, y1, x2, y2 = detection[:4]
            
            # Calculate relative position and size
            vehicle_center_x = (x1 + x2) / 2
            vehicle_width = x2 - x1
            vehicle_height = y2 - y1
            relative_distance = 1 - (y2 / frame_height)
            
            # Distance-based speed adjustment
            base_following_speed = 30
            if relative_distance < 0.2:  # Very close
                following_speed = base_following_speed * 0.6
            elif relative_distance < 0.4:  # Moderately close
                following_speed = base_following_speed * 0.8
            else:  # Safe distance
                following_speed = base_following_speed
            
            # Determine if vehicle is in our lane
            in_our_lane = (vehicle_center_x > frame_width * 0.4 and 
                          vehicle_center_x < frame_width * 0.6)
            
            if in_our_lane:
                if lane_type == 'continuous':
                    return 'follow', int(following_speed)
                else:
                    # Only overtake if the vehicle is significantly slower
                    if vehicle_height > frame_height * 0.4:  # Close vehicle
                        return 'overtake', 40
                    else:
                        return 'follow', int(following_speed)
                    
            return None, None
            
        except Exception as e:
            print(f"Error processing vehicle detection: {e}")
            return None, None

    def is_safe_following_distance(self, vehicle_height, frame_height):
        """Determine if following distance is safe based on vehicle size in frame."""
        relative_size = vehicle_height / frame_height
        return relative_size < 0.4  # Threshold for safe following distance

    def calculate_following_speed(self, relative_distance, base_speed):
        """Calculate appropriate following speed based on relative distance."""
        if relative_distance < 0.2:
            return base_speed * 0.6
        elif relative_distance < 0.4:
            return base_speed * 0.8
        return base_speed

class AutonomousController:
    def __init__(self, camera_width, camera_height):
        self.car = CarController()
        self.detector = ObjectDetector()
        self.lane_analyzer = LaneAnalyzer(camera_width, camera_height)
        self.traffic_processor = TrafficRuleProcessor()
        self.frame_width = camera_width
        self.frame_height = camera_height
        
    def process_detection(self, detection, frame):
        """Process each detected object and determine appropriate action."""
        try:
            x1, y1, x2, y2, conf, cls = detection
            state = self.car.state
            cls = int(cls)  # Ensure class index is integer
            
            # Process different types of detections
            if cls in [self.traffic_processor.CLASS_NAMES.index(x) for x in 
                      ['traffic-red', 'traffic-yellow', 'traffic-green']]:
                action, speed = self.traffic_processor.process_traffic_signal(cls, state)
                
            elif cls in [self.traffic_processor.CLASS_NAMES.index(x) for x in 
                        ['stop-sign', 'parking-sign', 'crosswalk-sign', 
                         'highway-entry-sign', 'highway-exit-sign', 'round-about-sign',
                         'priority-sign', 'no-entry-road-sign', 'one-way-road-sign']]:
                action, speed = self.traffic_processor.process_traffic_sign(cls, state)
                
            elif cls == self.traffic_processor.CLASS_NAMES.index('pedestrian'):
                action, speed = self.traffic_processor.process_pedestrian(
                    detection, self.frame_height)
                
            elif cls == self.traffic_processor.CLASS_NAMES.index('car'):
                action, speed = self.traffic_processor.process_vehicle(
                    detection, state.current_lane_type, self.frame_width, self.frame_height)
                
            else:
                return None, None
                
            # Log the detection and action
            print(f"Detected {self.traffic_processor.CLASS_NAMES[cls]}: {action}, speed={speed}")
            return action, speed
            
        except Exception as e:
            print(f"Error processing detection: {e}")
            return None, None
        
    def update_vehicle_state(self, action, speed):
        """Update vehicle state based on determined action and speed."""
        try:
            if action == 'stop':
                self.car.brake()
            elif action in ['slow', 'caution']:
                if speed is not None:
                    self.car.set_speed(int(speed))
                else:
                    self.car.set_speed(int(self.car.base_speed * 0.7))
            elif action == 'proceed':
                if speed is not None:
                    self.car.set_speed(int(speed))
                else:
                    self.car.set_speed(self.car.base_speed)
            elif action == 'parking':
                if speed is not None:
                    self.car.set_speed(int(speed))
                else:
                    self.car.set_speed(int(self.car.base_speed * 0.5))
            elif action == 'follow':
                if speed is not None:
                    self.car.set_speed(int(speed))
                else:
                    self.car.set_speed(int(self.car.base_speed * 0.7))
            elif action == 'overtake':
                if speed is not None:
                    self.car.set_speed(int(speed))
                else:
                    self.car.set_speed(self.car.base_speed)
                    
        except Exception as e:
            print(f"Error updating vehicle state: {e}")
            # Fallback to safe behavior
            self.car.set_speed(self.car.base_speed)

def run_autonomous_system(picam2, lk, ld):
    """Run the autonomous system with continuous operation."""
    
    controller = AutonomousController(picam2.camera_config["main"]["size"][0],
                                    picam2.camera_config["main"]["size"][1])
    
    # Constants
    STOP_TIMEOUT = 2.0  # seconds
    DEFAULT_SPEED = controller.car.base_speed
    last_stop_time = 0
    
    try:
        # Initial setup
        controller.car.set_power_state(30)
        time.sleep(1)
        controller.car.set_speed(DEFAULT_SPEED)
        print("Starting autonomous system...")
        
        while True:
            frame = picam2.capture_array()
            current_time = time.time()
            
            # Lane detection
            lane_results = ld.lanes_detection(frame)
            angle, processed_frame = lk.lane_keeping(lane_results)
            
            # Update lane type if lanes are detected
            if lane_results is not None:
                controller.car.state.current_lane_type = controller.lane_analyzer.detect_lane_type(
                    frame, lane_results)
            
            # Detect objects
            detections = controller.detector.detect(frame)
            
            # Process each detection
            highest_priority_action = None
            highest_priority_speed = None
            stop_condition_detected = False
            
            # Process detections if any
            if len(detections) > 0:
                for det in detections:
                    action, speed = controller.process_detection(det, frame)
                    
                    if action == 'stop':
                        stop_condition_detected = True
                        last_stop_time = current_time
                        controller.car.brake()
                        break
                    elif action == 'caution' and not stop_condition_detected:
                        controller.car.set_speed(speed if speed is not None else int(DEFAULT_SPEED * 0.7))
                    elif action == 'proceed' and not highest_priority_action:
                        controller.car.set_speed(speed if speed is not None else DEFAULT_SPEED)
            
            # If no stop condition is detected and we've waited long enough after a stop
            if not stop_condition_detected:
                if current_time - last_stop_time > STOP_TIMEOUT:
                    # Resume normal speed if we're stopped
                    if controller.car.state.current_speed == 0:
                        print("Resuming normal operation")
                        controller.car.set_speed(DEFAULT_SPEED)
            
            # Apply steering
            if angle is not None:
                controller.car.set_steering(angle)
            else:
                # If no lanes detected, keep going straight
                controller.car.set_steering(0)
                if not stop_condition_detected and current_time - last_stop_time > STOP_TIMEOUT:
                    controller.car.set_speed(DEFAULT_SPEED)
            
            # Display frame
            cv2.imshow('Autonomous System', processed_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Stopping...")
                break
                
    finally:
        controller.car.close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    log = logging.getLogger('Root logger')
    
    config = configparser.ConfigParser()
    config.read("config.ini")
    
    picam2 = Picamera2()
    preview_config = picam2.create_preview_configuration(
        main={"format": 'RGB888', "size": (640, 480)},
        transform=Transform(hflip=False, vflip=False)
    )
    picam2.configure(preview_config)
    picam2.start()
    
    try:
        frame = picam2.capture_array()
        
        # Initialize lane detection
        lk = LaneKeeping(frame.shape[1], frame.shape[0], log, "455")
        ld = LaneDetection(frame.shape[1], frame.shape[0], "455", lk)
        
        # Set lane detection parameters
        ld.square_pulses_min_height = 80
        ld.square_pulses_pix_dif = 10
        ld.square_pulses_min_height_dif = 20
        ld.square_pulses_allowed_peaks_width_error = 15
        
        run_autonomous_system(picam2, lk, ld)
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

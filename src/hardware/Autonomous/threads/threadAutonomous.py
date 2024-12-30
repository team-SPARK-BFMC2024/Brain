from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (mainCamera, SteerMotor, SpeedMotor, LaneKeeping,
                                          Location)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
import cv2
import numpy as np
import onnxruntime as ort
import base64
import time
from enum import Enum

class DrivingState(Enum):
    NORMAL = "normal"
    PARKING = "parking"
    ROUNDABOUT = "roundabout"
    HIGHWAY = "highway"
    CITY = "city"
    INTERSECTION = "intersection"
    STOPPING = "stopping"

# Add at class level:
class threadAutonomous(ThreadWithStop):
    # Constants
    LANE_WIDTH = 35  # cm
    CONFIDENCE_THRESHOLD = 0.5
    OBSTACLE_SIZE_THRESHOLD = 0.1
    SIGN_SIZE_THRESHOLD = 0.05
    STEERING_SMOOTH_FACTOR = 0.3
    SPEED_SMOOTH_FACTOR = 0.2
    ROUNDABOUT_EXIT_TIME = 5.0
    STOP_WAIT_TIME = 3.0

class threadAutonomous(ThreadWithStop):
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        
        # Initialize state
        self.driving_state = DrivingState.CITY
        self.stop_start_time = None
        self.parking_spot_found = False
        self.roundabout_entry_time = None
        
        # Speed limits (in cm/s)
        self.speed_limits = {
            DrivingState.HIGHWAY: {"min": 40, "max": 60},
            DrivingState.CITY: {"min": 20, "max": 40}
        }
        
        # Subscribe to messages
        self.subscribe()
        self.initialize_model()
        
        # Detection classes
        self.classes = ['car', 'closed-road-stand', 'crosswalk-sign', 
                       'highway-entry-sign', 'highway-exit-sign', 
                       'no-entry-road-sign', 'one-way-road-sign', 
                       'parking-sign', 'parking-spot', 'pedestrian', 
                       'priority-sign', 'round-about-sign', 'stop-line',
                       'stop-sign', 'traffic-light', 'traffic-green', 
                       'traffic-red', 'traffic-yellow']
        
        super(threadAutonomous, self).__init__()

    def initialize_model(self):
        """Initialize YOLOv8 ONNX model"""
        try:
            # Create ONNX Runtime session
            self.session = ort.InferenceSession("best.onnx", 
                                            providers=['CPUExecutionProvider'])
            
            # Get model metadata
            self.input_name = self.session.get_inputs()[0].name
            self.input_shape = self.session.get_inputs()[0].shape
            self.output_names = [o.name for o in self.session.get_outputs()]
            
            self.logging.info("ONNX model initialized successfully")
        except Exception as e:
            self.logging.error(f"Failed to initialize model: {str(e)}")
            raise

    def subscribe(self):
        """Subscribe to required messages"""
        self.camera_subscriber = messageHandlerSubscriber(
            self.queuesList, mainCamera, "lastOnly", True)
        self.location_subscriber = messageHandlerSubscriber(
            self.queuesList, Location, "lastOnly", True)
        
        # Control senders
        self.steer_sender = messageHandlerSender(self.queuesList, SteerMotor)
        self.speed_sender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.lane_offset_sender = messageHandlerSender(self.queuesList, LaneKeeping)

    def detect_parking_spot(self, img):
        """Detect empty parking spots"""
        # Implementation for parking spot detection
        # Returns True if empty spot found, False otherwise
        pass

    def handle_pedestrian(self, bbox):
        """Handle pedestrian detection based on position"""
        x, y, w, h = bbox
        
        # Check if pedestrian is near crosswalk
        # This would need to be integrated with crosswalk detection
        is_at_crosswalk = False  # Implement crosswalk detection
        
        if is_at_crosswalk or (y > 0.4):  # Pedestrian is on road
            return 0.0  # Stop
        return None  # No specific action needed

    def handle_traffic_light(self, detections):
        """Process traffic light detections"""
        for class_id, confidence, bbox in detections:
            if class_id >= len(self.classes):
                continue
                
            class_name = self.classes[class_id]
            if class_name == 'traffic-red':
                return 0.0  # Stop
            elif class_name == 'traffic-yellow':
                return 0.2  # Proceed with caution
            elif class_name == 'traffic-green':
                return None  # Proceed normally
        
        return None

    def handle_traffic_sign(self, class_name, bbox):
        """Handle different traffic signs"""
        x, y, w, h = bbox
        sign_size = w * h
        
        if sign_size < 0.05:  # Sign is too far
            return None, None
        
        if class_name == 'stop-sign':
            if self.stop_start_time is None:
                self.stop_start_time = time.time()
                return 0.0, 0.0  # Stop
            elif time.time() - self.stop_start_time < 3.0:
                return 0.0, 0.0  # Keep stopping for 3 seconds
            else:
                self.stop_start_time = None
                
        elif class_name == 'parking-sign':
            self.driving_state = DrivingState.PARKING
            return None, 0.2  # Slow down to look for spot
            
        elif class_name == 'crosswalk-sign':
            return None, 0.2  # Slow down
            
        elif class_name == 'priority-sign':
            return None, None  # Proceed normally
            
        elif class_name == 'highway-entry-sign':
            self.driving_state = DrivingState.HIGHWAY
            return None, 0.5  # Accelerate to highway speed
            
        elif class_name == 'highway-exit-sign':
            self.driving_state = DrivingState.CITY
            return None, 0.3  # Decelerate to city speed
            
        elif class_name == 'one-way-road-sign':
            # Adjust steering to follow one-way direction
            return 0.0, None
            
        elif class_name == 'round-about-sign':
            self.driving_state = DrivingState.ROUNDABOUT
            if self.roundabout_entry_time is None:
                self.roundabout_entry_time = time.time()
            return 0.3, 0.2  # Initial roundabout entry
            
        elif class_name == 'no-entry-road-sign':
            return 1.0, 0.0  # Turn away from no-entry road
            
        return None, None

    # Add these methods:
    def _check_roundabout_exit(self):
        """Check if we should exit the roundabout"""
        # Implement logic to check for roundabout exit conditions
        return False

    def _send_commands(self, steering, speed, lane_offset):
        """Send control commands to actuators"""
        self.steer_sender.send(str(steering))
        self.speed_sender.send(str(speed))
        self.lane_offset_sender.send(int(lane_offset * 100))

    def _debug_output(self, steering, speed, lane_offset, detections):
        """Output debug information"""
        self.logging.info(f"Steering: {steering:.2f}, Speed: {speed:.2f}, Lane offset: {lane_offset:.2f}")
        self.logging.info(f"Detected objects: {[(self.classes[d[0]], d[1]) for d in detections]}")

    def detect_objects(self, img):
        """Run object detection using ONNX model
        
        Args:
            img: Input image (BGR format)
            
        Returns:
            List of detections (class_id, confidence, bbox)
        """
        try:
            # Preprocess image
            input_img = cv2.resize(img, (640, 640))
            input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
            input_img = input_img.transpose(2, 0, 1)  # HWC to CHW
            input_img = input_img.astype(np.float32)
            input_img /= 255.0
            input_img = np.expand_dims(input_img, 0)  # Add batch dimension
            
            # Run inference
            outputs = self.session.run(self.output_names, 
                                    {self.input_name: input_img})
            
            # Process detections (assuming YOLOv8 output format)
            predictions = outputs[0]  # Shape: (1, num_boxes, 85)
            predictions = predictions[0]  # Remove batch dimension
            
            # Filter detections
            detections = []
            for pred in predictions:
                confidence = pred[4]
                if confidence > self.CONFIDENCE_THRESHOLD:
                    class_scores = pred[5:]
                    class_id = np.argmax(class_scores)
                    bbox = pred[0:4]  # x, y, w, h
                    detections.append((int(class_id), float(confidence), bbox))
            
            return detections
            
        except Exception as e:
            self.logging.error(f"Error in object detection: {str(e)}")
            return []
    
    def process_image(self, img):
        """Process image for lane detection
        
        Args:
            img: Input image (BGR format)
            
        Returns:
            edges: Processed edge image
        """
        # Resize image to standard size
        src_img = cv2.resize(img, (720, 480))
        # Convert to grayscale
        gray_img = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)
        # Apply blur
        blur_img = cv2.blur(gray_img, (5, 5), cv2.BORDER_DEFAULT)
        # Edge detection
        edges = cv2.Canny(blur_img, 190, 230, None, 3)
        return edges, src_img

    def middle_lane_point(self, lines):
        """Calculate middle point between lane lines
        
        Args:
            lines: Detected lines from HoughLinesP
            
        Returns:
            tuple: (x, y) coordinates of middle point
        """
        y_const = 350
        x_const = 360
        x_right_list = []
        x_left_list = []

        x_right = 720
        x_left = 0

        for line in lines:
            x1, y1, x2, y2 = line[0]
            x_check = (x1 + x2) / 2
            y_check = (y1 + y2) / 2
            check_x = x_check - x_const
            
            if max([y1, y2]) > 300:
                if check_x > 0 and x_check < x_right:
                    _x = int((x_check + x_right) / 2)
                    x_right_list.append(_x)
                    x_right = np.average(x_right_list)
                elif check_x < 0 and x_check > x_left:
                    _x = int((x_check + x_left) / 2)
                    x_left_list.append(_x)
                    x_left = np.average(x_left_list)

        if len(x_left_list) == 0:
            x_left = -200
        if len(x_right_list) == 0:
            x_right = 920

        x = int((x_right + x_left) / 2)
        return (x, y_const)

    def lane_tracking(self, edges):
        """Track lanes using Hough transform
        
        Args:
            edges: Edge detected image
            
        Returns:
            tuple: (lines_list, (x,y)) where x,y is the middle point
        """
        lines_list = []
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi/180,
            threshold=30,
            minLineLength=10,
            maxLineGap=4
        )

        if lines is None:
            return [], (360, 350)  # Return center point if no lines detected

        for points in lines:
            x1, y1, x2, y2 = points[0]
            lines_list.append([x1, y1, x2, y2])

        middle_point = self.middle_lane_point(lines)
        return lines_list, middle_point

    def detect_lanes(self, img):
        try:
            edges, src_img = self.process_image(img)
            lines, (middle_x, middle_y) = self.lane_tracking(edges)
            
            if not lines:
                if hasattr(self, '_last_offset'):
                    return self._last_offset  # Return last known good offset
                return 0.0
            
            image_center = 360
            offset = middle_x - image_center
            normalized_offset = offset / (720/2)
            normalized_offset = max(-1.0, min(1.0, normalized_offset))
            
            # Store last known good offset
            self._last_offset = normalized_offset
            
            if self.debugging:
                self._draw_debug_visualization(src_img, lines, middle_x, middle_y)
            
            return normalized_offset
                
        except Exception as e:
            self.logging.error(f"Error in lane detection: {str(e)}")
            return getattr(self, '_last_offset', 0.0)

    def _draw_debug_visualization(self, img, lines, middle_x, middle_y):
        """Draw debug visualization for lane detection"""
        debug_img = img.copy()
        for line in lines:
            x1, y1, x2, y2 = line
            cv2.line(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(debug_img, (middle_x, middle_y), radius=5, color=(0, 0, 255), thickness=-1)
        cv2.circle(debug_img, (360, 350), radius=5, color=(255, 0, 0), thickness=-1)
        cv2.imshow("Lane Detection Debug", debug_img)
        cv2.waitKey(1)

    def detect_line_type(self, img):
        """Detect if lane line is dashed or continuous
        
        Args:
            img: Input image (BGR format)
            
        Returns:
            str: "dashed" or "continuous"
        """
        try:
            edges, _ = self.process_image(img)
            lines, _ = self.lane_tracking(edges)
            
            if not lines:
                return "continuous"  # Default to continuous if no lines detected
                
            # Calculate average line length
            avg_length = np.mean([
                np.sqrt((x2-x1)**2 + (y2-y1)**2) 
                for x1, y1, x2, y2 in lines
            ])
            
            # Threshold for distinguishing between dashed and continuous
            # Adjust these thresholds based on your specific case
            if avg_length < 50:  # Short lines indicate dashed
                return "dashed"
            return "continuous"
            
        except Exception as e:
            self.logging.error(f"Error in line type detection: {str(e)}")
            return "continuous"

    def plan_path(self, detections, lane_offset, img):
        """Plan path based on all rules and detections"""
        try:
            # Add state validation
            if self.driving_state not in self.speed_limits:
                self.driving_state = DrivingState.CITY
                self.logging.warning("Invalid driving state, resetting to CITY")

            # Add detection validation
            valid_detections = [
                d for d in detections 
                if d[0] < len(self.classes)
            ]

            # Default values based on current state
            steering = -lane_offset  # Basic lane centering
            speed = self.speed_limits[self.driving_state]["min"] / 100.0  # Convert to normalized speed
            
            # Process traffic lights first
            light_speed = self.handle_traffic_light(detections)
            if light_speed is not None:
                speed = light_speed
            
            # Check for pedestrians
            for class_id, confidence, bbox in detections:
                if self.classes[class_id] == 'pedestrian':
                    ped_speed = self.handle_pedestrian(bbox)
                    if ped_speed is not None:
                        speed = ped_speed
            
            # Process other detections
            for class_id, confidence, bbox in detections:
                if class_id >= len(self.classes):
                    continue
                    
                class_name = self.classes[class_id]
                sign_steer, sign_speed = self.handle_traffic_sign(class_name, bbox)
                
                if sign_steer is not None:
                    steering = sign_steer
                if sign_speed is not None:
                    speed = sign_speed
                
                # Handle obstacles (other cars)
                if class_name == 'car':
                    x, y, w, h = bbox
                    if w * h > 0.1:  # Car is close
                        line_type = self.detect_line_type(img)
                        if line_type == "dashed":
                            steering = 0.5  # Move to overtake
                            speed = 0.3
                        else:
                            speed = 0.1  # Slow down and follow
            
            # State-specific behaviors
            if self.driving_state == DrivingState.PARKING:
                if self.detect_parking_spot(img):
                    steering = 0.8  # Turn into spot
                    speed = 0.1
                    self.parking_spot_found = True
                elif self.parking_spot_found:
                    self.driving_state = DrivingState.CITY
                    
            elif self.driving_state == DrivingState.ROUNDABOUT:
                if self.roundabout_entry_time:
                    time_in_roundabout = time.time() - self.roundabout_entry_time
                    steering = 0.3  # Maintain counter-clockwise motion
                    if time_in_roundabout > 5.0:  # Assume exit after 5 seconds
                        self.roundabout_entry_time = None
                        self.driving_state = DrivingState.CITY
            
            # Apply speed limits
            speed = min(speed, self.speed_limits[self.driving_state]["max"] / 100.0)
            speed = max(speed, self.speed_limits[self.driving_state]["min"] / 100.0)

             # Add more robust state transitions
            if self.driving_state == DrivingState.ROUNDABOUT:
                if self.roundabout_entry_time:
                    elapsed_time = time.time() - self.roundabout_entry_time
                    # Add distance-based exit condition
                    if elapsed_time > 5.0 or self._check_roundabout_exit():
                        self.roundabout_entry_time = None
                        self.driving_state = DrivingState.CITY

            # Add speed smoothing
            speed = self._smooth_speed(speed)  # Add speed smoothing method
            
            # Limit steering
            steering = max(-1, min(1, steering))
            
            return steering, speed
            
        except Exception as e:
            self.logging.error(f"Error in path planning: {str(e)}")
            return 0.0, 0.0

    def _change_driving_state(self, new_state):
        """Safely change driving state"""
        if new_state not in DrivingState:
            self.logging.warning(f"Invalid state transition attempted: {new_state}")
            return False
            
        old_state = self.driving_state
        self.driving_state = new_state
        self.logging.info(f"State transition: {old_state} -> {new_state}")
        return True
        
    def _smooth_speed(self, target_speed, smooth_factor=0.2):
        """Smooth speed changes to prevent jerky motion"""
        if not hasattr(self, '_current_speed'):
            self._current_speed = 0.0
        
        self._current_speed = (
            self._current_speed * (1 - smooth_factor) + 
            target_speed * smooth_factor
        )
        return self._current_speed

    def _decode_frame(self, frame_data):
        """Decode frame from base64 data"""
        try:
            img_data = base64.b64decode(frame_data)
            nparr = np.frombuffer(img_data, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError("Failed to decode image")
            return img
        except Exception as e:
            self.logging.error(f"Error decoding image: {str(e)}")
            return None
    
    def run(self):
        """Main thread loop"""
        prev_steering = 0.0
        steering_smooth_factor = 0.3

        while self._running:
            try:
                # Get camera frame
                frame_data = self.camera_subscriber.receive()
                if frame_data is None:
                    time.sleep(0.01)
                    continue
                    
                # Process frame
                img = self._decode_frame(frame_data)
                if img is None:
                    continue

                # Run detection pipeline
                detections = self.detect_objects(img)
                lane_offset = self.detect_lanes(img)
                steering, speed = self.plan_path(detections, lane_offset, img)

                # Smooth steering
                steering = (
                    prev_steering * (1 - steering_smooth_factor) + 
                    steering * steering_smooth_factor
                )
                prev_steering = steering

                # Send commands
                self._send_commands(steering, speed, lane_offset)

                if self.debugging:
                    self._debug_output(steering, speed, lane_offset, detections)

            except Exception as e:
                self.logging.error(f"Error in main loop: {str(e)}")
                continue

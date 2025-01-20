import cv2
import numpy as np
from ultralytics import YOLO
from enum import Enum
import time
from dataclasses import dataclass
from typing import List, Tuple, Optional

class AreaType(Enum):
    CITY = "city"
    HIGHWAY = "highway"

class LineType(Enum):
    DASHED = "dashed"
    CONTINUOUS = "continuous"
    UNKNOWN = "unknown"

class VehicleState(Enum):
    NORMAL = "NORMAL"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"
    YIELDING = "YIELDING"
    TURNING = "TURNING"
    ERROR = "ERROR"

@dataclass
class VehicleData:
    position: Tuple[float, float]
    speed: float
    orientation: float
    area_type: AreaType
    current_lane: int
    is_overtaking: bool
    is_in_roundabout: bool
    last_communication_time: float
    current_state: VehicleState = VehicleState.NORMAL

class LaptopLaneFollowingSystem:
    def __init__(self, camera_id=0, width=640, height=480):
        # Initialize webcam
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        # Get actual camera resolution
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Initialize YOLO model
        self.model = YOLO('SPARK_workspace/camera_loader/best.pt')
        
        # Define ROI vertices - adjusted for better lane detection
        self.roi_vertices = np.array([
            [(0, self.height),
            (self.width//4, self.height//2),  # Adjusted ROI top-left
            (3*self.width//4, self.height//2),  # Adjusted ROI top-right
            (self.width, self.height)]
        ], dtype=np.int32)

    def detect_lanes(self, frame):
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply Canny edge detection with adjusted parameters
        edges = cv2.Canny(blur, 50, 150)
        
        # Create mask for ROI - Adjust these points to change where lines can be detected
        height, width = frame.shape[:2]
        roi_vertices = np.array([
            [(0, height),
            (width//4, height//2 - 20),  # Raised top points for longer lines
            (3*width//4, height//2 - 20),
            (width, height),
            (width, height//2),
            (0, height//2)]
        ], dtype=np.int32)
        
        # Create and apply mask
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, [roi_vertices], 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # Line detection with adjusted parameters
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,                # Distance resolution in pixels
            theta=np.pi/180,      # Angle resolution in radians
            threshold=25,         # Minimum number of votes (lower = more lines)
            minLineLength=40,     # Minimum line length (increase for longer lines)
            maxLineGap=50         # Maximum gap between line segments
        )
        
        # Initialize line categories
        left_lines = []
        right_lines = []
        horizontal_lines = []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 - x1 == 0:  # Avoid division by zero
                    continue
                
                # Calculate line angle
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                
                # Adjust these angles to change line categorization
                if abs(angle) < 30 and length > 30:  # Horizontal lines
                    horizontal_lines.append(line[0])
                elif -65 < angle < -25 and length > 40:  # Left lane lines (broader angle range)
                    # Extend the line downward
                    if y1 > y2:  # Make sure y1,x1 is the bottom point
                        x1, y1, x2, y2 = x2, y2, x1, y1
                    # Extend to bottom of frame
                    if y1 < height:
                        slope = (x1 - x2) / (y1 - y2)
                        x1_new = int(x2 + slope * (height - y2))
                        left_lines.append([x1_new, height, x2, y2])
                elif 25 < angle < 65 and length > 40:  # Right lane lines (broader angle range)
                    # Extend the line downward
                    if y1 > y2:  # Make sure y1,x1 is the bottom point
                        x1, y1, x2, y2 = x2, y2, x1, y1
                    # Extend to bottom of frame
                    if y1 < height:
                        slope = (x1 - x2) / (y1 - y2)
                        x1_new = int(x2 + slope * (height - y2))
                        right_lines.append([x1_new, height, x2, y2])
        
        # Draw detected lines
        line_image = np.zeros_like(frame)
        
        # Draw left lane lines in blue with increased thickness
        for line in left_lines:
            x1, y1, x2, y2 = line
            cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 3)
        
        # Draw right lane lines in green with increased thickness
        for line in right_lines:
            x1, y1, x2, y2 = line
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
        
        # Draw horizontal lines in red
        for line in horizontal_lines:
            x1, y1, x2, y2 = line
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 3)
        
        # Combine original frame with line detections
        result = cv2.addWeighted(frame, 0.8, line_image, 1.0, 0)
        
        return result, (left_lines, right_lines, horizontal_lines)

    def detect_objects(self, frame):
        detected_objects = []
        results = self.model.predict(frame, conf=0.25, iou=0.45)
        
        for result in results:
            boxes = result.boxes.cpu().numpy()
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].astype(int)
                class_id = int(box.cls[0])
                conf = box.conf[0]
                
                # Store detection information
                detected_objects.append({
                    'box': (x1, y1, x2, y2),
                    'class_id': class_id,
                    'confidence': conf
                })
        
        return frame, detected_objects

class SmartPathPlanner:
    def __init__(self, width: int = 640, height: int = 480):
        self.width = width
        self.height = height
        self.default_speed = 30.0
        self.default_steering = 0.0
        self.vehicle_data = VehicleData(
            position=(0, 0),
            speed=self.default_speed,
            orientation=0,
            area_type=AreaType.CITY,
            current_lane=1,
            is_overtaking=False,
            is_in_roundabout=False,
            last_communication_time=0
        )
        # Timers for different states
        self.stop_timer = None
        self.yield_timer = None
        self.traffic_wait_timer = None
        
        # Wait times
        self.stop_wait_time = 3.0
        self.yield_wait_time = 2.0
        self.traffic_wait_time = 5.0
        
        # Distance thresholds
        self.detection_threshold = 0.25
        self.emergency_stop_distance = 0.4
        self.slow_down_distance = 0.25
        self.caution_distance = 0.15

    def handle_car(self, box):
        """Handle other vehicles"""
        x1, y1, x2, y2 = box
        car_height = y2 - y1
        distance_factor = car_height / self.height
        
        if distance_factor > self.emergency_stop_distance:
            return (0.0, VehicleState.STOPPED)  # Emergency stop
        elif distance_factor > self.slow_down_distance:
            return (15.0, VehicleState.STOPPING)  # Slow down
        return self.handle_normal_state()

    def handle_closed_road_stand(self, box):
        """Handle closed road"""
        return (0.0, VehicleState.STOPPED)  # Always stop at closed road

    def handle_crosswalk_sign(self, box):
        """Handle crosswalk sign"""
        return (15.0, VehicleState.YIELDING)  # Slow down and be prepared to stop

    def handle_highway_entry_sign(self, box):
        """Handle highway entry"""
        self.vehicle_data.area_type = AreaType.HIGHWAY
        return (50.0, VehicleState.NORMAL)  # Increase speed for highway

    def handle_highway_exit_sign(self, box):
        """Handle highway exit"""
        self.vehicle_data.area_type = AreaType.CITY
        return (20.0, VehicleState.NORMAL)  # Decrease speed for city

    def handle_no_entry_road_sign(self, box):
        """Handle no entry sign"""
        return (0.0, VehicleState.STOPPED)  # Stop at no entry

    def handle_one_way_road_sign(self, box):
        """Handle one way road sign"""
        return (self.default_speed, VehicleState.NORMAL)  # Maintain speed but check direction

    def handle_parking_sign(self, box):
        """Handle parking sign"""
        return (10.0, VehicleState.NORMAL)  # Slow down in parking areas

    def handle_parking_spot(self, box):
        """Handle parking spot"""
        return (5.0, VehicleState.NORMAL)  # Very slow in parking spots

    def handle_pedestrian(self, box):
        """Handle pedestrian detection"""
        x1, y1, x2, y2 = box
        pedestrian_height = y2 - y1
        distance_factor = pedestrian_height / self.height
        
        if distance_factor > self.emergency_stop_distance:
            return (0.0, VehicleState.STOPPED)
        elif distance_factor > self.slow_down_distance:
            return (15.0, VehicleState.STOPPING)
        return self.handle_normal_state()

    def handle_priority_sign(self, box):
        """Handle priority sign"""
        return (self.default_speed, VehicleState.NORMAL)  # Maintain speed but be alert

    def handle_round_about_sign(self, box):
        """Handle roundabout sign"""
        self.vehicle_data.is_in_roundabout = True
        return (20.0, VehicleState.TURNING)  # Slow down for roundabout

    def handle_stop_line(self, box):
        """Handle stop line"""
        current_time = time.time()
        
        if self.stop_timer is None:
            self.stop_timer = current_time
            return (0.0, VehicleState.STOPPING)
            
        elif current_time - self.stop_timer < self.stop_wait_time:
            return (0.0, VehicleState.STOPPED)
            
        else:
            self.stop_timer = None
            return self.handle_normal_state()

    def handle_stop_sign(self, box):
        """Handle stop sign"""
        return self.handle_stop_line(box)  # Same behavior as stop line

    def handle_traffic_light(self, box):
        """Handle generic traffic light"""
        return (15.0, VehicleState.YIELDING)  # Slow down and prepare to stop

    def handle_traffic_green(self, box):
        """Handle green traffic light"""
        return self.handle_normal_state()  # Proceed normally

    def handle_traffic_red(self, box):
        """Handle red traffic light"""
        return (0.0, VehicleState.STOPPED)  # Stop at red light

    def handle_traffic_yellow(self, box):
        """Handle yellow traffic light"""
        return (15.0, VehicleState.STOPPING)  # Slow down and prepare to stop

    def handle_normal_state(self):
        """Handle normal driving state"""
        if self.vehicle_data.area_type == AreaType.HIGHWAY:
            return (50.0, VehicleState.NORMAL)
        return (self.default_speed, VehicleState.NORMAL)

    def process_detections(self, detected_objects: List[dict]) -> Tuple[float, VehicleState]:
        """Process all detected objects with proper priority"""
        if not detected_objects:
            return self.handle_normal_state()

        # Handler mapping
        handlers = {
            'car': self.handle_car,
            'closed-road-stand': self.handle_closed_road_stand,
            'crosswalk-sign': self.handle_crosswalk_sign,
            'highway-entry-sign': self.handle_highway_entry_sign,
            'highway-exit-sign': self.handle_highway_exit_sign,
            'no-entry-road-sign': self.handle_no_entry_road_sign,
            'one-way-road-sign': self.handle_one_way_road_sign,
            'parking-sign': self.handle_parking_sign,
            'parking-spot': self.handle_parking_spot,
            'pedestrian': self.handle_pedestrian,
            'priority-sign': self.handle_priority_sign,
            'round-about-sign': self.handle_round_about_sign,
            'stop-line': self.handle_stop_line,
            'stop-sign': self.handle_stop_sign,
            'traffic-light': self.handle_traffic_light,
            'traffic-green': self.handle_traffic_green,
            'traffic-red': self.handle_traffic_red,
            'traffic-yellow': self.handle_traffic_yellow
        }

        # Process objects by priority
        priority_classes = [
            'traffic-red',
            'stop-sign',
            'stop-line',
            'pedestrian',
            'car',
            'closed-road-stand',
            'no-entry-road-sign',
            'traffic-yellow',
            'crosswalk-sign',
            'round-about-sign',
            'traffic-light',
            'traffic-green',
            'highway-exit-sign',
            'highway-entry-sign',
            'one-way-road-sign',
            'priority-sign',
            'parking-sign',
            'parking-spot'
        ]

        try:
            # Check objects in priority order
            for class_name in priority_classes:
                for obj in detected_objects:
                    if obj['confidence'] < self.detection_threshold:
                        continue
                        
                    if CLASS_NAMES[obj['class_id']] == class_name:
                        return handlers[class_name](obj['box'])

        except Exception as e:
            print(f"Error processing detections: {str(e)}")
            return self.handle_normal_state()

        return self.handle_normal_state()

    def calculate_steering_angle(self, lane_info: dict) -> float:
        """Calculate steering angle based on lane position"""
        try:
            left_lines, right_lines = lane_info['lane_lines']
            
            if left_lines and right_lines:
                # Calculate average positions
                left_avg_x = np.mean([[(x1 + x2)/2 for x1, _, x2, _ in left_lines]])
                right_avg_x = np.mean([[(x1 + x2)/2 for x1, _, x2, _ in right_lines]])
                
                # Calculate center position
                center_x = (left_avg_x + right_avg_x) / 2
                frame_center = self.width / 2
                
                # Calculate deviation and steering
                deviation = center_x - frame_center
                max_deviation = self.width / 4
                max_angle = 15.0 if self.vehicle_data.area_type == AreaType.CITY else 10.0
                
                # Apply steering calculation
                steering_angle = (deviation / max_deviation) * max_angle
                steering_angle = np.clip(steering_angle, -max_angle, max_angle)
                
                # Adjust for roundabout
                if self.vehicle_data.is_in_roundabout:
                    steering_angle *= 1.5  # Increase steering for tighter turns
                
                return round(steering_angle, 1)
                
            elif left_lines:
                return 5.0  # Gentle right turn
            elif right_lines:
                return -5.0  # Gentle left turn
                
        except Exception as e:
            print(f"Error calculating steering angle: {str(e)}")
            
        return 0.0

    def plan_path(self, detected_objects: List[dict], lane_info: dict) -> Tuple[float, float, str]:
        """Main path planning function"""
        try:
            # Process detections and get vehicle response
            speed, state = self.process_detections(detected_objects)
            
            # Calculate steering angle
            steering_angle = self.calculate_steering_angle(lane_info)
            
            # Update vehicle data
            self.vehicle_data.speed = speed
            self.vehicle_data.current_state = state
            
            # Reset roundabout flag if we're not detecting it anymore
            if state != VehicleState.TURNING:
                self.vehicle_data.is_in_roundabout = False
            
            return speed, steering_angle, state.value
            
        except Exception as e:
            print(f"Error in path planning: {str(e)}")
            return self.default_speed, 0.0, VehicleState.ERROR.value

class SmartVehicleSystem(LaptopLaneFollowingSystem):
    def __init__(self, camera_id=0, width=640, height=480):
        super().__init__(camera_id, width, height)
        self.path_planner = SmartPathPlanner(width=width, height=height)
        self.prev_steering = 0
        self.steering_smooth_factor = 0.3

    def run(self):
        try:
            while True:
                ret, original_frame = self.cap.read()
                if not ret:
                    print("Failed to grab frame")
                    break

                frame = original_frame.copy()

                # Detect lanes
                _, lane_lines = self.detect_lanes(frame)

                # Detect objects and store them for visualization
                _, detected_objects = self.detect_objects(frame)
                self.current_detections = detected_objects  # Store current detections
                
                # Get path planning decisions
                speed, steering_angle, state = self.path_planner.plan_path(
                    detected_objects,
                    {"lane_lines": (lane_lines[0], lane_lines[1])}
                )

                # Create visualization with all elements
                result = self.create_visualization(original_frame, lane_lines, speed, steering_angle, state)
                
                # Show final result
                cv2.imshow("Smart Vehicle System", result)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            self.cap.release()
            cv2.destroyAllWindows()

    def create_visualization(self, frame, lane_lines, speed, steering_angle, state):
        height, width = frame.shape[:2]
        
        # Create separate overlays
        lane_overlay = np.zeros_like(frame)
        info_overlay = np.zeros_like(frame)
        detection_overlay = np.zeros_like(frame)
        
        # Draw lane lines
        left_lines, right_lines, horizontal_lines = lane_lines
        
        for line in left_lines:
            x1, y1, x2, y2 = line
            cv2.line(lane_overlay, (x1, y1), (x2, y2), (255, 0, 0), 2)
        
        for line in right_lines:
            x1, y1, x2, y2 = line
            cv2.line(lane_overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        for line in horizontal_lines:
            x1, y1, x2, y2 = line
            cv2.line(lane_overlay, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # Draw current detections
        if hasattr(self, 'current_detections') and self.current_detections:
            for obj in self.current_detections:
                x1, y1, x2, y2 = obj['box']
                class_id = obj['class_id']
                conf = obj['confidence']
                
                # Draw bounding box
                cv2.rectangle(detection_overlay, (x1, y1), (x2, y2), (0, 255, 255), 2)
                
                # Prepare and draw label
                label = f"{CLASS_NAMES[class_id]}: {conf:.2f}"
                
                # Get label size for background rectangle
                (label_w, label_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                
                # Draw label background
                cv2.rectangle(detection_overlay, 
                            (x1, y1 - label_h - 10), 
                            (x1 + label_w, y1),
                            (0, 255, 255), -1)
                
                # Draw label text
                cv2.putText(detection_overlay, label, (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        # Draw status information
        cv2.putText(info_overlay, f"State: {state}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 
                (0, 255, 0) if state == "NORMAL" else (0, 0, 255), 2)
        
        cv2.putText(info_overlay, f"Speed: {speed:.1f} cm/s",
                (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1,
                (0, 255, 0) if speed == 30.0 else (0, 0, 255), 2)
        
        cv2.putText(info_overlay, f"Steering: {steering_angle:.1f} degrees",
                (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1,
                (0, 255, 0) if abs(steering_angle) < 1.0 else (0, 0, 255), 2)
        
        # Draw intersection warning
        if len(horizontal_lines) > 0:
            cv2.putText(info_overlay, "INTERSECTION", (width-200, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Draw steering indicator
        center_x = width // 2
        center_y = height - 50
        steering_length = 100
        steering_x = center_x + int(steering_length * np.tan(np.radians(steering_angle)))
        cv2.line(info_overlay, (center_x, center_y),
                (steering_x, center_y - steering_length), (0, 255, 0), 3)
        
        # Combine all overlays with frame
        result = cv2.addWeighted(frame, 1.0, lane_overlay, 0.8, 0)
        result = cv2.addWeighted(result, 1.0, detection_overlay, 1.0, 0)
        result = cv2.addWeighted(result, 1.0, info_overlay, 0.7, 0)
        
        return result

# Class names
CLASS_NAMES = ['car', 'closed-road-stand', 'crosswalk-sign', 'highway-entry-sign',
               'highway-exit-sign', 'no-entry-road-sign', 'one-way-road-sign',
               'parking-sign', 'parking-spot', 'pedestrian', 'priority-sign',
               'round-about-sign', 'stop-line', 'stop-sign', 'traffic-light',
               'traffic-green', 'traffic-red', 'traffic-yellow']

if __name__ == "__main__":
    system = SmartVehicleSystem()
    system.run()
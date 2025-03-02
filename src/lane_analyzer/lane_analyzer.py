import math
from collections import deque
import numpy as np
import cv2

class LaneAnalyzer:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.frame_width = width
        self.bus_lane_region = None
        self.last_lane_type = 'dashed'
        self.last_detections = deque(maxlen=5)  # Increased buffer size for better temporal smoothing
        self.detection_threshold = 15
        self.crosswalk_detector = CrosswalkDetector(width, height)
        self.in_crosswalk_zone = False
        self.crosswalk_memory = deque(maxlen=10)  # Remember crosswalk status for last 10 frames
        
    def detect_lane_type(self, frame, lane_detection_results):
        """
        Optimized lane type detection with temporal smoothing and crosswalk filtering
        """
        # First check if we're in a crosswalk zone
        self.in_crosswalk_zone = self.crosswalk_detector.detect_crosswalk(frame)
        self.crosswalk_memory.append(self.in_crosswalk_zone)
        
        # If we've detected crosswalk patterns in majority of recent frames
        if sum(self.crosswalk_memory) > len(self.crosswalk_memory) / 2:
            # During crosswalk crossing, maintain the most recent reliable lane type
            # without trying to detect new types (which could be noisy)
            return self.last_lane_type
            
        left_lane = lane_detection_results.get("left", [])
        right_lane = lane_detection_results.get("right", [])
        
        # Quick return if no lanes detected
        if not left_lane and not right_lane:
            return self.last_lane_type
            
        # Fast lane type detection
        current_type = self._fast_lane_detection(left_lane, right_lane)
        
        # Update detection history
        self.last_detections.append(current_type)
        
        # Use majority voting from last detections for stability
        # This adds temporal smoothing to prevent rapid lane type changes
        if len(self.last_detections) >= 3:
            dashed_count = sum(1 for t in self.last_detections if t == 'dashed')
            # Require stronger consensus to change lane type (increased threshold)
            if self.last_lane_type == 'dashed':
                self.last_lane_type = 'dashed' if dashed_count > len(self.last_detections) // 3 else 'continuous'
            else:
                # Require more evidence to switch back to dashed
                self.last_lane_type = 'dashed' if dashed_count > len(self.last_detections) * 2 // 3 else 'continuous'
            
        return self.last_lane_type

    def _fast_lane_detection(self, left_lane, right_lane):
        """
        Faster lane type detection using simplified metrics
        """
        is_dashed = False
        
        # Check only critical points for faster processing
        if left_lane:
            gaps = self._count_significant_gaps(left_lane)
            if gaps >= 2:  # Reduced threshold for faster detection
                is_dashed = True
                
        if not is_dashed and right_lane:
            gaps = self._count_significant_gaps(right_lane)
            if gaps >= 2:
                is_dashed = True
                
        return 'dashed' if is_dashed else 'continuous'

    def _count_significant_gaps(self, lane_points):
        """
        Count only significant gaps between points for faster processing
        """
        if len(lane_points) < 2:
            return 0
            
        gaps = 0
        prev_point = lane_points[0]
        
        for point in lane_points[1:]:
            distance = math.sqrt((point[0] - prev_point[0])**2 + 
                               (point[1] - prev_point[1])**2)
            if distance > self.detection_threshold:
                # Additional check to filter out potential crosswalk noise
                if self.in_crosswalk_zone and distance > self.detection_threshold * 1.5:
                    # Higher threshold during crosswalk zones to avoid false positives
                    gaps += 1
                elif not self.in_crosswalk_zone and distance > self.detection_threshold:
                    gaps += 1
            prev_point = point
            
        return gaps

    def is_safe_to_overtake(self, obstacle_vehicles, lane_type, vehicle_state, road_conditions=None):
        """
        Optimized overtaking safety check with faster response
        """
        # Never overtake in crosswalk zones
        if self.in_crosswalk_zone:
            return False, "No overtaking in crosswalk zone"
            
        # Quick rejection for continuous lanes
        if lane_type == 'continuous':
            return False, "No overtaking in continuous lanes"
            
        # Fast check for critical safety conditions
        if road_conditions and (
            road_conditions.get('visibility', 100) < 100 or  # Reduced threshold
            road_conditions.get('oncoming_traffic', False)
        ):
            return False, "Unsafe conditions"

        # Quick vehicle capability check
        if vehicle_state.current_speed < vehicle_state.max_safe_speed * 0.8:
            return False, "Insufficient speed capability"

        # Fast adjacent lane check
        adjacent_lane_clear = all(
            v['position'][0] < self.frame_width/3 or 
            v['position'][0] > 2*self.frame_width/3 
            for v in obstacle_vehicles
        )
        
        if not adjacent_lane_clear:
            return False, "Adjacent lane occupied"

        # Simplified TTC calculation
        for vehicle in obstacle_vehicles:
            if vehicle['speed'] > vehicle_state.current_speed:
                ttc = self._quick_ttc(vehicle, vehicle_state)
                if ttc < 4:  # Reduced threshold for faster response
                    return False, "Approaching traffic"

        return True, "Safe to overtake"

    def _quick_ttc(self, vehicle, vehicle_state):
        """
        Simplified TTC calculation for faster processing
        """
        relative_speed = max(1, vehicle['speed'] - vehicle_state.current_speed)
        distance = vehicle['position'][1]
        return distance / relative_speed


class CrosswalkDetector:
    """
    Class to detect crosswalk patterns in the image
    """
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.roi_height = int(height * 0.6)  # Consider bottom 60% of image for crosswalk detection
        self.horizontal_line_threshold = 0.5  # Min percentage of width to consider a line horizontal
        self.min_line_length = width * 0.1  # Min length for a line to be considered
        self.min_parallel_lines = 3  # Min number of parallel lines to consider a crosswalk
        self.last_detections = deque(maxlen=5)  # For temporal smoothing
        
    def detect_crosswalk(self, frame):
        """
        Detect if there's a crosswalk pattern in the frame
        
        Returns:
            bool: True if crosswalk detected, False otherwise
        """
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Create ROI
            roi = gray[self.height - self.roi_height:self.height, :]
            
            # Apply blurring to reduce noise
            blurred = cv2.GaussianBlur(roi, (5, 5), 0)
            
            # Apply Canny edge detection
            edges = cv2.Canny(blurred, 50, 150)
            
            # Apply Hough Line Transform to detect lines
            lines = cv2.HoughLinesP(
                edges, 
                rho=1, 
                theta=np.pi/180, 
                threshold=50, 
                minLineLength=self.min_line_length, 
                maxLineGap=20
            )
            
            # Check if any lines detected
            if lines is None:
                self.last_detections.append(False)
                return False
                
            # Count horizontal lines (typical for crosswalks)
            horizontal_lines = 0
            
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Check if line is approximately horizontal
                if abs(y2 - y1) < self.horizontal_line_threshold * abs(x2 - x1):
                    horizontal_lines += 1
            
            # Crosswalk typically has multiple parallel horizontal lines
            is_crosswalk = horizontal_lines >= self.min_parallel_lines
            
            # Add to detection history
            self.last_detections.append(is_crosswalk)
            
            # Use temporal smoothing - require multiple consecutive detections
            return sum(self.last_detections) >= 3
            
        except Exception as e:
            print(f"Error in crosswalk detection: {e}")
            return False

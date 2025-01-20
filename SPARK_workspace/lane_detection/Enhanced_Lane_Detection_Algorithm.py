import cv2
import numpy as np
from typing import Tuple, List, Optional
import logging

class EnhancedLaneDetector:
    def __init__(self, config: dict = None):
        """
        Initialize lane detector with configurable parameters
        
        Args:
            config (dict): Configuration parameters for lane detection
        """
        self.config = {
            'roi_height_ratio': 0.55,      # ROI height from bottom (0.0 to 1.0)
            'roi_width_ratio': 0.1,        # ROI width from sides (0.0 to 0.5)
            'blur_kernel': (5, 5),         # Gaussian blur kernel size
            'canny_low': 50,               # Lower threshold for Canny
            'canny_high': 150,             # Upper threshold for Canny
            'hough_threshold': 25,         # Minimum votes for Hough lines
            'min_line_length': 40,         # Minimum line length
            'max_line_gap': 50,            # Maximum gap between lines
            'line_memory_frames': 5,       # Number of frames to remember lines
            'min_line_angle': 25,          # Minimum angle for valid lines
            'max_line_angle': 65,          # Maximum angle for valid lines
            'horizontal_angle_thresh': 30,  # Threshold for horizontal lines
        }
        if config:
            self.config.update(config)
        
        # Initialize line memory for temporal smoothing
        self.left_lines_memory: List[List[int]] = []
        self.right_lines_memory: List[List[int]] = []
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

    def preprocess_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Preprocess frame with noise reduction and edge detection
        
        Args:
            frame: Input frame in BGR format
            
        Returns:
            Tuple of processed edges and mask
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply advanced denoising
        denoised = cv2.fastNlMeansDenoising(gray)
        
        # Apply Gaussian blur
        blur = cv2.GaussianBlur(denoised, self.config['blur_kernel'], 0)
        
        # Adaptive thresholding for better edge detection in varying lighting
        thresh = cv2.adaptiveThreshold(
            blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY, 11, 2
        )
        
        # Apply Canny edge detection with automatic threshold calculation
        median = np.median(thresh)
        lower = int(max(0, (1.0 - 0.33) * median))
        upper = int(min(255, (1.0 + 0.33) * median))
        edges = cv2.Canny(thresh, lower, upper)
        
        return edges, thresh

    def create_roi_mask(self, frame_shape: Tuple[int, int]) -> np.ndarray:
        """
        Create ROI mask with dynamic sizing based on frame dimensions
        
        Args:
            frame_shape: Tuple of frame height and width
            
        Returns:
            ROI mask array
        """
        height, width = frame_shape[:2]
        roi_height = int(height * self.config['roi_height_ratio'])
        roi_width_offset = int(width * self.config['roi_width_ratio'])
        
        # Define ROI polygon with dynamic points
        roi_vertices = np.array([
            [(0, height),
             (roi_width_offset, roi_height),
             (width - roi_width_offset, roi_height),
             (width, height),
             (width, roi_height),
             (0, roi_height)]
        ], dtype=np.int32)
        
        # Create and return mask
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(mask, [roi_vertices], 255)
        return mask

    def filter_and_group_lines(self, lines: np.ndarray, frame_shape: Tuple[int, int]) -> Tuple[List, List, List]:
        """
        Filter and group detected lines into categories with advanced filtering
        
        Args:
            lines: Detected Hough lines
            frame_shape: Frame dimensions
            
        Returns:
            Tuple of left, right, and horizontal line groups
        """
        height = frame_shape[0]
        left_lines = []
        right_lines = []
        horizontal_lines = []
        
        if lines is None:
            return [], [], []
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0:
                continue
            
            # Calculate line properties
            angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            slope = (y2 - y1) / (x2 - x1)
            
            # Filter out invalid lines based on properties
            if length < self.config['min_line_length']:
                continue
                
            if abs(slope) < 0.1:  # Nearly horizontal lines
                if abs(angle) < self.config['horizontal_angle_thresh']:
                    horizontal_lines.append(line[0])
            elif -self.config['max_line_angle'] < angle < -self.config['min_line_angle']:
                # Left lane line processing
                if y1 > y2:  # Ensure correct orientation
                    x1, y1, x2, y2 = x2, y2, x1, y1
                # Extend line to bottom of frame
                if y1 < height:
                    slope = (x1 - x2) / (y1 - y2)
                    x1_new = int(x2 + slope * (height - y2))
                    left_lines.append([x1_new, height, x2, y2])
            elif self.config['min_line_angle'] < angle < self.config['max_line_angle']:
                # Right lane line processing
                if y1 > y2:
                    x1, y1, x2, y2 = x2, y2, x1, y1
                if y1 < height:
                    slope = (x1 - x2) / (y1 - y2)
                    x1_new = int(x2 + slope * (height - y2))
                    right_lines.append([x1_new, height, x2, y2])
        
        return left_lines, right_lines, horizontal_lines

    def smooth_lines(self, left_lines: List, right_lines: List) -> Tuple[List, List]:
        """
        Apply temporal smoothing to detected lines
        
        Args:
            left_lines: Current frame's left lines
            right_lines: Current frame's right lines
            
        Returns:
            Smoothed left and right lines
        """
        # Update line memory
        self.left_lines_memory.append(left_lines)
        self.right_lines_memory.append(right_lines)
        
        # Keep only recent frames
        if len(self.left_lines_memory) > self.config['line_memory_frames']:
            self.left_lines_memory.pop(0)
        if len(self.right_lines_memory) > self.config['line_memory_frames']:
            self.right_lines_memory.pop(0)
        
        # Average lines over recent frames
        smoothed_left = []
        smoothed_right = []
        
        if self.left_lines_memory:
            all_left = np.array([line for frame in self.left_lines_memory for line in frame])
            if len(all_left) > 0:
                mean_left = np.mean(all_left, axis=0)
                smoothed_left = [mean_left.astype(int)]
        
        if self.right_lines_memory:
            all_right = np.array([line for frame in self.right_lines_memory for line in frame])
            if len(all_right) > 0:
                mean_right = np.mean(all_right, axis=0)
                smoothed_right = [mean_right.astype(int)]
        
        return smoothed_left, smoothed_right

    def detect_lanes(self, frame: np.ndarray) -> Tuple[np.ndarray, Tuple[List, List, List]]:
        """
        Main lane detection method with enhanced processing
        
        Args:
            frame: Input frame in BGR format
            
        Returns:
            Tuple of processed frame and detected line groups
        """
        try:
            # Preprocess frame
            edges, thresh = self.preprocess_frame(frame)
            
            # Create and apply ROI mask
            mask = self.create_roi_mask(frame.shape)
            masked_edges = cv2.bitwise_and(edges, mask)
            
            # Detect lines using Hough transform
            lines = cv2.HoughLinesP(
                masked_edges,
                rho=1,
                theta=np.pi/180,
                threshold=self.config['hough_threshold'],
                minLineLength=self.config['min_line_length'],
                maxLineGap=self.config['max_line_gap']
            )
            
            # Filter and group lines
            left_lines, right_lines, horizontal_lines = self.filter_and_group_lines(lines, frame.shape)
            
            # Apply temporal smoothing
            smoothed_left, smoothed_right = self.smooth_lines(left_lines, right_lines)
            
            # Draw lines on output frame
            line_image = np.zeros_like(frame)
            
            # Draw left lane lines (blue)
            for line in smoothed_left:
                x1, y1, x2, y2 = line
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 3)
            
            # Draw right lane lines (green)
            for line in smoothed_right:
                x1, y1, x2, y2 = line
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
            
            # Draw horizontal lines (red)
            for line in horizontal_lines:
                x1, y1, x2, y2 = line
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 3)
            
            # Combine original frame with lines
            result = cv2.addWeighted(frame, 0.8, line_image, 1.0, 0)
            
            return result, (smoothed_left, smoothed_right, horizontal_lines)
            
        except Exception as e:
            self.logger.error(f"Error in lane detection: {str(e)}")
            return frame, ([], [], [])

    def get_lane_info(self, lines: Tuple[List, List, List]) -> dict:
        """
        Extract lane information from detected lines
        
        Args:
            lines: Tuple of left, right, and horizontal lines
            
        Returns:
            Dictionary containing lane information
        """
        left_lines, right_lines, horizontal_lines = lines
        
        info = {
            'left_lane_detected': len(left_lines) > 0,
            'right_lane_detected': len(right_lines) > 0,
            'horizontal_lines_count': len(horizontal_lines),
            'lane_width': None,
            'lane_center_offset': None
        }
        
        # Calculate lane width and center offset if both lanes are detected
        if info['left_lane_detected'] and info['right_lane_detected']:
            left_x = np.mean([line[0] for line in left_lines])
            right_x = np.mean([line[0] for line in right_lines])
            info['lane_width'] = right_x - left_x
            info['lane_center_offset'] = (left_x + right_x) / 2
        
        return info
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUIntegration:
    """
    Class for integrating IMU data to track vehicle position.
    
    This class implements a simple dead reckoning algorithm using IMU data
    (accelerometer, gyroscope) combined with the vehicle's speed to estimate 
    the vehicle's position and orientation over time.
    """
    
    def __init__(self, initial_x=0, initial_y=0, initial_yaw=0, logger=None):
        """
        Initialize the IMU integration module.
        
        Args:
            initial_x (float): Initial X coordinate (meters)
            initial_y (float): Initial Y coordinate (meters)
            initial_yaw (float): Initial yaw angle (degrees)
            logger: Logger instance for debugging
        """
        # Current pose
        self.x = initial_x
        self.y = initial_y
        self.yaw = initial_yaw  # degrees
        
        # Previous measurements for filtering
        self.prev_accel = np.zeros(3)
        self.prev_gyro = np.zeros(3)
        self.prev_speed = 0
        
        # Calibration offsets
        self.accel_offset = np.zeros(3)
        self.gyro_offset = np.zeros(3)
        
        # Filtering parameters
        self.alpha_accel = 0.2  # Low-pass filter constant for acceleration
        self.alpha_gyro = 0.3   # Low-pass filter constant for gyroscope
        self.alpha_speed = 0.4  # Low-pass filter constant for speed
        
        # Dead reckoning parameters
        self.last_update_time = time.time()
        self.distance_traveled = 0
        
        # Noise thresholds
        self.accel_threshold = 0.05  # m/s²
        self.gyro_threshold = 0.02    # rad/s
        
        self.logger = logger
    
    def calibrate(self, accel_samples, gyro_samples):
        """
        Calibrate the IMU by computing the average offsets.
        
        Args:
            accel_samples (list): List of accelerometer readings
            gyro_samples (list): List of gyroscope readings
        """
        if not accel_samples or not gyro_samples:
            return
        
        # Compute average offsets
        self.accel_offset = np.mean(accel_samples, axis=0)
        self.gyro_offset = np.mean(gyro_samples, axis=0)
        
        if self.logger:
            self.logger.info(f"IMU calibrated: accel_offset={self.accel_offset}, gyro_offset={self.gyro_offset}")
    
    def update(self, accel, gyro, speed, delta_time=None):
        """
        Update position estimate using IMU data and speed.
        
        Args:
            accel (numpy.ndarray): 3D accelerometer reading [ax, ay, az] in m/s²
            gyro (numpy.ndarray): 3D gyroscope reading [wx, wy, wz] in rad/s
            speed (float): Vehicle speed in m/s
            delta_time (float): Time elapsed since last update (s). If None, computed automatically.
            
        Returns:
            tuple: Updated (x, y, yaw) position
        """
        # Calculate time delta
        current_time = time.time()
        dt = delta_time if delta_time is not None else (current_time - self.last_update_time)
        self.last_update_time = current_time
        
        # Ensure minimum time delta to avoid division by zero
        if dt < 0.001:
            dt = 0.001
        
        # Apply calibration offset
        accel = accel - self.accel_offset
        gyro = gyro - self.gyro_offset
        
        # Apply noise threshold
        accel = np.where(np.abs(accel) < self.accel_threshold, 0, accel)
        gyro = np.where(np.abs(gyro) < self.gyro_threshold, 0, gyro)
        
        # Apply low-pass filter
        accel = self.alpha_accel * accel + (1 - self.alpha_accel) * self.prev_accel
        gyro = self.alpha_gyro * gyro + (1 - self.alpha_gyro) * self.prev_gyro
        filtered_speed = self.alpha_speed * speed + (1 - self.alpha_speed) * self.prev_speed
        
        # Save current values for next iteration
        self.prev_accel = accel
        self.prev_gyro = gyro
        self.prev_speed = filtered_speed
        
        # Extract useful components (assuming vehicle is mostly moving on x-y plane)
        yaw_rate = gyro[2]  # z-axis rotation rate
        
        # Update yaw (degrees)
        yaw_change = math.degrees(yaw_rate * dt)
        self.yaw += yaw_change
        
        # Normalize yaw to [-180, 180)
        self.yaw = (self.yaw + 180) % 360 - 180
        
        # Convert speed to x, y components using yaw
        yaw_rad = math.radians(self.yaw)
        dx = filtered_speed * math.cos(yaw_rad) * dt
        dy = filtered_speed * math.sin(yaw_rad) * dt
        
        # Update position
        self.x += dx
        self.y += dy
        
        # Update total distance traveled
        self.distance_traveled += abs(filtered_speed * dt)
        
        return (self.x, self.y, self.yaw)
    
    def reset(self, x=0, y=0, yaw=0):
        """
        Reset the position tracking to the given coordinates.
        
        Args:
            x (float): New X coordinate
            y (float): New Y coordinate
            yaw (float): New yaw angle in degrees (optional, default: 0)
        """
        self.x = x
        self.y = y
        
        if yaw is not None:
            self.yaw = yaw
        
        self.distance_traveled = 0
        self.last_update_time = time.time()
        
        
        if self.logger:
            self.logger.info(f"IMU integration reset to position: ({x}, {y}), yaw: {self.yaw}°")
            
        return x, y, self.yaw
    
    def get_pose(self):
        """
        Get the current pose of the vehicle.
        
        Returns:
            tuple: (x, y, yaw) position and orientation
        """
        return (self.x, self.y, self.yaw)
    
    def get_distance_traveled(self):
        """
        Get the total distance traveled since last reset.
        
        Returns:
            float: Distance in meters
        """
        return self.distance_traveled


class EnhancedPositionEstimator:
    """
    Enhanced position estimator that combines IMU integration with map matching.
    
    This class improves position estimates by using the map graph to constrain
    the vehicle position to the road network.
    """
    
    def __init__(self, path_planner, initial_x=0, initial_y=0, initial_yaw=0, logger=None):
        """
        Initialize the enhanced position estimator.
        
        Args:
            path_planner (PathPlanner): Path planner instance with map data
            initial_x (float): Initial X coordinate
            initial_y (float): Initial Y coordinate
            initial_yaw (float): Initial yaw angle (degrees)
            logger: Logger instance for debugging
        """
        self.imu_integration = IMUIntegration(initial_x, initial_y, initial_yaw, logger)
        self.path_planner = path_planner
        self.logger = logger
        
        # Position correction parameters
        self.map_matching_weight = 0.3  # Weight for map correction vs. IMU integration
        self.max_correction_distance = 0.5  # Maximum correction distance (meters)
        
        # Last known map-matched position
        self.map_matched_position = (initial_x, initial_y)
        self.on_road = True
    
    def update(self, accel, gyro, speed, delta_time=None):
        """
        Update position estimate using IMU data and map matching.
        
        Args:
            accel (numpy.ndarray): 3D accelerometer reading [ax, ay, az] in m/s²
            gyro (numpy.ndarray): 3D gyroscope reading [wx, wy, wz] in rad/s
            speed (float): Vehicle speed in m/s
            delta_time (float): Time elapsed since last update (s)
            
        Returns:
            tuple: Updated (x, y, yaw) position
        """
        # Update dead reckoning position
        x, y, yaw = self.imu_integration.update(accel, gyro, speed, delta_time)
        
        # Skip map matching if we're not following a path
        if not self.path_planner.current_path:
            return x, y, yaw
        
        # Find the closest node in the path
        closest_node = self.path_planner.find_closest_node(x, y)
        closest_node_x = float(self.path_planner.G.nodes[closest_node]['x'])
        closest_node_y = float(self.path_planner.G.nodes[closest_node]['y'])
        
        # Find the closest point on the path segment
        map_x, map_y = self._find_closest_point_on_path((x, y))
        
        # Calculate correction
        correction_x = map_x - x
        correction_y = map_y - y
        correction_distance = math.sqrt(correction_x**2 + correction_y**2)
        
        # Apply correction if it's within limits
        if correction_distance < self.max_correction_distance:
            # Apply weighted correction
            corrected_x = x + correction_x * self.map_matching_weight
            corrected_y = y + correction_y * self.map_matching_weight
            
            # Update the map-matched position
            self.map_matched_position = (map_x, map_y)
            self.on_road = True
            
            # Update position in IMU integration
            self.imu_integration.x = corrected_x
            self.imu_integration.y = corrected_y
            
            return corrected_x, corrected_y, yaw
        else:
            self.on_road = False
            return x, y, yaw
    
    def _find_closest_point_on_path(self, position):
        """
        Find the closest point on the current path to the given position.
        
        Args:
            position (tuple): (x, y) coordinate
            
        Returns:
            tuple: (x, y) of the closest point on the path
        """
        if not self.path_planner.current_path:
            return position
        
        x, y = position
        
        # Get current and next node in path
        current_idx = max(0, self.path_planner.target_node_index - 1)
        next_idx = min(current_idx + 1, len(self.path_planner.current_path) - 1)
        
        if current_idx == next_idx:
            # We're at the end of the path
            node_id = self.path_planner.current_path[current_idx]
            return (float(self.path_planner.G.nodes[node_id]['x']), 
                    float(self.path_planner.G.nodes[node_id]['y']))
        
        # Get the coordinates of the two nodes
        current_node = self.path_planner.current_path[current_idx]
        next_node = self.path_planner.current_path[next_idx]
        
        x1 = float(self.path_planner.G.nodes[current_node]['x'])
        y1 = float(self.path_planner.G.nodes[current_node]['y'])
        x2 = float(self.path_planner.G.nodes[next_node]['x'])
        y2 = float(self.path_planner.G.nodes[next_node]['y'])
        
        # Calculate the closest point on the line segment
        dx = x2 - x1
        dy = y2 - y1
        
        # Handle case where nodes are at the same location
        if dx == 0 and dy == 0:
            return (x1, y1)
        
        # Calculate projection
        t = ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)
        
        # Limit to line segment
        t = max(0, min(1, t))
        
        # Calculate the closest point
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        return (closest_x, closest_y)
    
    def calibrate(self, accel_samples, gyro_samples):
        """
        Calibrate the IMU integration.
        
        Args:
            accel_samples (list): List of accelerometer readings
            gyro_samples (list): List of gyroscope readings
        """
        self.imu_integration.calibrate(accel_samples, gyro_samples)
    
    def reset(self, x=0, y=0, yaw=0):
        """
        Reset the position tracking to the given coordinates.
        
        Args:
            x (float): New X coordinate
            y (float): New Y coordinate
            yaw (float): New yaw angle in degrees
        """
        self.imu_integration.reset(x, y, yaw)
        self.map_matched_position = (x, y)
    
    def get_pose(self):
        """
        Get the current pose of the vehicle.
        
        Returns:
            tuple: (x, y, yaw) position and orientation
        """
        return self.imu_integration.get_pose()
    
    def get_distance_traveled(self):
        """
        Get the total distance traveled since last reset.
        
        Returns:
            float: Distance in meters
        """
        return self.imu_integration.get_distance_traveled()
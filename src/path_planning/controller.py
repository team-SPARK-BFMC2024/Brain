import time
import logging
import numpy as np
import math
from .path_planner import PathPlanner
from .imu_integration import EnhancedPositionEstimator

class PathPlanningController:
    """
    Controller for integrating path planning with vehicle control.
    
    This class bridges the path planning module with the vehicle's control system,
    providing steering and speed commands based on the planned path.
    """
    
    def __init__(self, map_path=None, logger=None):
        """
        Initialize the path planning controller.
        
        Args:
            map_path (str): Path to the map graph file
            logger: Logger instance for debugging
        """
        # Setup logger
        self.logger = logger or logging.getLogger("PathPlanningController")
        
        # Initialize path planner
        self.path_planner = PathPlanner(map_path, self.logger)
        
        # Initialize position estimator
        self.position_estimator = EnhancedPositionEstimator(self.path_planner, logger=self.logger)
        
        # Control parameters
        self.steering_gain = 0.5
        self.speed_gain = 1.0
        self.max_steering_angle = 25  # degrees
        
        # State variables
        self.current_path = []
        self.is_active = False
        self.last_update_time = time.time()
        
        # Sensor data
        self.imu_data = {'accel': np.zeros(3), 'gyro': np.zeros(3)}
        self.speed = 0.0  # m/s
        
        # Debug values
        self.debug_info = {}
    
    def set_path(self, start_node, end_node):
        """
        Set a new path to follow.
        
        Args:
            start_node (str): Start node ID
            end_node (str): End node ID
            
        Returns:
            bool: True if path was set successfully, False otherwise
        """
        # Plan a path
        path = self.path_planner.plan_path(start_node, end_node)
        
        if not path:
            self.logger.error(f"Failed to plan path from {start_node} to {end_node}")
            return False
        
        self.current_path = path
        self.is_active = True
        self.logger.info(f"Path set from {start_node} to {end_node}, length: {len(path)} nodes")
        
        return True
    
    def update_sensor_data(self, accel, gyro, speed):
        """
        Update sensor data for position estimation.
        
        Args:
            accel (numpy.ndarray): 3D accelerometer reading [ax, ay, az]
            gyro (numpy.ndarray): 3D gyroscope reading [wx, wy, wz]
            speed (float): Current speed in m/s
        """
        self.imu_data['accel'] = np.array(accel, dtype=float)
        self.imu_data['gyro'] = np.array(gyro, dtype=float)
        self.speed = float(speed)
    
    def update(self):
        """
        Update controller state and generate control commands.
        
        Returns:
            tuple: (steering_angle, target_speed, is_active)
        """
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Default control values (no steering, maintain speed)
        steering_angle = 0.0
        target_speed = self.speed
        
        # Skip if not active
        if not self.is_active:
            return steering_angle, target_speed, False
        
        # Update position estimate
        x, y, yaw = self.position_estimator.update(
            self.imu_data['accel'],
            self.imu_data['gyro'],
            self.speed,
            dt
        )
        
        # Update path tracking
        self.path_planner.update_position(x, y, yaw, dt)
        
        # Check if path is complete
        if self.path_planner.path_complete:
            self.logger.info("Path complete")
            self.is_active = False
            return 0.0, 0.0, False
        
        # Calculate steering based on path
        speed_factor = min(1.0, max(0.3, self.speed / 5.0))  # Normalize speed for steering calculation
        steering_angle = self.path_planner.calculate_steering(speed_factor)
        
        # Adjust speed based on path curvature
        target_speed = self._calculate_target_speed(steering_angle)
        
        # Update debug info
        self._update_debug_info(x, y, yaw, steering_angle, target_speed)
        
        return steering_angle, target_speed, True
    
    def _calculate_target_speed(self, steering_angle):
        """
        Calculate target speed based on steering angle (reduce speed in curves).
        
        Args:
            steering_angle (float): Current steering angle
            
        Returns:
            float: Target speed
        """
        # Base speed (could be configurable)
        base_speed = 200  # mm/s
        
        # Reduce speed in curves
        abs_steering = abs(steering_angle)
        speed_factor = 1.0 - (abs_steering / self.max_steering_angle) * 0.5
        
        # Ensure speed doesn't drop below a minimum
        speed_factor = max(0.5, speed_factor)
        
        # Special case for intersections
        if self.path_planner.is_at_intersection():
            speed_factor *= 0.8  # Slow down at intersections
        
        return base_speed * speed_factor
    
    def _update_debug_info(self, x, y, yaw, steering_angle, target_speed):
        """
        Update debug information.
        
        Args:
            x (float): X coordinate
            y (float): Y coordinate
            yaw (float): Yaw angle
            steering_angle (float): Calculated steering angle
            target_speed (float): Target speed
        """
        self.debug_info = {
            'position': (x, y),
            'yaw': yaw,
            'steering_angle': steering_angle,
            'target_speed': target_speed,
            'current_node': self.path_planner.current_node,
            'target_node': self.path_planner.current_path[self.path_planner.target_node_index] 
                            if self.path_planner.current_path else None,
            'progress': self.path_planner.get_progress_percentage(),
            'remaining_distance': self.path_planner.get_remaining_distance(),
            'at_intersection': self.path_planner.is_at_intersection()
        }
    
    def reset(self):
        """
        Reset controller state.
        """
        self.path_planner.current_path = []
        self.is_active = False
        self.position_estimator.reset()
        self.last_update_time = time.time()
        self.logger.info("Path planning controller reset")
    
    def calibrate_imu(self, accel_samples, gyro_samples):
        """
        Calibrate the IMU integration.
        
        Args:
            accel_samples (list): List of accelerometer readings
            gyro_samples (list): List of gyroscope readings
        """
        self.position_estimator.calibrate(accel_samples, gyro_samples)
    
    def get_debug_info(self):
        """
        Get debug information about the current state.
        
        Returns:
            dict: Debug information
        """
        return self.debug_info
    
    def find_closest_node(self, x, y):
        """
        Find the closest node in the map to the given position.
        
        Args:
            x (float): X coordinate
            y (float): Y coordinate
            
        Returns:
            str: Node ID of the closest node
        """
        return self.path_planner.find_closest_node(x, y)
    
    def get_current_position(self):
        """
        Get the current estimated position.
        
        Returns:
            tuple: (x, y, yaw)
        """
        return self.position_estimator.get_pose()
    
    def is_path_complete(self):
        """
        Check if the current path is complete.
        
        Returns:
            bool: True if path is complete, False otherwise
        """
        return self.path_planner.path_complete
    
    def get_distance_traveled(self):
        """
        Get the total distance traveled.
        
        Returns:
            float: Distance in meters
        """
        return self.position_estimator.get_distance_traveled()
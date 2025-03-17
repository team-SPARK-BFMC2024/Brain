import time
import logging
import os
import networkx as nx
import numpy as np
from math import degrees, atan2, sin, cos, radians
from .imu_processor import IMUProcessor
from .path_planner import PathPlanner

class PathController:
    """
    Controller class that combines path planning and IMU data to guide the vehicle.
    
    This controller uses a path planner to determine the optimal path and IMU data 
    to track the vehicle's progress along the path, calculating steering commands.
    """
    
    def __init__(self, map_file=None):
        """
        Initialize the path controller.
        
        Args:
            map_file (str, optional): Path to the map file (.graphml format)
        """
        self.logger = logging.getLogger('PathController')
        
        # Default map file if none provided
        if map_file is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            map_file = os.path.join(current_dir, 'maps', 'track_test_node.graphml')
            
        self.imu_processor = IMUProcessor()
        self.path_planner = PathPlanner(map_file)
        
        # Navigation state
        self.current_node = None
        self.target_node = None
        self.next_waypoint = None
        self.path = []
        self.is_active = False
        self.distance_to_next_node = float('inf')
        self.node_radius = 0.5  # meters - distance at which we consider a node reached
        
        # Control parameters
        self.max_steering_angle = 25.0  # degrees
        self.lookahead_distance = 1.0  # meters
        self.navigation_started = False
        self.current_imu_data = None
        
    def start_navigation(self, start_node=None, end_node=None):
        """
        Start navigation along a path.
        
        Args:
            start_node (str, optional): Starting node ID
            end_node (str, optional): Target node ID
            
        Returns:
            bool: True if navigation started successfully
        """
        try:
            # Reset the IMU position tracking
            self.imu_processor.reset_position()
            
            if start_node is None or end_node is None:
                # Use default navigation path - start at first node, end at last
                nodes = list(self.path_planner.graph.nodes())
                if not nodes:
                    self.logger.error("No nodes found in map")
                    return False
                    
                if start_node is None:
                    start_node = nodes[0]
                if end_node is None:
                    end_node = nodes[-1]
            
            # Find path between nodes
            self.path = self.path_planner.find_path(start_node, end_node)
            
            if not self.path:
                self.logger.error(f"No path found between {start_node} and {end_node}")
                return False
                
            self.current_node = start_node
            self.target_node = end_node
            self.next_waypoint = self._get_next_waypoint()
            
            self.is_active = True
            self.navigation_started = True
            self.logger.info(f"Navigation started from {start_node} to {end_node}")
            self.logger.info(f"Path: {' -> '.join(self.path)}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error starting navigation: {e}")
            return False
            
    def stop_navigation(self):
        """Stop the navigation process."""
        self.is_active = False
        self.navigation_started = False
        self.logger.info("Navigation stopped")
        
    def process_imu_data(self, data):
        """
        Process IMU data for navigation.
        
        Args:
            data (str): Raw IMU data string
            
        Returns:
            bool: True if data was successfully processed
        """
        if not self.imu_processor.parse_imu_data(data):
            return False
            
        self.current_imu_data = data
        return True
        
    def update(self):
        """
        Update the navigation state and calculate steering commands.
        
        Returns:
            tuple: (steering_angle, speed_recommendation, reached_target)
        """
        if not self.is_active or self.next_waypoint is None:
            return 0.0, None, False
            
        # Update position tracking with IMU data
        distance_traveled, current_yaw = self.imu_processor.update_position()
        
        # Update distance to next node
        current_position = self.imu_processor.get_position()[:2]  # x, y only
        next_position = self._get_waypoint_position(self.next_waypoint)
        
        # Calculate distance to next waypoint
        self.distance_to_next_node = np.sqrt(
            (next_position[0] - current_position[0])**2 +
            (next_position[1] - current_position[1])**2
        )
        
        # Check if we've reached the next waypoint
        if self.distance_to_next_node < self.node_radius:
            self.current_node = self.next_waypoint
            self.logger.info(f"Reached node {self.current_node}")
            
            # Check if we've reached the target
            if self.current_node == self.target_node:
                self.logger.info("Reached final destination!")
                self.is_active = False
                return 0.0, 0, True
                
            # Move to the next waypoint
            self.next_waypoint = self._get_next_waypoint()
            if not self.next_waypoint:
                self.logger.warning("No more waypoints available")
                self.is_active = False
                return 0.0, 0, False
                
            next_position = self._get_waypoint_position(self.next_waypoint)
        
        # Calculate steering angle to next waypoint
        steering_angle = self._calculate_steering_angle(current_position, current_yaw, next_position)
        
        # Determine appropriate speed based on turn sharpness
        # Sharper turns = slower speed
        turn_sharpness = abs(steering_angle) / self.max_steering_angle
        
        # For significant turns, use fixed low speed (10 cm/s)
        if turn_sharpness > 0.6:  
            speed_factor = 0.4 
        else:
            speed_factor = 1.0 - (0.6 * turn_sharpness / 0.6)
        
        return steering_angle, speed_factor, False
        
    def _get_next_waypoint(self):
        """Get the next waypoint in the path."""
        if not self.path:
            return None
            
        current_index = self.path.index(self.current_node) if self.current_node in self.path else -1
        
        if current_index < 0 or current_index >= len(self.path) - 1:
            return None
            
        return self.path[current_index + 1]
        
    def _get_waypoint_position(self, node_id):
        """Get the position of a waypoint from the graph."""
        if node_id not in self.path_planner.graph:
            self.logger.warning(f"Node {node_id} not found in graph")
            return (0, 0)
            
        node_data = self.path_planner.graph.nodes[node_id]
        x = node_data.get('x', 0)
        y = node_data.get('y', 0)
        
        return (float(x), float(y))
        
    def _calculate_steering_angle(self, current_position, current_yaw, target_position):
        """
        Calculate the steering angle needed to reach the target.
        
        Args:
            current_position (tuple): Current position (x, y)
            current_yaw (float): Current yaw angle in degrees
            target_position (tuple): Target position (x, y)
            
        Returns:
            float: Steering angle in degrees
        """
        # Vector from current position to target
        dx = target_position[0] - current_position[0]
        dy = target_position[1] - current_position[1]
        
        # Calculate desired heading angle
        desired_yaw_rad = atan2(dy, dx)
        desired_yaw = degrees(desired_yaw_rad)
        
        # Normalize to [-180, 180]
        desired_yaw = (desired_yaw + 180) % 360 - 180
        current_yaw = (current_yaw + 180) % 360 - 180
        
        # Calculate the difference between current and desired yaw
        yaw_error = desired_yaw - current_yaw
        
        # Normalize to [-180, 180]
        if yaw_error > 180:
            yaw_error -= 360
        elif yaw_error < -180:
            yaw_error += 360
            
        # Convert yaw error to steering angle
        # We need to limit the max steering angle
        steering_angle = max(min(yaw_error, self.max_steering_angle), -self.max_steering_angle)
        
        return steering_angle
        
    def get_navigation_status(self):
        """
        Get the current navigation status.
        
        Returns:
            dict: Status information including current position, next waypoint, etc.
        """
        position = self.imu_processor.get_position()
        
        return {
            'active': self.is_active,
            'current_node': self.current_node,
            'next_node': self.next_waypoint,
            'target_node': self.target_node,
            'position_x': position[0],
            'position_y': position[1],
            'yaw': position[2],
            'distance_to_next': self.distance_to_next_node,
            'total_distance': self.imu_processor.get_total_distance()
        }

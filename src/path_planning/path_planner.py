import networkx as nx
import numpy as np
from scipy.spatial import distance
import math
import time
import os
import logging

class PathPlanner:
    """
    Improved path planning class that calculates and manages the path for the autonomous vehicle.
    
    This class uses A* algorithm with the BFMC map graph and integrates with the vehicle's IMU
    to track position and progress along the path.
    """
    
    def __init__(self, map_path=None, logger=None):
        """
        Initialize the PathPlanner with map data and required parameters.
        
        Args:
            map_path (str): Path to the graphml file containing the map graph
            logger (logging.Logger): Logger object for debug messages
        """
        # Setup logger
        self.logger = logger or logging.getLogger("PathPlanner")
        
        # Set default map path if none provided
        if map_path is None:
            # Try to locate map file in common locations
            possible_paths = [
                os.path.join(os.path.dirname(__file__), "maps", "converted_graph.graphml"),
                os.path.join(os.path.dirname(__file__), "converted_graph.graphml"),
                "converted_graph.graphml"
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    map_path = path
                    break
            
            if map_path is None:
                raise FileNotFoundError("Map file not found. Please provide a valid path.")
        
        self.logger.info(f"Loading map from: {map_path}")
        
        # Load the map graph
        self.G = nx.read_graphml(map_path)
        
        # Define special nodes
        self.intersection_nodes = ['19', '47', '59', '82', '83']
        self.central_nodes = ['20', '48']
        self.roundabout_entry_nodes = []  # Fill with actual node IDs if needed
        self.roundabout_exit_nodes = []   # Fill with actual node IDs if needed
        
        # Path tracking parameters
        self.current_path = []
        self.target_node_index = 0
        self.current_node = None
        self.target_node = None
        self.path_complete = False
        
        # Position tracking
        self.current_position = (0, 0)  # (x, y) coordinates
        self.current_yaw = 0  # degrees
        self.last_position_update = time.time()
        
        # Distance threshold to consider a node as reached (in meters)
        self.node_reached_threshold = 0.3
        
        # Steering control parameters
        self.steering_gain = 0.5
        self.max_steering_angle = 25
        self.turning_left = False
        self.turn_angle = 0
        
        # Progress tracking
        self.distance_traveled = 0
        self.previous_position = None
        
        self.logger.info("PathPlanner initialized successfully")
    
    def plan_path(self, start_node, target_node):
        """
        Plan a path from start_node to target_node using A* algorithm.
        
        Args:
            start_node (str): Starting node ID
            target_node (str): Target node ID
            
        Returns:
            list: List of node IDs representing the path
        """
        try:
            # Find the shortest path using A*
            path = list(nx.all_shortest_paths(self.G, source=start_node, target=target_node))[0]
            
            self.logger.info(f"Path planned from {start_node} to {target_node}: {path}")
            
            # Reset path tracking parameters
            self.current_path = path
            self.target_node_index = 0
            self.current_node = path[0]
            self.target_node = path[1] if len(path) > 1 else path[0]
            self.path_complete = False
            
            return path
        
        except nx.NetworkXNoPath:
            self.logger.error(f"No path found between {start_node} and {target_node}")
            return []
        except Exception as e:
            self.logger.error(f"Error planning path: {e}")
            return []
    
    def find_closest_node(self, x, y):
        """
        Find the closest node in the graph to the given coordinates.
        
        Args:
            x (float): X coordinate
            y (float): Y coordinate
            
        Returns:
            str: Node ID of the closest node
        """
        distances = {}
        
        for node in self.G.nodes():
            node_x = float(self.G.nodes[node]['x'])
            node_y = float(self.G.nodes[node]['y'])
            
            distances[node] = distance.euclidean((x, y), (node_x, node_y))
        
        closest_node = min(distances, key=distances.get)
        closest_distance = distances[closest_node]
        
        self.logger.debug(f"Closest node to ({x}, {y}): {closest_node} (distance: {closest_distance:.3f})")
        
        return closest_node
    
    def update_position(self, x, y, yaw, delta_time):
        """
        Update the current position and orientation of the vehicle.
        
        Args:
            x (float): X coordinate
            y (float): Y coordinate
            yaw (float): Yaw angle in degrees
            delta_time (float): Time elapsed since last update in seconds
        """
        self.current_yaw = yaw
        
        # Calculate distance traveled
        if self.previous_position is not None:
            distance_moved = distance.euclidean((x, y), self.previous_position)
            self.distance_traveled += distance_moved
        
        self.previous_position = (x, y)
        self.current_position = (x, y)
        self.last_position_update = time.time()
        
        # Check if we've reached the target node
        self.check_node_progress()
    
    def check_node_progress(self):
        """
        Check if the vehicle has reached the target node and update accordingly.
        """
        if self.path_complete or not self.current_path:
            return
        
        # Get the current target node coordinates
        target_node = self.current_path[self.target_node_index]
        target_x = float(self.G.nodes[target_node]['x'])
        target_y = float(self.G.nodes[target_node]['y'])
        
        # Calculate distance to target node
        distance_to_target = distance.euclidean(self.current_position, (target_x, target_y))
        
        # Check if we've reached the target node
        if distance_to_target < self.node_reached_threshold:
            self.logger.info(f"Reached node {target_node}")
            
            # Update current node
            self.current_node = target_node
            
            # Move to the next node
            self.target_node_index += 1
            
            # Check if we've completed the path
            if self.target_node_index >= len(self.current_path):
                self.logger.info("Path complete")
                self.path_complete = True
                return
            
            # Update target node
            self.target_node = self.current_path[self.target_node_index]
            self.logger.info(f"New target node: {self.target_node}")
    
    def calculate_steering(self, speed_factor=1.0):
        """
        Calculate the steering angle to reach the target node.
        
        Args:
            speed_factor (float): Factor to adjust steering based on speed
            
        Returns:
            float: Steering angle in degrees
        """
        if self.path_complete or not self.current_path:
            return 0
        
        # Get current and target node coordinates
        current_node_id = self.current_node or self.current_path[0]
        target_node_id = self.current_path[self.target_node_index]
        
        current_x, current_y = self.current_position
        target_x = float(self.G.nodes[target_node_id]['x'])
        target_y = float(self.G.nodes[target_node_id]['y'])
        
        # Check if this is an intersection node
        is_intersection = current_node_id in self.intersection_nodes
        if is_intersection:
            return self.handle_intersection_navigation(current_node_id, target_node_id, speed_factor)
        
        # Calculate direction vector
        dx = target_x - current_x
        dy = target_y - current_y
        
        # Calculate target angle in degrees
        target_angle = math.degrees(math.atan2(dy, dx))
        
        # Normalize to range [0, 360)
        if target_angle < 0:
            target_angle += 360
        
        # Calculate the difference between current yaw and target angle
        yaw = self.current_yaw
        if yaw < 0:
            yaw += 360
        
        angle_diff = target_angle - yaw
        
        # Normalize to range [-180, 180)
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
        
        # Calculate steering angle with P controller
        steering_angle = angle_diff * self.steering_gain * speed_factor
        
        # Limit steering angle
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        
        self.logger.debug(f"Steering angle: {steering_angle:.2f}° (Target: {target_angle:.2f}°, Current: {yaw:.2f}°)")
        
        return steering_angle
    
    def handle_intersection_navigation(self, current_node, target_node, speed_factor):
        """
        Handle navigation through intersections.
        
        Args:
            current_node (str): Current node ID
            target_node (str): Target node ID
            speed_factor (float): Speed factor
            
        Returns:
            float: Steering angle in degrees
        """
        # Get the path indices
        current_idx = self.current_path.index(current_node)
        target_idx = self.current_path.index(target_node)
        
        # Get node after target (for determining turn direction)
        next_idx = target_idx + 1
        next_node = self.current_path[next_idx] if next_idx < len(self.current_path) else None
        
        if next_node is None:
            return 0  # No need to turn if we're at the end
        
        # Get coordinates
        current_x = float(self.G.nodes[current_node]['x'])
        current_y = float(self.G.nodes[current_node]['y'])
        target_x = float(self.G.nodes[target_node]['x'])
        target_y = float(self.G.nodes[target_node]['y'])
        next_x = float(self.G.nodes[next_node]['x'])
        next_y = float(self.G.nodes[next_node]['y'])
        
        # Calculate vectors
        v1 = (target_x - current_x, target_y - current_y)
        v2 = (next_x - target_x, next_y - target_y)
        
        # Calculate angle between vectors
        dot_product = v1[0]*v2[0] + v1[1]*v2[1]
        mag1 = math.sqrt(v1[0]**2 + v1[1]**2)
        mag2 = math.sqrt(v2[0]**2 + v2[1]**2)
        
        # Avoid division by zero
        if mag1 * mag2 == 0:
            angle = 0
        else:
            angle = math.acos(dot_product/(mag1*mag2))
            angle = math.degrees(angle)
        
        # Determine turn direction (left or right)
        cross_product = v1[0]*v2[1] - v1[1]*v2[0]
        turn_direction = "left" if cross_product > 0 else "right"
        
        # Calculate steering based on turn direction and angle
        if turn_direction == "left":
            self.turning_left = True
            steering_factor = -1.0
        else:
            self.turning_left = False
            steering_factor = 1.0
        
        # Scale the steering angle based on the sharpness of the turn
        turn_sharpness = min(1.0, angle / 90.0)
        steering_angle = steering_factor * turn_sharpness * self.max_steering_angle * speed_factor
        
        self.logger.debug(f"Intersection turn: {turn_direction}, angle: {angle:.2f}°, steering: {steering_angle:.2f}°")
        
        return steering_angle
    
    def is_at_intersection(self):
        """
        Check if the vehicle is currently at an intersection.
        
        Returns:
            bool: True if at intersection, False otherwise
        """
        if not self.current_path or self.path_complete:
            return False
        
        # Check if current or next node is an intersection
        lookahead = 2  # Look ahead this many nodes
        for i in range(lookahead):
            idx = self.target_node_index + i
            if idx < len(self.current_path) and self.current_path[idx] in self.intersection_nodes:
                return True
        
        return False
    
    def is_at_roundabout(self):
        """
        Check if the vehicle is currently at a roundabout.
        
        Returns:
            bool: True if at roundabout, False otherwise
        """
        if not self.current_path or self.path_complete:
            return False
        
        # Check if current or next node is a roundabout entry
        lookahead = 2  # Look ahead this many nodes
        for i in range(lookahead):
            idx = self.target_node_index + i
            if idx < len(self.current_path) and self.current_path[idx] in self.roundabout_entry_nodes:
                return True
        
        return False
    
    def update_path_from_position(self, x, y):
        """
        Update the current path based on the current position.
        
        Args:
            x (float): X coordinate
            y (float): Y coordinate
            
        Returns:
            bool: True if path was updated, False otherwise
        """
        if not self.current_path:
            return False
        
        # Find the closest node to current position
        closest_node = self.find_closest_node(x, y)
        
        # Check if the closest node is in our path
        if closest_node in self.current_path:
            closest_idx = self.current_path.index(closest_node)
            
            # Check if we need to update our position in the path
            if closest_idx > self.target_node_index:
                self.logger.info(f"Updating path position from node {self.current_path[self.target_node_index]} to {closest_node}")
                self.target_node_index = closest_idx
                self.current_node = closest_node
                
                # Update target node
                if self.target_node_index + 1 < len(self.current_path):
                    self.target_node = self.current_path[self.target_node_index + 1]
                else:
                    self.target_node = closest_node
                    self.path_complete = True
                
                return True
        
        return False
    
    def get_direction_to_target(self):
        """
        Get the direction from current position to target node.
        
        Returns:
            str: Direction as 'left', 'right', 'forward', or 'backward'
        """
        if self.path_complete or not self.current_path:
            return "forward"
        
        target_node = self.current_path[self.target_node_index]
        target_x = float(self.G.nodes[target_node]['x'])
        target_y = float(self.G.nodes[target_node]['y'])
        
        current_x, current_y = self.current_position
        
        # Calculate vector to target
        dx = target_x - current_x
        dy = target_y - current_y
        
        # Calculate angle to target in degrees
        target_angle = math.degrees(math.atan2(dy, dx))
        if target_angle < 0:
            target_angle += 360
        
        # Calculate the difference between current yaw and target angle
        yaw = self.current_yaw
        if yaw < 0:
            yaw += 360
        
        angle_diff = target_angle - yaw
        
        # Normalize to range [-180, 180)
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
        
        # Determine direction
        if abs(angle_diff) < 45:
            return "forward"
        elif abs(angle_diff) > 135:
            return "backward"
        elif angle_diff > 0:
            return "left"
        else:
            return "right"
    
    def get_progress_percentage(self):
        """
        Get the progress percentage along the current path.
        
        Returns:
            float: Progress percentage (0-100)
        """
        if not self.current_path:
            return 0
        
        if self.path_complete:
            return 100
        
        return (self.target_node_index / len(self.current_path)) * 100
    
    def get_remaining_distance(self):
        """
        Estimate the remaining distance to the end of the path.
        
        Returns:
            float: Remaining distance in meters
        """
        if self.path_complete or not self.current_path:
            return 0
        
        remaining_distance = 0
        
        # Current position to next node
        current_x, current_y = self.current_position
        next_node = self.current_path[self.target_node_index]
        next_x = float(self.G.nodes[next_node]['x'])
        next_y = float(self.G.nodes[next_node]['y'])
        
        remaining_distance += distance.euclidean((current_x, current_y), (next_x, next_y))
        
        # Sum distances between remaining nodes
        for i in range(self.target_node_index, len(self.current_path) - 1):
            current_node = self.current_path[i]
            next_node = self.current_path[i + 1]
            
            current_x = float(self.G.nodes[current_node]['x'])
            current_y = float(self.G.nodes[current_node]['y'])
            next_x = float(self.G.nodes[next_node]['x'])
            next_y = float(self.G.nodes[next_node]['y'])
            
            remaining_distance += distance.euclidean((current_x, current_y), (next_x, next_y))
        
        return remaining_distance
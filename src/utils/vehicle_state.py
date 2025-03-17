import time
from enum import Enum

# Import ParkingStates class from parking_handler
from src.parking_handler.parking_handler import ParkingStates

class VehicleState:
    def __init__(self):
        # Basic attributes
        self.current_speed = 0
        self.target_speed = 0
        self.current_lane_type = 'dashed'
        self.steering_angle = 0
        self.zone_type = 'city'  # 'city' or 'highway'
        
        # Traffic state
        self.stop_timer = None
        self.last_state = None
        self.in_roundabout = False
        
        # Parking attributes
        self.parking_mode = False
        self.parking_state = None  # Uses ParkingStates enum
        self.target_spot = None
        self.current_angle = 0  # Current vehicle angle (degrees)
        self.distance_traveled = 0  # Distance traveled in parking mode (mm)
        
        # Statistics - for tracking overall performance
        self.total_distance = 0  # Total distance traveled (m)
        self.start_time = time.time()  # Start time (seconds)
        
        # Time tracking - for internal calculations
        self.last_update_time = time.time()
        
    def update_distance(self, delta_time_ms):
        """
        Update distance traveled based on current speed
        
        This method tracks two types of distance:
        1. total_distance: Total distance traveled (m) - independent of parking state
        2. distance_traveled: Distance traveled in parking mode (mm) - only updated when in parking mode
        
        Args:
            delta_time_ms: Time elapsed in milliseconds
        """
        try:
            # Check input parameter to avoid division by zero or negative values
            if delta_time_ms <= 0:
                print(f"Warning: Invalid delta_time value: {delta_time_ms}")
                return
                
            # Convert time from milliseconds to seconds for calculations
            delta_time_s = delta_time_ms / 1000.0
            
            # Calculate distance traveled from speed (mm/s) and time (s)
            # Use abs(speed) because distance is magnitude, not dependent on direction
            distance_m = abs(self.current_speed) * delta_time_s / 1000.0  # Convert from mm/s to m/s
            
            # Update total distance traveled (m)
            self.total_distance += distance_m
            
            # If in parking mode, update the specific distance traveled (mm)
            if self.parking_state is not None:
                # Distance in parking mode is calculated in mm
                # No division by 1000 because current_speed is already mm/s
                distance_mm = abs(self.current_speed) * delta_time_s
                self.distance_traveled += distance_mm
                
            # Update last time for next calculation
            self.last_update_time = time.time()
            
        except Exception as e:
            print(f"Error in update_distance: {e}")
        
    def reset_parking_state(self):
        """
        Reset parking state and related variables
        """
        self.parking_mode = False
        self.parking_state = None
        self.target_spot = None
        self.distance_traveled = 0
        self.current_angle = 0
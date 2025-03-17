import re
import time
import numpy as np
import logging
from math import sin, cos, radians

class IMUProcessor:
    """
    Processes IMU data for localization and path planning.
    
    This class handles parsing IMU data, calculating position changes, 
    and integrating IMU data for position estimation.
    """
    def __init__(self):
        """Initialize the IMU processor."""
        self.logger = logging.getLogger('IMUProcessor')
        
        # IMU state
        self.roll = 0.0  # deg
        self.pitch = 0.0  # deg
        self.yaw = 0.0  # deg
        self.accel_x = 0.0  # m/s²
        self.accel_y = 0.0  # m/s²
        self.accel_z = 0.0  # m/s²
        
        # Position tracking
        self.position_x = 0.0  # meters
        self.position_y = 0.0  # meters
        self.velocity_x = 0.0  # m/s
        self.velocity_y = 0.0  # m/s
        self.total_distance = 0.0  # meters
        
        # Timing
        self.last_update_time = time.time()
        
        # Calibration and filtering
        self.accel_bias_x = 0.0
        self.accel_bias_y = 0.0
        self.calibration_samples = []
        self.is_calibrated = False
        self.alpha = 0.2  # Low-pass filter coefficient
        
    def parse_imu_data(self, data):
        """
        Parse IMU data from string format.
        
        Example format: @imu:roll,pitch,yaw,accel_x,accel_y,accel_z\r\n
        
        Args:
            data (str): IMU data string
            
        Returns:
            bool: True if data was successfully parsed, False otherwise
        """
        try:
            # Match the IMU data format using regex
            match = re.search(r'@imu:([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)', data)
            if match:
                # Extract values
                self.roll = float(match.group(1))
                self.pitch = float(match.group(2))
                self.yaw = float(match.group(3))
                self.accel_x = float(match.group(4))
                self.accel_y = float(match.group(5))
                self.accel_z = float(match.group(6))
                
                # Apply low-pass filter
                self.accel_x = self.alpha * self.accel_x + (1 - self.alpha) * self.accel_x
                self.accel_y = self.alpha * self.accel_y + (1 - self.alpha) * self.accel_y
                
                return True
            return False
        except Exception as e:
            self.logger.error(f"Error parsing IMU data: {e}")
            return False
            
    def calibrate(self, num_samples=100):
        """
        Calibrate the IMU by calculating bias.
        
        Args:
            num_samples (int): Number of samples to use for calibration
            
        Returns:
            bool: True if calibration was successful
        """
        self.logger.info(f"Starting IMU calibration with {num_samples} samples")
        self.calibration_samples = []
        
        # Vehicle should be stationary during calibration
        for _ in range(num_samples):
            if hasattr(self, 'last_imu_data') and self.last_imu_data:
                self.calibration_samples.append((self.accel_x, self.accel_y))
            time.sleep(0.01)
            
        if len(self.calibration_samples) > num_samples / 2:
            # Calculate average bias
            accel_x_values = [sample[0] for sample in self.calibration_samples]
            accel_y_values = [sample[1] for sample in self.calibration_samples]
            
            self.accel_bias_x = sum(accel_x_values) / len(accel_x_values)
            self.accel_bias_y = sum(accel_y_values) / len(accel_y_values)
            
            self.is_calibrated = True
            self.logger.info(f"IMU calibration complete. Bias: x={self.accel_bias_x:.4f}, y={self.accel_bias_y:.4f}")
            return True
        else:
            self.logger.warning("IMU calibration failed: not enough samples")
            return False
            
    def update_position(self):
        """
        Update position based on IMU data and time elapsed since last update.
        
        Returns:
            tuple: (distance_traveled, current_yaw)
        """
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Skip if time step is too large (indicates an issue)
        if dt > 0.5:  # Greater than 500ms
            return 0.0, self.yaw
            
        # Correct accelerations with calibration bias
        if self.is_calibrated:
            accel_x_corrected = self.accel_x - self.accel_bias_x
            accel_y_corrected = self.accel_y - self.accel_bias_y
        else:
            accel_x_corrected = self.accel_x
            accel_y_corrected = self.accel_y
            
        # Apply dead zone to acceleration to reduce noise
        dead_zone = 0.05  # m/s²
        if abs(accel_x_corrected) < dead_zone:
            accel_x_corrected = 0.0
        if abs(accel_y_corrected) < dead_zone:
            accel_y_corrected = 0.0
            
        # Convert yaw to global coordinate accelerations
        yaw_rad = radians(self.yaw)
        global_accel_x = accel_x_corrected * cos(yaw_rad) - accel_y_corrected * sin(yaw_rad)
        global_accel_y = accel_x_corrected * sin(yaw_rad) + accel_y_corrected * cos(yaw_rad)
        
        # Update velocity using acceleration and time
        self.velocity_x += global_accel_x * dt
        self.velocity_y += global_accel_y * dt
        
        # Simple velocity decay to prevent drift when not accelerating
        decay_factor = 0.95
        self.velocity_x *= decay_factor
        self.velocity_y *= decay_factor
        
        # Update position using velocity and time
        delta_x = self.velocity_x * dt
        delta_y = self.velocity_y * dt
        
        self.position_x += delta_x
        self.position_y += delta_y
        
        # Calculate distance traveled in this step
        distance_step = np.sqrt(delta_x**2 + delta_y**2)
        self.total_distance += distance_step
        
        return distance_step, self.yaw
        
    def reset_position(self):
        """Reset the position tracking."""
        self.position_x = 0.0
        self.position_y = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.total_distance = 0.0
        self.last_update_time = time.time()
        
    def get_position(self):
        """
        Get the current position and orientation.
        
        Returns:
            tuple: (x, y, yaw) - position in meters and orientation in degrees
        """
        return (self.position_x, self.position_y, self.yaw)
        
    def get_total_distance(self):
        """
        Get the total distance traveled.
        
        Returns:
            float: Total distance traveled in meters
        """
        return self.total_distance

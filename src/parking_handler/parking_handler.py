import time
import math
from enum import Enum

class ParkingStates(Enum):
    SEARCHING = 0
    APPROACHING = 1
    ALIGNING = 2
    REVERSING = 3
    ADJUSTING = 4
    PARKED = 5
    WAITING = 6
    LEAVING = 7

class ParkingHandler:
    def __init__(self):
        # Parking parameters based on image - Reduced speed for timely processing
        self.parking_params = {
            'spot_width': 765,   # mm 
            'spot_depth': 390,   # mm 
            'approach_speed': 150,  # mm/s
            'align_speed': 120,  # mm/s
            'reverse_speed': 100,  # mm/s
            'min_distance': 400,  # mm
            'final_angle': 0,    # degrees
            'wait_time': 3.0,    # seconds
            'exit_distance': 800, # mm 
        }
        # PID constants for steering control - Reduced coefficients for slower response
        self.pid_constants = {
            'Kp': 0.4,  
            'Ki': 0.05,
            'Kd': 0.15  
        }
        self.error_integral = 0
        self.last_error = 0
        self.wait_start_time = None
        self.last_log_time = time.time()
        self.leaving_started = False
        self.leaving_start_position = 0

    def process_parking(self, vehicle_state, detection, frame_width, frame_height):
        """
        Process parking logic based on current state and parking spot detection
        
        Returns:
            tuple: (action, speed, steering_angle)
        """
        try:
            current_time = time.time()
            
            if current_time - self.last_log_time > 1.0:
                print(f"Parking state: {vehicle_state.parking_state}")
                self.last_log_time = current_time

            if vehicle_state.parking_state == ParkingStates.WAITING:
                if current_time - self.wait_start_time >= self.parking_params['wait_time']:
                    vehicle_state.parking_state = ParkingStates.LEAVING
                    self.leaving_started = True
                    self.leaving_start_position = vehicle_state.distance_traveled
                    print("Wait time completed, starting to leave parking spot")
                    return 'leaving', self.parking_params['approach_speed'], 0
                return 'waiting', 0, 0

            if detection is None:
                action, speed, steering = self._handle_no_detection(vehicle_state)
                return action, speed, steering

            x1, y1, x2, y2 = detection[:4]
            spot_center_x = (x1 + x2) / 2
            spot_width = x2 - x1
            relative_distance = y2 / frame_height

            if vehicle_state.parking_state == ParkingStates.SEARCHING:
                return self._handle_searching(vehicle_state, spot_center_x, spot_width, relative_distance, frame_width)
            elif vehicle_state.parking_state == ParkingStates.APPROACHING:
                return self._handle_approaching(vehicle_state, spot_center_x, relative_distance, frame_width)
            elif vehicle_state.parking_state == ParkingStates.ALIGNING:
                return self._handle_aligning(vehicle_state, spot_center_x, relative_distance)
            elif vehicle_state.parking_state == ParkingStates.REVERSING:
                return self._handle_reversing(vehicle_state, spot_center_x, relative_distance)
            elif vehicle_state.parking_state == ParkingStates.ADJUSTING:
                return self._handle_adjusting(vehicle_state, spot_center_x, relative_distance)
            elif vehicle_state.parking_state == ParkingStates.PARKED:
                vehicle_state.parking_state = ParkingStates.WAITING
                self.wait_start_time = current_time
                return 'waiting', 0, 0
            elif vehicle_state.parking_state == ParkingStates.LEAVING:
                return self._handle_leaving(vehicle_state, frame_width)
            else:
                return 'error', 0, 0

        except Exception as e:
            print(f"Parking error: {e}")
            return 'error', 0, 0

    def _handle_no_detection(self, vehicle_state):
        if vehicle_state.parking_state == ParkingStates.SEARCHING:
            return 'searching', self.parking_params['approach_speed'] * 0.8, 0
        elif vehicle_state.parking_state == ParkingStates.LEAVING:
            return self._handle_leaving(vehicle_state, None)
        return 'continue', vehicle_state.current_speed, 0

    def _handle_searching(self, vehicle_state, spot_center_x, spot_width, relative_distance, frame_width):
        # Ensure parking slot detection is large enough
        if spot_width >= self.parking_params['spot_width'] * 0.8:
            if 0.3 < relative_distance < 0.7:
                vehicle_state.parking_state = ParkingStates.APPROACHING
                vehicle_state.target_spot = {'center_x': spot_center_x, 'width': spot_width}
                print(f"Found parking spot, width: {spot_width:.1f}, center: {spot_center_x:.1f}")
                return 'approaching', self.parking_params['approach_speed'], 0
                
        error = frame_width / 2 - spot_center_x
        steering_angle = error * 0.015  # Reduced from 0.02 to 0.015 for less sensitivity
        return 'searching', self.parking_params['approach_speed'] * 0.8, steering_angle

    def _handle_approaching(self, vehicle_state, spot_center_x, relative_distance, frame_width):
        # Reduced distance threshold to transition to alignment state earlier
        if relative_distance > 0.75:  # Reduced from 0.8 to 0.75
            vehicle_state.parking_state = ParkingStates.ALIGNING
            print("Close to parking spot, aligning")
            return 'aligning', self.parking_params['align_speed'], 0
            
        target_x = vehicle_state.target_spot['center_x']
        error = target_x - spot_center_x
        self.error_integral += error
        derivative = error - self.last_error
        steering_angle = (self.pid_constants['Kp'] * error +
                          self.pid_constants['Ki'] * self.error_integral +
                          self.pid_constants['Kd'] * derivative)
        self.last_error = error
        
        # Limit steering angle to avoid excessive steering
        steering_angle = max(min(steering_angle, 25), -25)  # Reduced from 30 to 25
        
        return 'approaching', self.parking_params['approach_speed'], steering_angle

    def _handle_aligning(self, vehicle_state, spot_center_x, relative_distance):
        # Set target angle for reverse parking
        target_angle = 25  # Reduced from 30 to 25
        
        if abs(vehicle_state.current_angle - target_angle) < 5:
            vehicle_state.parking_state = ParkingStates.REVERSING
            print("Aligned, starting to reverse")
            return 'reversing', -self.parking_params['reverse_speed'], -target_angle
            
        steering_angle = self._calculate_align_angle(vehicle_state.current_angle, target_angle)
        
        vehicle_state.current_angle += steering_angle * 0.08  # Reduced from 0.1 to 0.08
        
        return 'aligning', self.parking_params['align_speed'], steering_angle

    def _handle_reversing(self, vehicle_state, spot_center_x, relative_distance):
        if relative_distance < 0.2:
            vehicle_state.parking_state = ParkingStates.ADJUSTING
            print("Reversed enough, adjusting position")
            return 'adjusting', self.parking_params['align_speed'] * 0.7, 0
            
        # Use negative angle when reversing to maintain vehicle direction
        steering_angle = -vehicle_state.current_angle * 0.4  # Reduced from 0.5 to 0.4
        
        return 'reversing', -self.parking_params['reverse_speed'], steering_angle

    def _handle_adjusting(self, vehicle_state, spot_center_x, relative_distance):
        if abs(vehicle_state.current_angle - self.parking_params['final_angle']) < 2:
            vehicle_state.parking_state = ParkingStates.PARKED
            print("Parking completed successfully")
            return 'parked', 0, 0
            
        steering_angle = self._calculate_adjust_angle(vehicle_state.current_angle, self.parking_params['final_angle'])
        
        vehicle_state.current_angle += steering_angle * 0.08  # Reduced from 0.1 to 0.08
        
        return 'adjusting', self.parking_params['align_speed'] * 0.4, steering_angle  # Reduced from 0.5 to 0.4

    def _handle_leaving(self, vehicle_state, frame_width):
        """
        Process leaving the parking spot
        
        Args:
            vehicle_state: Current vehicle state
            frame_width: Frame width (or None if no frame)
            
        Returns:
            tuple: (action, speed, steering_angle)
        """
        try:
            if not self.leaving_started:
                self.leaving_started = True
                self.leaving_start_position = vehicle_state.distance_traveled
                print("Started leaving parking spot")
            
            distance_moved = vehicle_state.distance_traveled - self.leaving_start_position
            print(f"Distance moved while leaving: {distance_moved:.2f} mm")
            
            if distance_moved >= self.parking_params['exit_distance']:
                print("Successfully left parking spot after moving", 
                      f"{distance_moved:.2f} mm. Returning to normal operation")
                
                self.leaving_started = False
                
                return 'proceed', self.parking_params['approach_speed'] * 0.8, 0
            
            steering_angle = 0
            
            if frame_width is not None:
                # TODO: Place for logic to adjust steering angle when leaving
                pass
                
            return 'leaving', self.parking_params['approach_speed'] * 0.7, steering_angle
            
        except Exception as e:
            print(f"Error while handling leaving: {e}")
            return 'leaving', self.parking_params['approach_speed'] * 0.6, 0

    def _calculate_align_angle(self, current_angle, target_angle):
        """Calculate necessary steering angle for alignment"""
        error = target_angle - current_angle
        steering_angle = max(min(error * 0.6, 25), -25)  # Reduced from 0.7/30 to 0.6/25
        return steering_angle

    def _calculate_adjust_angle(self, current_angle, target_angle):
        """Calculate necessary steering angle for final position adjustment"""
        error = target_angle - current_angle
        # Adjust slower to avoid overshooting
        steering_angle = max(min(error * 0.25, 12), -12)  # Reduced from 0.3/15 to 0.25/12
        return steering_angle
        
    def reset_parking_state(self):
        """
        Reset all states of the parking handler
        """
        self.error_integral = 0
        self.last_error = 0
        self.wait_start_time = None
        self.leaving_started = False
        self.leaving_start_position = 0
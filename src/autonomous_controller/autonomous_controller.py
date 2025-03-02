from car_controller.car_controller import CarController
from object_detector.object_detector import ObjectDetector
from lane_analyzer.lane_analyzer import LaneAnalyzer
from traffic_rule_processor.traffic_rule_processor import TrafficRuleProcessor
from parking_handler.parking_handler import ParkingHandler, ParkingStates
from utils.vehicle_state import VehicleState
from utils.lane_detection_config import LaneDetectionConfig
import time

class AutonomousController:
    def __init__(self, camera_width, camera_height):
        self.car = CarController()
        self.detector = ObjectDetector()
        self.lane_analyzer = LaneAnalyzer(camera_width, camera_height)
        self.traffic_processor = TrafficRuleProcessor()
        self.parking_handler = ParkingHandler()
        self.frame_width = camera_width
        self.frame_height = camera_height
        self.parking_mode = False
        self.parking_spot_detection = None

    def process_detection(self, detection, frame):
        try:
            if len(detection) < 6:
                return None, None
            x1, y1, x2, y2, conf, cls = detection
            state = self.car.state
            cls = int(cls)
            
            # Kiểm tra nếu đang ở chế độ đỗ xe
            if self.parking_mode and state.parking_state is not None:
                # Xử lý parking_spot nếu được phát hiện
                if cls == self.traffic_processor.CLASS_NAMES.index('parking-spot'):
                    self.parking_spot_detection = detection
                    action, speed, steering = self.parking_handler.process_parking(
                        state, detection, self.frame_width, self.frame_height)
                    
                    # Nếu quá trình đỗ xe đã kết thúc
                    if action == 'proceed':
                        self.exit_parking_mode()
                        print("Parking completed, returning to normal operation")
                        
                    return action, speed
                else:
                    # Nếu đang trong chế độ đỗ xe nhưng không phát hiện parking spot
                    # vẫn cần gọi xử lý đỗ xe với detection=None
                    action, speed, steering = self.parking_handler.process_parking(
                        state, None, self.frame_width, self.frame_height)
                    
                    if action == 'proceed':
                        self.exit_parking_mode()
                        print("Parking completed, returning to normal operation")
                        
                    return action, speed
                    
            # Xử lý các trường hợp thông thường
            if cls in [self.traffic_processor.CLASS_NAMES.index(x) for x in 
                      ['traffic-red', 'traffic-yellow', 'traffic-green']]:
                action, speed = self.traffic_processor.process_traffic_signal(cls, state)
            elif cls in [self.traffic_processor.CLASS_NAMES.index(x) for x in 
                        ['stop-sign', 'parking-sign', 'crosswalk-sign', 
                         'highway-entry-sign', 'highway-exit-sign', 'round-about-sign',
                         'priority-sign', 'no-entry-road-sign', 'one-way-road-sign']]:
                action, speed = self.traffic_processor.process_traffic_sign(cls, state)
                
                # Kích hoạt chế độ đỗ xe khi nhìn thấy biển báo đỗ xe
                if cls == self.traffic_processor.CLASS_NAMES.index('parking-sign') and not self.parking_mode:
                    self.enter_parking_mode()
                    print("Parking sign detected, entering parking mode")
                    
            elif cls == self.traffic_processor.CLASS_NAMES.index('pedestrian'):
                action, speed = self.traffic_processor.process_pedestrian(
                    detection, self.frame_height)
            elif cls == self.traffic_processor.CLASS_NAMES.index('car'):
                action, speed = self.traffic_processor.process_vehicle(
                    detection, state.current_lane_type, self.frame_width, self.frame_height)
            elif cls == self.traffic_processor.CLASS_NAMES.index('parking-spot'):
                self.parking_spot_detection = detection
                # Nếu đang ở chế độ tìm kiếm chỗ đậu xe
                if self.parking_mode and state.parking_state == ParkingStates.SEARCHING:
                    action, speed = 'searching', self.car.base_speed * 0.5
                else:
                    return None, None
            else:
                return None, None
                
            print(f"Detected {self.traffic_processor.CLASS_NAMES[cls]}: {action}, speed={speed}")
            return action, speed
        except Exception as e:
            print(f"Error processing detection: {e}")
            return None, None
            
    def enter_parking_mode(self):
        """
        Bật chế độ đỗ xe và khởi tạo các trạng thái liên quan
        """
        self.parking_mode = True
        self.car.state.parking_mode = True
        self.car.state.parking_state = ParkingStates.SEARCHING
        # Reset khoảng cách di chuyển
        self.car.state.distance_traveled = 0
        # Giảm tốc độ
        self.car.set_speed(int(self.car.base_speed * 0.5))
        
    def exit_parking_mode(self):
        """
        Tắt chế độ đỗ xe và reset các trạng thái liên quan
        """
        self.parking_mode = False
        self.car.state.reset_parking_state()
        self.parking_spot_detection = None
        
    def update_vehicle_state(self, action, speed, steering=None):
        try:
            # Cập nhật góc lái nếu có
            if steering is not None:
                self.car.set_steering(int(steering))
                
            # Xử lý các hành động
            if action == 'stop':
                self.car.brake()
            elif action in ['slow', 'caution']:
                if speed is not None:
                    self.car.set_speed(int(speed))
                else:
                    self.car.set_speed(int(self.car.base_speed * 0.7))
            elif action == 'proceed':
                if speed is not None:
                    self.car.set_speed(int(speed))
                else:
                    self.car.set_speed(self.car.base_speed)
                
                # Kiểm tra nếu đang trong chế độ đỗ xe và nhận lệnh proceed
                # Nghĩa là quá trình đỗ xe đã hoàn thành
                if self.parking_mode:
                    self.parking_mode = False
                    self.car.state.reset_parking_state()
                    print("Parking completed, returning to normal mode")
                    
            elif action == 'parking':
                # Kích hoạt chế độ đỗ xe
                self.parking_mode = True
                self.car.state.parking_state = ParkingStates.SEARCHING
                print("Entering parking mode")
                
                if speed is not None:
                    self.car.set_speed(int(speed))
                else:
                    self.car.set_speed(int(self.car.base_speed * 0.5))
                    
            elif action == 'follow':
                if speed is not None:
                    self.car.set_speed(int(speed))
                else:
                    self.car.set_speed(int(self.car.base_speed * 0.7))
                    
            elif action == 'overtake':
                if speed is not None:
                    self.car.set_speed(int(speed))
                else:
                    self.car.set_speed(self.car.base_speed)
                    
            # Xử lý các hành động cho parking
            elif action in ['searching', 'approaching', 'aligning', 'reversing', 
                          'adjusting', 'waiting', 'leaving']:
                print(f"Parking action: {action}, speed: {speed}, steering: {steering}")
                
                # Xử lý tốc độ âm (lùi xe)
                if speed is not None:
                    if speed < 0:
                        if not self.car.reversing:
                            self.car.set_reversing(True)
                        self.car.set_speed(abs(int(speed)))
                    else:
                        if self.car.reversing:
                            self.car.set_reversing(False)
                        self.car.set_speed(int(speed))
                else:
                    # Nếu không có tốc độ được chỉ định, sử dụng tốc độ mặc định
                    # dựa vào hành động (tiến hay lùi)
                    if action == 'reversing':
                        if not self.car.reversing:
                            self.car.set_reversing(True)
                        self.car.set_speed(int(self.car.base_speed * 0.4))
                    else:
                        if self.car.reversing:
                            self.car.set_reversing(False)
                        self.car.set_speed(int(self.car.base_speed * 0.5))        
            elif action == 'parked':
                self.car.brake()
                print("Vehicle successfully parked")
                
            elif action == 'error':
                # Xử lý lỗi trong quá trình đỗ xe
                print("Error in parking process, stopping vehicle")
                self.car.brake()
                
        except Exception as e:
            print(f"Error updating vehicle state: {e}")
            # Fallback to safe behavior
            self.car.set_speed(self.car.base_speed)

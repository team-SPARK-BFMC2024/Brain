import time
from enum import Enum

# Import lớp ParkingStates từ parking_handler
from src.parking_handler.parking_handler import ParkingStates

class VehicleState:
    def __init__(self):
        # Thuộc tính cơ bản
        self.current_speed = 0
        self.target_speed = 0
        self.current_lane_type = 'dashed'
        self.steering_angle = 0
        self.zone_type = 'city'  # 'city' hoặc 'highway'
        
        # Trạng thái giao thông
        self.stop_timer = None
        self.last_state = None
        self.in_roundabout = False
        
        # Thuộc tính cho quá trình đỗ xe
        self.parking_mode = False
        self.parking_state = None  # Sử dụng ParkingStates enum
        self.target_spot = None
        self.current_angle = 0  # Góc hiện tại của xe (độ)
        self.distance_traveled = 0  # Khoảng cách di chuyển trong chế độ đỗ xe (mm)
        
        # Số liệu thống kê
        self.total_distance = 0  # Tổng khoảng cách di chuyển (m)
        self.start_time = time.time()  # Thời gian bắt đầu (giây)
        
        # Biến theo dõi thời gian
        self.last_update_time = time.time()
        
    def update_distance(self, delta_time_ms):
        """
        Cập nhật khoảng cách di chuyển dựa trên tốc độ hiện tại
        
        Phương thức này theo dõi hai loại khoảng cách:
        1. total_distance: Tổng khoảng cách di chuyển (m) - không phụ thuộc vào trạng thái đỗ xe
        2. distance_traveled: Khoảng cách di chuyển trong chế độ đỗ xe (mm) - chỉ được cập nhật khi đang trong chế độ đỗ xe
        
        Args:
            delta_time_ms: Thời gian trôi qua tính bằng mili giây
        """
        try:
            # Kiểm tra tham số đầu vào
            if delta_time_ms <= 0:
                print(f"Warning: Invalid delta_time value: {delta_time_ms}")
                return
                
            # Chuyển đổi thời gian từ mili giây sang giây để tính toán
            delta_time_s = delta_time_ms / 1000.0
            
            # Tính toán khoảng cách di chuyển từ tốc độ (mm/s) và thời gian (s)
            # Sử dụng abs(speed) vì khoảng cách là độ lớn, không phụ thuộc vào hướng
            distance_m = abs(self.current_speed) * delta_time_s / 1000.0  # Chuyển từ mm/s sang m/s
            
            # Cập nhật tổng khoảng cách di chuyển (m)
            self.total_distance += distance_m
            
            # Nếu đang trong chế độ đỗ xe, cập nhật khoảng cách di chuyển riêng (mm)
            if self.parking_state is not None:
                # Khoảng cách trong chế độ đỗ xe được tính bằng mm
                # Không chia cho 1000 vì current_speed đã là mm/s
                distance_mm = abs(self.current_speed) * delta_time_s
                self.distance_traveled += distance_mm
                
            # Cập nhật thời gian cuối cùng
            self.last_update_time = time.time()
            
        except Exception as e:
            print(f"Error in update_distance: {e}")
        
    def reset_parking_state(self):
        """
        Reset trạng thái đỗ xe và các biến liên quan
        """
        self.parking_mode = False
        self.parking_state = None
        self.target_spot = None
        self.distance_traveled = 0
        self.current_angle = 0

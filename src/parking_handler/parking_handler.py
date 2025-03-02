import time
import math
from enum import Enum

# Định nghĩa các trạng thái
class ParkingStates(Enum):
    SEARCHING = 0   # Tìm chỗ đậu
    APPROACHING = 1 # Tiếp cận chỗ đậu
    ALIGNING = 2    # Căn chỉnh góc
    REVERSING = 3   # Lùi vào chỗ đậu
    ADJUSTING = 4   # Điều chỉnh vị trí
    PARKED = 5      # Đã đỗ
    WAITING = 6     # Chờ 3 giây
    LEAVING = 7     # Ra khỏi chỗ đậu

class ParkingHandler:
    def __init__(self):
        # Các thông số đỗ xe dựa trên hình ảnh - Giảm tốc độ để đảm bảo xử lý kịp thời
        self.parking_params = {
            'spot_width': 765,   # mm (76.5 cm)
            'spot_depth': 390,   # mm (39 cm)
            'approach_speed': 150,  # Giảm từ 250 xuống 150 mm/s
            'align_speed': 120,  # Giảm từ 150 xuống 120 mm/s
            'reverse_speed': 100,  # Giảm từ 150 xuống 100 mm/s
            'min_distance': 400,  # mm
            'final_angle': 0,    # độ
            'wait_time': 3.0,    # giây
            'exit_distance': 800, # mm - khoảng cách để coi như đã ra khỏi chỗ đậu
        }
        # Hằng số PID để điều khiển góc lái - Giảm hệ số để phản ứng chậm hơn
        self.pid_constants = {
            'Kp': 0.4,  # Giảm từ 0.5 xuống 0.4
            'Ki': 0.05, # Giảm từ 0.1 xuống 0.05
            'Kd': 0.15  # Giảm từ 0.2 xuống 0.15
        }
        self.error_integral = 0
        self.last_error = 0
        self.wait_start_time = None  # Thời gian bắt đầu chờ
        self.last_log_time = time.time()  # Thời gian log cuối cùng
        # Theo dõi tiến độ rời khỏi chỗ đậu
        self.leaving_started = False
        self.leaving_start_position = 0

    def process_parking(self, vehicle_state, detection, frame_width, frame_height):
        """
        Xử lý logic đỗ xe dựa trên trạng thái hiện tại và phát hiện chỗ đậu
        
        Returns:
            tuple: (action, speed, steering_angle)
        """
        try:
            current_time = time.time()
            
            # Log không quá thường xuyên để tránh spam
            if current_time - self.last_log_time > 1.0:
                print(f"Parking state: {vehicle_state.parking_state}")
                self.last_log_time = current_time

            # Xử lý trạng thái WAITING (chờ 3 giây)
            if vehicle_state.parking_state == ParkingStates.WAITING:
                if current_time - self.wait_start_time >= self.parking_params['wait_time']:
                    vehicle_state.parking_state = ParkingStates.LEAVING
                    self.leaving_started = True
                    # Reset distance_traveled để theo dõi quá trình ra khỏi chỗ đậu
                    self.leaving_start_position = vehicle_state.distance_traveled
                    print("Wait time completed, starting to leave parking spot")
                    return 'leaving', self.parking_params['approach_speed'], 0
                return 'waiting', 0, 0  # Dừng xe trong khi chờ

            # Nếu không phát hiện chỗ đậu
            if detection is None:
                action, speed, steering = self._handle_no_detection(vehicle_state)
                return action, speed, steering

            # Lấy thông tin từ camera
            x1, y1, x2, y2 = detection[:4]
            spot_center_x = (x1 + x2) / 2
            spot_width = x2 - x1
            relative_distance = y2 / frame_height

            # Xử lý từng trạng thái
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
            # Giảm tốc độ tìm kiếm
            return 'searching', self.parking_params['approach_speed'] * 0.8, 0
        elif vehicle_state.parking_state == ParkingStates.LEAVING:
            # Nếu đã bắt đầu rời đi nhưng không thấy chỗ đậu, tiếp tục di chuyển
            return self._handle_leaving(vehicle_state, None)
        return 'continue', vehicle_state.current_speed, 0

    def _handle_searching(self, vehicle_state, spot_center_x, spot_width, relative_distance, frame_width):
        # Đảm bảo phát hiện parking slot đủ lớn
        if spot_width >= self.parking_params['spot_width'] * 0.8:
            if 0.3 < relative_distance < 0.7:
                vehicle_state.parking_state = ParkingStates.APPROACHING
                vehicle_state.target_spot = {'center_x': spot_center_x, 'width': spot_width}
                print(f"Found parking spot, width: {spot_width:.1f}, center: {spot_center_x:.1f}")
                return 'approaching', self.parking_params['approach_speed'], 0
                
        # Nếu không tìm thấy chỗ đậu phù hợp, tiếp tục tìm kiếm với tốc độ chậm
        error = frame_width / 2 - spot_center_x
        steering_angle = error * 0.015  # Giảm từ 0.02 xuống 0.015 để xe không quá nhạy
        return 'searching', self.parking_params['approach_speed'] * 0.8, steering_angle

    def _handle_approaching(self, vehicle_state, spot_center_x, relative_distance, frame_width):
        # Giảm ngưỡng khoảng cách để chuyển sang trạng thái căn chỉnh sớm hơn
        if relative_distance > 0.75:  # Giảm từ 0.8 xuống 0.75
            vehicle_state.parking_state = ParkingStates.ALIGNING
            print("Close to parking spot, aligning")
            return 'aligning', self.parking_params['align_speed'], 0
            
        # Điều chỉnh hướng để tiếp cận chỗ đậu
        target_x = vehicle_state.target_spot['center_x']
        error = target_x - spot_center_x
        self.error_integral += error
        derivative = error - self.last_error
        steering_angle = (self.pid_constants['Kp'] * error +
                          self.pid_constants['Ki'] * self.error_integral +
                          self.pid_constants['Kd'] * derivative)
        self.last_error = error
        
        # Giới hạn góc lái để không quá lớn
        steering_angle = max(min(steering_angle, 25), -25)  # Giảm từ 30 xuống 25
        
        return 'approaching', self.parking_params['approach_speed'], steering_angle

    def _handle_aligning(self, vehicle_state, spot_center_x, relative_distance):
        # Đặt góc lái mục tiêu để chuẩn bị cho việc lùi vào chỗ đậu
        target_angle = 25  # Giảm từ 30 xuống 25
        
        # Nếu góc hiện tại đã gần với góc mục tiêu
        if abs(vehicle_state.current_angle - target_angle) < 5:
            vehicle_state.parking_state = ParkingStates.REVERSING
            print("Aligned, starting to reverse")
            return 'reversing', -self.parking_params['reverse_speed'], -target_angle
            
        # Tính toán góc lái cần thiết để đạt góc căn chỉnh mục tiêu
        steering_angle = self._calculate_align_angle(vehicle_state.current_angle, target_angle)
        
        # Cập nhật góc hiện tại của xe (trong thực tế, cần phải đọc từ cảm biến)
        vehicle_state.current_angle += steering_angle * 0.08  # Giảm từ 0.1 xuống 0.08
        
        return 'aligning', self.parking_params['align_speed'], steering_angle

    def _handle_reversing(self, vehicle_state, spot_center_x, relative_distance):
        # Nếu xe đã lùi đủ xa
        if relative_distance < 0.2:
            vehicle_state.parking_state = ParkingStates.ADJUSTING
            print("Reversed enough, adjusting position")
            return 'adjusting', self.parking_params['align_speed'] * 0.7, 0
            
        # Lái xe lùi với góc phù hợp để đi vào chỗ đậu
        # Sử dụng góc âm khi lùi để duy trì hướng xe
        steering_angle = -vehicle_state.current_angle * 0.4  # Giảm từ 0.5 xuống 0.4
        
        return 'reversing', -self.parking_params['reverse_speed'], steering_angle

    def _handle_adjusting(self, vehicle_state, spot_center_x, relative_distance):
        # Điều chỉnh vị trí cuối cùng
        if abs(vehicle_state.current_angle - self.parking_params['final_angle']) < 2:
            vehicle_state.parking_state = ParkingStates.PARKED
            print("Parking completed successfully")
            return 'parked', 0, 0
            
        # Tính toán góc lái cần để điều chỉnh thẳng hàng
        steering_angle = self._calculate_adjust_angle(vehicle_state.current_angle, self.parking_params['final_angle'])
        
        # Cập nhật góc hiện tại của xe
        vehicle_state.current_angle += steering_angle * 0.08  # Giảm từ 0.1 xuống 0.08
        
        return 'adjusting', self.parking_params['align_speed'] * 0.4, steering_angle  # Giảm từ 0.5 xuống 0.4

    def _handle_leaving(self, vehicle_state, frame_width):
        """
        Xử lý quá trình rời khỏi chỗ đậu xe
        
        Args:
            vehicle_state: Trạng thái hiện tại của xe
            frame_width: Chiều rộng khung hình (hoặc None nếu không có frame)
            
        Returns:
            tuple: (action, speed, steering_angle)
        """
        try:
            # Nếu chưa bắt đầu rời đi, thiết lập điểm bắt đầu
            if not self.leaving_started:
                self.leaving_started = True
                self.leaving_start_position = vehicle_state.distance_traveled
                print("Started leaving parking spot")
            
            # Tính khoảng cách đã di chuyển kể từ khi bắt đầu rời đi
            distance_moved = vehicle_state.distance_traveled - self.leaving_start_position
            print(f"Distance moved while leaving: {distance_moved:.2f} mm")
            
            # Kiểm tra xem đã di chuyển đủ xa chưa
            if distance_moved >= self.parking_params['exit_distance']:
                # Đã ra khỏi chỗ đậu, reset trạng thái
                print("Successfully left parking spot after moving", 
                      f"{distance_moved:.2f} mm. Returning to normal operation")
                
                # Reset các biến theo dõi rời đi
                self.leaving_started = False
                
                # Trả về 'proceed' để thông báo AutonomousController chuyển sang chế độ bình thường
                return 'proceed', self.parking_params['approach_speed'] * 0.8, 0  # Giảm tốc độ khi rời đi
            
            # Chưa ra khỏi chỗ đậu, tiếp tục di chuyển
            steering_angle = 0
            
            # Nếu có frame_width, có thể thực hiện điều khiển phức tạp hơn
            if frame_width is not None:
                # TODO: Đây là nơi có thể thêm logic để điều chỉnh góc lái khi rời đi
                pass
                
            return 'leaving', self.parking_params['approach_speed'] * 0.7, steering_angle  # Giảm tốc độ khi rời đi
            
        except Exception as e:
            print(f"Error while handling leaving: {e}")
            # Trong trường hợp lỗi, vẫn tiếp tục rời đi với tốc độ an toàn
            return 'leaving', self.parking_params['approach_speed'] * 0.6, 0

    def _calculate_align_angle(self, current_angle, target_angle):
        """Tính toán góc lái cần thiết để căn chỉnh"""
        error = target_angle - current_angle
        # Giới hạn góc lái
        steering_angle = max(min(error * 0.6, 25), -25)  # Giảm từ 0.7/30 xuống 0.6/25
        return steering_angle

    def _calculate_adjust_angle(self, current_angle, target_angle):
        """Tính toán góc lái cần thiết để điều chỉnh vị trí cuối cùng"""
        error = target_angle - current_angle
        # Điều chỉnh chậm hơn để tránh quá mức
        steering_angle = max(min(error * 0.25, 12), -12)  # Giảm từ 0.3/15 xuống 0.25/12
        return steering_angle
        
    def reset_parking_state(self):
        """
        Reset tất cả các trạng thái của bộ xử lý đỗ xe
        """
        self.error_integral = 0
        self.last_error = 0
        self.wait_start_time = None
        self.leaving_started = False
        self.leaving_start_position = 0

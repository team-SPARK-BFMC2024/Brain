import serial
import time
from utils.vehicle_state import VehicleState

class CarController:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1  # Reduced timeout
        )
        time.sleep(1)  # Reduced initialization time
        self.state = VehicleState()
        self.min_speed = 100
        self.max_speed = 350
        self.base_speed = 200
        self.state.target_speed = self.base_speed
        self.last_command_time = time.time()
        self.command_interval = 0.02  # 50Hz max command rate
        self.reversing = False  # Trạng thái lùi xe
        self.reversing_transition_time = 0.5  # Thời gian chuyển đổi giữa tiến và lùi (giây)

    def set_power_state(self, state: int):
        """
        Set the power state of the vehicle
        """
        command = f"#kl:{state};;\r\n"
        return self.send_command(command)
        
    def send_command(self, command: str):
        """
        Optimized command sending with rate limiting
        """
        current_time = time.time()
        if current_time - self.last_command_time < self.command_interval:
            return None
            
        try:
            self.serial.write(command.encode('ascii'))
            self.last_command_time = current_time
            
            # Quick read with timeout
            response = self.serial.readline().decode().strip()
            return response if response else None
        except Exception as e:
            print(f"Command error: {e}")
            return None

    def set_speed(self, speed: int):
        """
        Optimized speed control with smoother transitions
        """
        if speed is None:
            speed = self.base_speed
            
        try:
            # Smooth speed transition
            current_speed = self.state.current_speed
            target_speed = max(min(int(speed), self.max_speed), self.min_speed)
            
            # Only send command if speed change is significant
            if abs(current_speed - target_speed) >= 2:
                self.state.current_speed = target_speed
                
                # Sử dụng trạng thái lùi xe hiện tại
                if self.reversing:
                    command = f"#reverse:{target_speed};;\r\n"
                else:
                    command = f"#speed:{target_speed};;\r\n"
                    
                return self.send_command(command)
            return None
        except (TypeError, ValueError):
            print(f"Invalid speed value: {speed}, using base speed")
            return self.set_speed(self.base_speed)

    def set_reversing(self, state: bool):
        """
        Thiết lập chế độ lùi xe và xử lý chuyển đổi an toàn giữa tiến và lùi
        
        Args:
            state: True để lùi xe, False để tiến
            
        Returns:
            bool: Trạng thái lùi xe hiện tại sau khi xử lý
        """
        # Nếu không thay đổi trạng thái, không cần làm gì thêm
        if state == self.reversing:
            return self.reversing
            
        print(f"Changing direction: {'Forward to Reverse' if state else 'Reverse to Forward'}")
        
        # Dừng xe trước khi chuyển đổi hướng
        self.brake()
        
        # Đợi để đảm bảo xe đã dừng hoàn toàn
        time.sleep(self.reversing_transition_time)
        
        # Cập nhật trạng thái lùi xe
        self.reversing = state
        
        # Nếu chuyển sang lùi, gửi lệnh với tốc độ 0 để khởi tạo chế độ lùi
        if self.reversing:
            command = "#reverse:0;;\r\n"
            self.send_command(command)
            print("Initialized reverse mode")
        else:
            # Khởi tạo lại chế độ tiến với tốc độ 0
            command = "#speed:0;;\r\n"
            self.send_command(command)
            print("Initialized forward mode")
            
        # Đảm bảo trạng thái hiện tại là đã dừng
        self.state.current_speed = 0
        
        return self.reversing

    def set_steering(self, angle: int):
        """
        Optimized steering control with reduced latency
        """
        # Only update if angle has changed significantly
        if abs(self.state.steering_angle - angle) < 2:
            return None
            
        self.state.steering_angle = angle
        steering_command = int(angle * 20)
        steering_command = max(min(steering_command, 230), -230)
        command = f"#steer:{steering_command};;\r\n"
        return self.send_command(command)

    def brake(self):
        """
        Emergency brake with highest priority
        """
        self.state.current_speed = 0
        command = "#brake:0;;\r\n"
        # Bypass normal command interval for emergency brake
        self.serial.write(command.encode('ascii'))
        return self.serial.readline().decode().strip()

    def reverse(self, speed: int):
        """
        Lùi xe với tốc độ nhất định sau khi đã thiết lập chế độ lùi
        
        Args:
            speed: Tốc độ lùi (giá trị dương)
        """
        if not self.reversing:
            print("Warning: Calling reverse() without setting reverse mode first")
            self.set_reversing(True)
            
        speed = abs(speed)  # Đảm bảo tốc độ là dương
        return self.set_speed(speed)
        
    def forward(self, speed: int):
        """
        Tiến xe với tốc độ nhất định sau khi đã thiết lập chế độ tiến
        
        Args:
            speed: Tốc độ tiến (giá trị dương)
        """
        if self.reversing:
            print("Warning: Calling forward() while in reverse mode")
            self.set_reversing(False)
            
        speed = abs(speed)  # Đảm bảo tốc độ là dương
        return self.set_speed(speed)

    def close(self):
        """
        Optimized shutdown sequence
        """
        self.brake()
        time.sleep(0.2)  # Reduced closing time
        self.set_power_state(0)
        self.serial.close()

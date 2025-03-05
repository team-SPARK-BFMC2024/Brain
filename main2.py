import cv2
import logging
import argparse
import os
import socket
import serial
import time
from src.lane_detection.lane_detection import LaneDetection
from src.lane_detection.lane_keeping import LaneKeeping
from src.system_runner.system_runner import run_autonomous_system

# Thiết lập logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger('Root logger')

# Cấu hình UART để nhận dữ liệu IMU
ser = serial.Serial("/dev/ttyTHS1", 115200, timeout=1)

def get_ip_address():
    """Lấy địa chỉ IP của thiết bị"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception as e:
        print(f"Không thể lấy địa chỉ IP: {e}")
        return "Unknown"

def read_imu_data():
    """Đọc dữ liệu từ IMU qua UART"""
    try:
        ser.write(b'@imu\n')  # Gửi yêu cầu dữ liệu
        data = ser.readline().decode().strip()
        if data.startswith("@imu:"):
            values = list(map(float, data[5:].split(',')))
            return values  # [roll, pitch, yaw, accel_x, accel_y, accel_z]
    except Exception as e:
        print(f"Lỗi đọc IMU: {e}")
    return None

def main():
    jetson_ip = get_ip_address()
    print(f"Jetson IP: {jetson_ip}")
    print(f"Laptop Server IP: 192.168.163.162")

    parser = argparse.ArgumentParser(description='Hệ thống xe tự hành với streaming')
    parser.add_argument('--model', type=str, default=None, help='Đường dẫn đến model YOLO (engine/pt)')
    parser.add_argument('--camera', type=int, default=0, help='Camera ID')
    parser.add_argument('--width', type=int, default=640, help='Chiều rộng camera')
    parser.add_argument('--height', type=int, default=480, help='Chiều cao camera')
    parser.add_argument('--server', type=str, default='192.168.163.162', help='IP của server streaming (default: 192.168.163.162)')
    parser.add_argument('--port', type=int, default=8089, help='Port của server streaming (default: 8089)')
    parser.add_argument('--no-stream', action='store_true', help='Không stream về server')
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        log.error("Could not open webcam")
        exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    try:
        ret, frame = cap.read()
        if not ret:
            raise Exception("Could not read initial frame")

        lk = LaneKeeping(frame.shape[1], frame.shape[0], log, "455")
        ld = LaneDetection(frame.shape[1], frame.shape[0], "455", lk)

        ld.square_pulses_min_height = 80
        ld.square_pulses_pix_dif = 10
        ld.square_pulses_min_height_dif = 20
        ld.square_pulses_allowed_peaks_width_error = 15

        server_ip = None
        server_port = None
        if not args.no_stream:
            server_ip = args.server
            server_port = args.port
            print(f"Streaming will be enabled to {server_ip}:{server_port}")
        else:
            print("Streaming is disabled")

        # Biến theo dõi node
        node_target = 30  # Node cuối cùng
        node_distance_cm = 37  # Khoảng cách giữa các node (cm)
        current_distance = 0  # Khoảng cách đã đi
        current_node = 1  # Node bắt đầu

        # Chạy hệ thống tự hành
        while current_node <= node_target:
            imu_data = read_imu_data()
            if imu_data:
                accel_x = imu_data[3]
                dt = 0.1  # Khoảng thời gian lấy mẫu (giả định 10Hz)
                distance_step = 0.5 * accel_x * dt * dt  # x = 1/2 * a * t^2
                current_distance += distance_step * 100  # Chuyển sang cm

                if current_distance >= node_distance_cm:
                    print(f"Xe đã đến node {current_node}")
                    current_node += 1
                    current_distance = 0  # Reset khoảng cách cho node tiếp theo

            time.sleep(0.1)

    except Exception as e:
        log.error(f"Error: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

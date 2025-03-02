import cv2
import logging
import argparse
import os
import socket
from lane_detection.lane_detection import LaneDetection
from lane_detection.lane_keeping import LaneKeeping
from system_runner.system_runner import run_autonomous_system

# Thiết lập logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger('Root logger')

def get_ip_address():
    """Lấy địa chỉ IP của thiết bị"""
    try:
        # Tạo socket để kiểm tra kết nối
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Không cần kết nối thật, chỉ cần để lấy địa chỉ IP
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception as e:
        print(f"Không thể lấy địa chỉ IP: {e}")
        return "Unknown"

def main():
    # Hiển thị thông tin địa chỉ IP
    jetson_ip = get_ip_address()
    print(f"Jetson IP: {jetson_ip}")
    print(f"Laptop Server IP: 192.168.163.162")
    
    # Tạo parser để xử lý tham số dòng lệnh
    parser = argparse.ArgumentParser(description='Hệ thống xe tự hành với streaming')
    parser.add_argument('--model', type=str, default=None, help='Đường dẫn đến model YOLO (engine/pt)')
    parser.add_argument('--camera', type=int, default=0, help='Camera ID')
    parser.add_argument('--width', type=int, default=640, help='Chiều rộng camera')
    parser.add_argument('--height', type=int, default=480, help='Chiều cao camera')
    parser.add_argument('--server', type=str, default='192.168.163.162', help='IP của server streaming (default: 192.168.163.162)')
    parser.add_argument('--port', type=int, default=8089, help='Port của server streaming (default: 8089)')
    parser.add_argument('--no-stream', action='store_true', help='Không stream về server')
    args = parser.parse_args()

    # Mở camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        log.error("Could not open webcam")
        exit(1)

    # Thiết lập độ phân giải camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    try:
        # Đọc frame đầu tiên để khởi tạo các đối tượng
        ret, frame = cap.read()
        if not ret:
            raise Exception("Could not read initial frame")

        # Khởi tạo đối tượng lane detection
        lk = LaneKeeping(frame.shape[1], frame.shape[0], log, "455")
        ld = LaneDetection(frame.shape[1], frame.shape[0], "455", lk)

        # Thiết lập các tham số nhận diện làn đường tối ưu 
        ld.square_pulses_min_height = 80
        ld.square_pulses_pix_dif = 10
        ld.square_pulses_min_height_dif = 20
        ld.square_pulses_allowed_peaks_width_error = 15

        # Xác định thông tin server
        server_ip = None
        server_port = None
        if not args.no_stream:
            server_ip = args.server
            server_port = args.port
            print(f"Streaming will be enabled to {server_ip}:{server_port}")
        else:
            print("Streaming is disabled")

        # Chạy hệ thống tự hành
        run_autonomous_system(cap, lk, ld, server_ip, server_port, model_path=args.model)

    except Exception as e:
        log.error(f"Error: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

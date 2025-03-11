import cv2
import logging
import argparse
import os
import socket
import json
import threading
import time
import numpy as np
from src.lane_detection.lane_detection import LaneDetection
from src.lane_detection.lane_keeping import LaneKeeping
from src.system_runner.system_runner import run_autonomous_system
from src.path_planning.path_planner import PathPlanner
from src.path_planning.controller import PathPlanningController
import websocket
from websocket import create_connection
import ssl

# Thiết lập logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger('Root logger')

# Biến toàn cục cho WebSocket
ws_client = None
ws_thread = None
telemetry_data = {
    "speed": 0,
    "steering": 0,
    "brake": False,
    "mode": "manual",
    "path_progress": 0,
    "current_node": "",
    "target_node": "",
    "position": [0, 0],
    "distance_traveled": 0
}
telemetry_lock = threading.Lock()

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

def websocket_thread(server_address):
    """Thread xử lý kết nối và gửi dữ liệu WebSocket"""
    global ws_client
    
    while True:
        try:
            # Kết nối đến server WebSocket
            log.info(f"Connecting to WebSocket server at {server_address}")
            ws_client = create_connection(server_address)
            log.info("WebSocket connection established")
            
            # Gửi dữ liệu telemetry
            while True:
                try:
                    with telemetry_lock:
                        data_to_send = json.dumps(telemetry_data)
                    ws_client.send(data_to_send)
                    time.sleep(0.1)  # Gửi dữ liệu mỗi 100ms
                except Exception as e:
                    log.error(f"Error sending data: {e}")
                    break
                    
        except Exception as e:
            log.error(f"WebSocket connection error: {e}")
            time.sleep(5)  # Thử kết nối lại sau 5 giây
            
            # Đóng kết nối cũ nếu còn tồn tại
            if ws_client:
                try:
                    ws_client.close()
                except:
                    pass
                ws_client = None

def update_telemetry(speed, steering, brake, mode, path_data=None):
    """Cập nhật dữ liệu telemetry để gửi qua WebSocket"""
    with telemetry_lock:
        telemetry_data["speed"] = speed
        telemetry_data["steering"] = steering
        telemetry_data["brake"] = brake
        telemetry_data["mode"] = mode
        
        if path_data:
            telemetry_data["path_progress"] = path_data.get("progress", 0)
            telemetry_data["current_node"] = path_data.get("current_node", "")
            telemetry_data["target_node"] = path_data.get("target_node", "")
            telemetry_data["position"] = path_data.get("position", [0, 0])
            telemetry_data["distance_traveled"] = path_data.get("distance_traveled", 0)

def main():
    # Hiển thị thông tin địa chỉ IP
    jetson_ip = get_ip_address()
    print(f"Jetson IP: {jetson_ip}")
    
    # Tạo parser để xử lý tham số dòng lệnh
    parser = argparse.ArgumentParser(description='Hệ thống xe tự hành với path planning và streaming')
    parser.add_argument('--model', type=str, default=None, help='Đường dẫn đến model YOLO (engine/pt)')
    parser.add_argument('--camera', type=int, default=0, help='Camera ID')
    parser.add_argument('--width', type=int, default=640, help='Chiều rộng camera')
    parser.add_argument('--height', type=int, default=480, help='Chiều cao camera')
    parser.add_argument('--server', type=str, default='192.168.163.162', help='IP của server streaming (default: 192.168.163.162)')
    parser.add_argument('--port', type=int, default=8089, help='Port của server streaming (default: 8089)')
    parser.add_argument('--ws-port', type=int, default=8090, help='Port của server WebSocket (default: 8090)')
    parser.add_argument('--no-stream', action='store_true', help='Không stream về server')
    parser.add_argument('--map', type=str, default='src/path_planning/converted_graph.graphml', help='Đường dẫn đến file map graph')
    parser.add_argument('--mode', type=str, default='lane_keeping', choices=['manual', 'lane_keeping', 'path_planning'], help='Chế độ điều khiển ban đầu')
    parser.add_argument('--start', type=str, help='ID của node bắt đầu cho path planning')
    parser.add_argument('--end', type=str, help='ID của node kết thúc cho path planning')
    parser.add_argument('--ws-server', type=str, default='0.0.0.0', help='IP của WebSocket server (default: 0.0.0.0)')
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
        
        # Kiểm tra file map tồn tại
        if not os.path.exists(args.map):
            log.error(f"Map file not found: {args.map}")
            log.info("Path planning will be disabled")
            path_planner = None
        else:
            # Khởi tạo đối tượng path planning
            log.info(f"Initializing path planning with map: {args.map}")
            path_planner = PathPlanningController(args.map, log)
            
            # Set mode path planning
            if args.mode == 'path_planning':
                if args.start and args.end:
                    log.info(f"Planning path from node {args.start} to {args.end}")
                    success = path_planner.set_path(args.start, args.end)
                    if success:
                        log.info("Path planned successfully")
                    else:
                        log.error("Failed to plan path, falling back to lane keeping mode")
                        args.mode = 'lane_keeping'
                else:
                    log.warning("Path planning mode selected but no start/end nodes provided")
                    log.info("Use --start and --end parameters to specify path")
                    args.mode = 'lane_keeping'

        # Khởi tạo WebSocket client
        ws_server_address = f"ws://{args.ws_server}:{args.ws_port}/telemetry"
        global ws_thread
        ws_thread = threading.Thread(target=websocket_thread, args=(ws_server_address,), daemon=True)
        ws_thread.start()
        log.info(f"WebSocket client thread started, connecting to {ws_server_address}")

        # Xác định thông tin server
        stream_server_ip = None
        stream_server_port = None
        if not args.no_stream:
            stream_server_ip = args.server
            stream_server_port = args.port
            print(f"Video streaming will be enabled to {stream_server_ip}:{stream_server_port}")
        else:
            print("Video streaming is disabled")

        # Chạy hệ thống tự hành với path planning
        run_autonomous_system_with_path_planning(
            cap, lk, ld, 
            stream_server_ip, stream_server_port, 
            path_planner, args.mode,
            model_path=args.model
        )

    except Exception as e:
        log.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cap.release()
        cv2.destroyAllWindows()
        
        # Đóng kết nối WebSocket
        if ws_client:
            try:
                ws_client.close()
            except:
                pass

def run_autonomous_system_with_path_planning(cap, lk, ld, server_ip, server_port, path_planner, initial_mode, model_path=None):
    """
    Hàm chạy hệ thống tự hành tích hợp với path planning.
    
    Parameters:
    -----------
    cap : cv2.VideoCapture
        Đối tượng camera capture
    lk : LaneKeeping
        Đối tượng lane keeping
    ld : LaneDetection
        Đối tượng lane detection
    server_ip : str
        Địa chỉ IP của server để stream video
    server_port : int
        Port của server để stream video
    path_planner : PathPlanningController
        Đối tượng điều khiển path planning
    initial_mode : str
        Chế độ điều khiển ban đầu ('manual', 'lane_keeping', 'path_planning')
    model_path : str
        Đường dẫn đến model YOLO (nếu có)
    """
    from src.system_runner.system_runner import initialize_system, process_frame
    
    # Khởi tạo các thông số từ hệ thống
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # Kiểm tra camera đã được mở chưa
    if not cap.isOpened():
        log.error("Camera not opened")
        return
    
    # Khởi tạo controller và các thành phần hệ thống
    controller, detector = initialize_system(frame_width, frame_height, model_path)
    
    # Thiết lập chế độ ban đầu
    current_mode = initial_mode
    
    # Khởi tạo cửa sổ hiển thị
    cv2.namedWindow("Autonomous System", cv2.WINDOW_NORMAL)
    
    # Biến theo dõi thời gian
    last_time = time.time()
    last_process_time = time.time()
    frame_count = 0
    
    # Socket để stream video (nếu cần)
    streaming_socket = None
    if server_ip and server_port:
        try:
            streaming_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            log.info(f"Video streaming enabled to {server_ip}:{server_port}")
        except Exception as e:
            log.error(f"Failed to create streaming socket: {e}")
            streaming_socket = None
    
    log.info("System initialized, starting main loop")
    print("Controls: q=Quit, l=Lane Keeping Mode, p=Path Planning Mode, m=Manual Mode")
    
    try:
        while True:
            # Đọc frame từ camera
            ret, frame = cap.read()
            if not ret:
                log.error("Failed to read frame from camera")
                break
            
            # Tính thời gian delta
            current_time = time.time()
            delta_time = current_time - last_time
            last_time = current_time
            
            # Cập nhật khoảng cách di chuyển
            controller.car.state.update_distance(delta_time * 1000)  # Convert to ms
            
            # Chỉ xử lý nếu đủ thời gian (30fps)
            if current_time - last_process_time < 0.033:
                # Xử lý phím
                key = cv2.waitKey(1)
                if key == ord('q'):
                    log.info("Quitting...")
                    break
                elif key == ord('l'):
                    current_mode = 'lane_keeping'
                    log.info("Switched to lane keeping mode")
                elif key == ord('p'):
                    if path_planner:
                        current_mode = 'path_planning'
                        log.info("Switched to path planning mode")
                    else:
                        log.warning("Path planning not available")
                elif key == ord('m'):
                    current_mode = 'manual'
                    log.info("Switched to manual mode")
                
                # Hiển thị frame hiện tại
                cv2.imshow("Autonomous System", frame)
                
                # Stream video nếu cần
                if streaming_socket:
                    try:
                        # Nén frame để giảm kích thước
                        _, img_encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                        # Gửi dữ liệu qua UDP
                        streaming_socket.sendto(img_encoded.tobytes(), (server_ip, server_port))
                    except Exception as e:
                        log.error(f"Streaming error: {e}")
                
                continue
            
            # Đánh dấu thời gian xử lý
            last_process_time = current_time
            frame_count += 1
            
            # Tạo bản sao của frame để hiển thị
            display_frame = frame.copy()
            
            # Xử lý theo mode hiện tại
            if current_mode == 'lane_keeping':
                # Thực hiện lane detection
                lane_results = ld.lanes_detection(frame.copy())
                
                # Xử lý frame với lane keeping và object detection
                steering_angle, speed, brake, overlaid_frame = process_frame(
                    controller, detector, frame, lane_results
                )
                
                if overlaid_frame is not None:
                    display_frame = overlaid_frame
                
                # Cập nhật telemetry
                update_telemetry(speed, steering_angle, brake, "lane_keeping")
                
                # In thông tin debug
                log.debug(f"Lane Keeping - Steering: {steering_angle}, Speed: {speed}, Brake: {brake}")
                
            elif current_mode == 'path_planning' and path_planner:
                # Khởi tạo dữ liệu IMU giả lập
                accel, gyro = generate_synthetic_imu(controller.car.state)
                
                # Cập nhật dữ liệu cảm biến cho path planning
                path_planner.update_sensor_data(accel, gyro, controller.car.state.current_speed / 1000.0)
                
                # Cập nhật path planning
                steering_angle, target_speed, active = path_planner.update()
                
                if active:
                    # Áp dụng điều khiển từ path planning
                    controller.car.set_steering(int(steering_angle))
                    controller.car.set_speed(int(target_speed))
                    brake = False
                    
                    # Lấy debug info
                    path_info = path_planner.get_debug_info()
                    
                    # Overlay thông tin path planning lên frame
                    draw_path_planning_overlay(display_frame, path_info)
                    
                    # Cập nhật telemetry với thông tin path planning
                    update_telemetry(
                        target_speed, 
                        steering_angle, 
                        brake, 
                        "path_planning", 
                        {
                            "progress": path_info.get('progress', 0),
                            "current_node": path_info.get('current_node', ""),
                            "target_node": path_info.get('target_node', ""),
                            "position": path_info.get('position', [0, 0]),
                            "distance_traveled": path_planner.position_estimator.get_distance_traveled()
                        }
                    )
                    
                    # In thông tin debug
                    log.debug(f"Path Planning - Steering: {steering_angle}, Speed: {target_speed}, Progress: {path_info.get('progress', 0):.1f}%")
                else:
                    # Path planning không hoạt động, quay về chế độ lane keeping
                    log.info("Path planning inactive, switching to lane keeping mode")
                    current_mode = 'lane_keeping'
                    continue
                
            else:  # Manual mode
                # Trong chế độ manual, vẫn thực hiện object detection
                detections = detector.detect(frame)
                
                # Hiển thị detections trên frame
                if isinstance(detections, (list, np.ndarray)) and len(detections) > 0:
                    for det in detections:
                        if len(det) >= 6:
                            x1, y1, x2, y2, conf, cls = map(int, det[:6])
                            cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Giữ nguyên tốc độ và góc lái
                steering_angle = controller.car.state.steering_angle
                speed = controller.car.state.current_speed
                brake = False
                
                # Cập nhật telemetry
                update_telemetry(speed, steering_angle, brake, "manual")
            
            # Thêm overlay hiển thị thông tin
            draw_info_overlay(display_frame, controller.car.state, current_mode)
            
            # Hiển thị frame
            cv2.imshow("Autonomous System", display_frame)
            
            # Stream video nếu cần
            if streaming_socket:
                try:
                    # Nén frame để giảm kích thước
                    _, img_encoded = cv2.imencode('.jpg', display_frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                    # Gửi dữ liệu qua UDP
                    streaming_socket.sendto(img_encoded.tobytes(), (server_ip, server_port))
                except Exception as e:
                    log.error(f"Streaming error: {e}")
            
            # Xử lý phím
            key = cv2.waitKey(1)
            if key == ord('q'):
                log.info("Quitting...")
                break
            elif key == ord('l'):
                current_mode = 'lane_keeping'
                log.info("Switched to lane keeping mode")
            elif key == ord('p'):
                if path_planner:
                    current_mode = 'path_planning'
                    log.info("Switched to path planning mode")
                else:
                    log.warning("Path planning not available")
            elif key == ord('m'):
                current_mode = 'manual'
                log.info("Switched to manual mode")
    
    finally:
        # Dừng xe khi kết thúc
        controller.car.brake()
        controller.car.close()
        if streaming_socket:
            streaming_socket.close()

def generate_synthetic_imu(vehicle_state):
    """
    Tạo dữ liệu IMU giả lập dựa trên trạng thái xe.
    
    Parameters:
    -----------
    vehicle_state : VehicleState
        Trạng thái hiện tại của xe
    
    Returns:
    --------
    tuple
        (accel, gyro) dữ liệu IMU giả lập
    """
    # Chuyển đổi từ mm/s sang m/s
    current_speed = vehicle_state.current_speed / 1000.0
    target_speed = vehicle_state.target_speed / 1000.0
    
    # Ước tính gia tốc theo hướng di chuyển
    accel_x = (target_speed - current_speed) / 0.1  # Giả định thời gian phản hồi 0.1s
    
    # Thêm nhiễu
    accel_noise = np.random.normal(0, 0.05, 3)
    gyro_noise = np.random.normal(0, 0.01, 3)
    
    # Ước tính tốc độ quay từ góc lái
    steering_angle = vehicle_state.steering_angle
    yaw_rate = steering_angle * 0.1  # Tỉ lệ đơn giản giữa góc lái và tốc độ quay
    
    # Tạo dữ liệu IMU
    accel = np.array([accel_x, 0, 9.81]) + accel_noise  # Thêm trọng lực vào trục z
    gyro = np.array([0, 0, yaw_rate]) + gyro_noise
    
    return accel, gyro

def draw_info_overlay(frame, vehicle_state, mode):
    """
    Vẽ overlay hiển thị thông tin lên frame.
    
    Parameters:
    -----------
    frame : numpy.ndarray
        Frame cần vẽ overlay
    vehicle_state : VehicleState
        Trạng thái hiện tại của xe
    mode : str
        Chế độ điều khiển hiện tại
    """
    # Vẽ overlay mờ ở góc trên bên phải
    h, w = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (w-210, 10), (w-10, 110), (0, 0, 0), -1)
    alpha = 0.7
    cv2.addWeighted(overlay, alpha, frame, 1-alpha, 0, frame)
    
    # Hiển thị thông tin
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, f"Mode: {mode}", (w-200, 30), font, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, f"Speed: {vehicle_state.current_speed} mm/s", (w-200, 50), font, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, f"Steering: {vehicle_state.steering_angle}°", (w-200, 70), font, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, f"Distance: {vehicle_state.total_distance:.2f} m", (w-200, 90), font, 0.5, (255, 255, 255), 1)

def draw_path_planning_overlay(frame, path_info):
    """
    Vẽ overlay hiển thị thông tin path planning lên frame.
    
    Parameters:
    -----------
    frame : numpy.ndarray
        Frame cần vẽ overlay
    path_info : dict
        Thông tin từ path planning
    """
    # Vẽ overlay mờ ở góc dưới bên trái
    h, w = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, h-110), (250, h-10), (0, 0, 0), -1)
    alpha = 0.7
    cv2.addWeighted(overlay, alpha, frame, 1-alpha, 0, frame)
    
    # Hiển thị thông tin
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, f"Progress: {path_info.get('progress', 0):.1f}%", (20, h-90), font, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, f"Current: {path_info.get('current_node', '')}", (20, h-70), font, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, f"Target: {path_info.get('target_node', '')}", (20, h-50), font, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, f"Remain: {path_info.get('remaining_distance', 0):.2f} m", (20, h-30), font, 0.5, (255, 255, 255), 1)

if __name__ == "__main__":
    main()
import time
import cv2
import numpy as np
import logging
import socket
import pickle
import struct
from src.autonomous_controller.autonomous_controller import AutonomousController
from src.parking_handler.parking_handler import ParkingStates

def connect_to_server(server_ip, server_port, max_retries=5):
    """Connect to the server on the laptop"""
    client_socket = None
    retry_count = 0
    
    while retry_count < max_retries:
        try:
            # Create new socket for each connection attempt
            if client_socket:
                client_socket.close()
                
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.settimeout(5)  # 5 second timeout for connection
            
            print(f"Attempt {retry_count+1}/{max_retries}: Connecting to server at {server_ip}:{server_port}")
            client_socket.connect((server_ip, server_port))
            print(f"Connection successful!")
            client_socket.settimeout(30)  # Longer timeout for normal operation
            return client_socket
            
        except socket.timeout:
            print(f"Connection timed out")
            retry_count += 1
            if retry_count < max_retries:
                print(f"Retrying in 3 seconds...")
                time.sleep(3)
            else:
                print(f"Failed to connect after {max_retries} attempts")
                return None
                
        except ConnectionRefusedError:
            print(f"Connection refused. Make sure server is running on {server_ip}:{server_port}")
            retry_count += 1
            if retry_count < max_retries:
                print(f"Retrying in 3 seconds...")
                time.sleep(3)
            else:
                print(f"Failed to connect after {max_retries} attempts")
                return None
                
        except Exception as e:
            print(f"Connection error: {e}")
            retry_count += 1
            if retry_count < max_retries:
                print(f"Retrying in 3 seconds...")
                time.sleep(3)
            else:
                print(f"Failed to connect after {max_retries} attempts")
                return None

def send_frame(client_socket, frame):
    """Send a frame to the server"""
    try:
        if client_socket:
            # Reduce frame size for better streaming performance
            frame_resized = cv2.resize(frame, (640, 480))
            _, encoded_frame = cv2.imencode('.jpg', frame_resized, [cv2.IMWRITE_JPEG_QUALITY, 80])
            data = pickle.dumps(encoded_frame)
            
            # Pack frame size and send
            message_size = struct.pack("L", len(data))
            client_socket.sendall(message_size + data)
            return True
    except (socket.error, BrokenPipeError, ConnectionResetError) as e:
        print(f"Connection error: {e}")
        return False
    except Exception as e:
        print(f"Error sending frame: {e}")
        return False
    return False

def run_autonomous_system(cap, lk, ld, server_ip=None, server_port=None, model_path=None):
    """
    Hàm chạy hệ thống xe tự hành, hiển thị một khung hình duy nhất với cả lane detection và object detection
    
    Args:
        cap: Camera capture object
        lk: LaneKeeping object
        ld: LaneDetection object
        server_ip: IP address của server nhận stream (nếu None thì không stream)
        server_port: Port của server nhận stream
        model_path: Đường dẫn đến model YOLO (nếu None sẽ sử dụng ObjectDetector mặc định)
    """
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # Tạo cửa sổ hiển thị
    cv2.namedWindow("Autonomous System", cv2.WINDOW_NORMAL)
    
    print("Đang khởi động hệ thống...")
    print("Nhấn 'q' để thoát chương trình")
    
    # Khởi tạo controller
    controller = AutonomousController(frame_width, frame_height)
    
    # Không cần sử dụng model_path nữa vì chúng ta chỉ dùng ObjectDetector mặc định
    print("Using default system ObjectDetector")
    
    # Khởi tạo kết nối socket nếu cần thiết
    client_socket = None
    if server_ip and server_port:
        print(f"Attempting to connect to streaming server at {server_ip}:{server_port}")
        client_socket = connect_to_server(server_ip, server_port)
        if not client_socket:
            print("Could not connect to server. Will continue without streaming.")
    
    # Các biến điều khiển
    STOP_TIMEOUT = 1.0
    DEFAULT_SPEED = controller.car.base_speed
    last_stop_time = 0
    last_process_time = time.time()
    last_time = time.time()
    
    # Biến điều khiển mới tối ưu cho Jetson
    frame_count = 0  # Chỉ dùng để theo dõi số frame đã xử lý
    
    # Thêm biến đếm để skip frame cho lane detection
    lane_frame_counter = 0  # Dùng để đếm và skip 1 frame sau khi đã xử lý 1 frame
    
    # Lưu giữ giá trị lane_fps giữa các frames để tránh nhảy về 0
    last_lane_fps = 0
    last_obj_fps = 0
    
    # Cờ hiệu khi hệ thống đã sẵn sàng
    system_ready = False
    initialization_frames = 10  # Số frame kiểm tra trước khi khởi động xe
    
    try:
        # Khởi tạo xe
        controller.car.set_power_state(30)
        time.sleep(0.5)
        
        # Đảm bảo xe đứng yên lúc khởi động
        controller.car.brake()
        
        # Hiển thị thông báo đang tải model
        loading_frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
        cv2.putText(loading_frame, "ĐANG TẢI MODEL OBJECT DETECTION...", 
                    (frame_width//2 - 220, frame_height//2), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imshow("Autonomous System", loading_frame)
        cv2.waitKey(1)
        
        # Gửi frame về server
        if client_socket:
            send_frame(client_socket, loading_frame)
        
        # Pre-load object detector model để đảm bảo đã sẵn sàng trước khi chạy
        dummy_frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
        controller.detector.detect(dummy_frame)
        print("Model đã được tải xong!")
        
        # Vòng lặp khởi tạo để kiểm tra hệ thống trước khi cho phép xe chạy
        init_count = 0
        while init_count < initialization_frames:
            ret, frame = cap.read()
            if not ret:
                raise Exception("Could not read frame during initialization")
            
            # Thêm thông báo khởi động vào khung hình chính
            display_frame = frame.copy()
            overlay = display_frame.copy()
            # Tạo một lớp overlay mờ để làm nổi bật thông báo
            cv2.rectangle(overlay, (0, frame_height//2-60), (frame_width, frame_height//2+60), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, display_frame, 0.5, 0, display_frame)
            cv2.putText(display_frame, "HỆ THỐNG ĐANG KHỞI ĐỘNG", 
                        (frame_width//2 - 150, frame_height//2 - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(display_frame, f"Tiến trình: {init_count+1}/{initialization_frames}", 
                        (frame_width//2 - 100, frame_height//2 + 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Hiển thị cửa sổ chính
            cv2.imshow("Autonomous System", display_frame)
            
            # Gửi frame về server
            if client_socket:
                send_frame(client_socket, display_frame)
            
            key = cv2.waitKey(1)
            if key == ord('q'):
                print("Thoát chương trình...")
                return
            
            init_count += 1
        
        # Hiển thị thông báo xe đang chạy
        ret, frame = cap.read()
        if ret:
            display_frame = frame.copy()
            overlay = display_frame.copy()
            cv2.rectangle(overlay, (0, frame_height//2-60), (frame_width, frame_height//2+60), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, display_frame, 0.5, 0, display_frame)
            cv2.putText(display_frame, "XE ĐANG CHẠY TỰ ĐỘNG", 
                        (frame_width//2 - 150, frame_height//2), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Autonomous System", display_frame)
            
            # Gửi frame về server
            if client_socket:
                send_frame(client_socket, display_frame)
                
            cv2.waitKey(1)
            
            # CHỈ BẮT ĐẦU CHẠY XE SAU KHI ĐÃ TẢI MODEL XONG
            controller.car.set_speed(DEFAULT_SPEED)
            print("Xe bắt đầu chạy với tốc độ:", DEFAULT_SPEED)
            system_ready = True
        
        # Vòng lặp chính của hệ thống
        while system_ready:
            loop_start_time = time.time()
            ret, frame = cap.read()
            
            if not ret:
                break
                
            current_time = time.time()
            delta_time = current_time - last_time
            last_time = current_time
            
            # Khởi tạo khung hình hiển thị
            combined_frame = frame.copy()
            
            # Cập nhật khoảng cách di chuyển
            controller.car.state.update_distance(delta_time * 1000)  # Chuyển đổi thành ms
            
            frame_count += 1
            
            # Process only if enough time has passed (adaptive frame processing)
            if current_time - last_process_time < 0.033:  # ~30fps max
                key = cv2.waitKey(1)
                if key == ord('q'):
                    print("Thoát chương trình...")
                    break
                    
                # Hiển thị frame hiện tại mà không xử lý thêm
                cv2.imshow("Autonomous System", combined_frame)
                
                # Gửi frame về server nếu cần
                if client_socket:
                    send_success = send_frame(client_socket, combined_frame)
                    if not send_success and frame_count % 30 == 0:
                        print("Failed to send frame, attempting to reconnect...")
                        client_socket = connect_to_server(server_ip, server_port, max_retries=1)
                        
                continue
                
            last_process_time = current_time
            
            # Nếu không ở chế độ đỗ xe, thực hiện phát hiện làn đường, nhưng skip 1 frame sau mỗi lần xử lý
            lane_results = None
            steering_angle = None
            lane_fps = last_lane_fps  # Sử dụng giá trị FPS cuối cùng
            
            if not controller.parking_mode:
                # Tăng biến đếm cho lane frame
                lane_frame_counter += 1
                
                # Chỉ xử lý mỗi frame thứ 2 (skip 1 frame)
                if lane_frame_counter % 2 == 1:
                    lane_start_time = time.time()
                    lane_frame = frame.copy()
                    lane_results = ld.lanes_detection(lane_frame)
                    
                    if lane_results is not None:
                        steering_angle, lane_overlay = lk.lane_keeping(lane_results)
                        
                        # Đè khung hình lane detection lên khung hình kết hợp
                        if lane_overlay is not None:
                            # Chỉ sao chép các phần đã vẽ từ lane_overlay
                            # Phát hiện sự khác biệt giữa lane_frame và lane_overlay
                            diff = cv2.absdiff(lane_frame, lane_overlay)
                            mask = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
                            _, mask = cv2.threshold(mask, 20, 255, cv2.THRESH_BINARY)
                            
                            # Áp dụng mask để chỉ sao chép các phần đã được vẽ
                            lane_roi = cv2.bitwise_and(lane_overlay, lane_overlay, mask=mask)
                            cv2.addWeighted(combined_frame, 1.0, lane_roi, 1.0, 0, combined_frame)
                        
                        if steering_angle is not None:
                            controller.car.state.current_lane_type = controller.lane_analyzer.detect_lane_type(
                                frame, lane_results)
                    
                    lane_fps = 1.0 / (time.time() - lane_start_time)
                    last_lane_fps = lane_fps  # Lưu lại giá trị FPS hiện tại
            
            # Xử lý object detection
            obj_start_time = time.time()
            
            # Sử dụng ObjectDetector từ hệ thống
            detections = controller.detector.detect(frame)
            
            # Vẽ bounding box trực tiếp lên khung hình kết hợp nếu có phát hiện
            if isinstance(detections, (list, np.ndarray)) and len(detections) > 0:
                for det in detections:
                    if len(det) >= 6:  # [x1, y1, x2, y2, conf, cls]
                        x1, y1, x2, y2, conf, cls = det
                        
                        # Chuyển đổi sang int
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        
                        # Lấy tên lớp
                        class_name = controller.traffic_processor.CLASS_NAMES[int(cls)] if int(cls) < len(controller.traffic_processor.CLASS_NAMES) else f"Class {int(cls)}"
                        
                        # Tạo màu dựa trên class id
                        color = (int(hash(class_name) % 255), 
                                 int(hash(class_name*2) % 255), 
                                 int(hash(class_name*3) % 255))
                        
                        # Vẽ bounding box
                        cv2.rectangle(combined_frame, (x1, y1), (x2, y2), color, 2)
                        
                        # Thêm nhãn với nền mờ
                        text = f"{class_name}: {conf:.2f}"
                        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                        cv2.rectangle(combined_frame, (x1, y1-text_size[1]-5), (x1+text_size[0]+5, y1), color, -1)
                        cv2.putText(combined_frame, text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            obj_fps = 1.0 / (time.time() - obj_start_time)
            last_obj_fps = obj_fps  # Lưu lại giá trị FPS hiện tại
            
            # Xử lý điều khiển xe từ kết quả detection
            highest_priority_speed = DEFAULT_SPEED
            stop_condition = False
            parking_action = None
            parking_speed = None
            parking_steering = None
            
            if isinstance(detections, (list, np.ndarray)) and len(detections) > 0:
                # Convert to list if numpy array
                if isinstance(detections, np.ndarray):
                    detections = detections.tolist()
                
                # Sort detections by proximity (y2 coordinate)
                detections.sort(key=lambda x: float(x[3]) if len(x) > 3 else 0, reverse=True)
                
                # Xử lý chế độ đỗ xe nếu đang trong parking mode
                if controller.parking_mode:
                    parking_spot_found = False
                    for det in detections:
                        if len(det) >= 6 and int(det[5]) == controller.traffic_processor.CLASS_NAMES.index('parking-spot'):
                            action, speed, steering = controller.parking_handler.process_parking(
                                controller.car.state, det, frame_width, frame_height)
                            parking_action = action
                            parking_speed = speed
                            parking_steering = steering
                            parking_spot_found = True
                            break
                            
                    # Nếu không tìm thấy parking spot nhưng đang trong chế độ đỗ xe
                    if not parking_spot_found:
                        action, speed, steering = controller.parking_handler.process_parking(
                            controller.car.state, None, frame_width, frame_height)
                        parking_action = action
                        parking_speed = speed
                        parking_steering = steering
                
                # Process only the most critical detections
                for det in detections[:3]:  # Limit to 3 most important detections
                    if len(det) >= 6:  # Ensure detection has all required elements
                        try:
                            action, speed = controller.process_detection(det, frame)
                            
                            if action == 'stop':
                                stop_condition = True
                                last_stop_time = current_time
                                controller.car.brake()
                                break
                            elif action in ['caution', 'slow'] and not stop_condition:
                                if speed is not None:
                                    highest_priority_speed = min(highest_priority_speed, 
                                        int(speed))
                                else:
                                    highest_priority_speed = min(highest_priority_speed,
                                        int(DEFAULT_SPEED * 0.5))
                            elif action == 'proceed':
                                if speed is not None:
                                    highest_priority_speed = min(highest_priority_speed,
                                        int(speed))
                        except Exception as e:
                            print(f"Error processing detection: {e}")
                            continue
            
            # Kết hợp thông tin từ lane detection và object detection
            if not stop_condition and current_time - last_stop_time > STOP_TIMEOUT:
                # Ưu tiên xử lý parking nếu đang trong chế độ đỗ xe
                if controller.parking_mode and parking_action is not None:
                    # Cập nhật trạng thái xe dựa trên hành động đỗ xe
                    controller.update_vehicle_state(parking_action, parking_speed, parking_steering)
                    
                    # Nếu đã hoàn thành quá trình đỗ xe
                    if parking_action == 'proceed':
                        controller.parking_mode = False
                        controller.car.state.reset_parking_state()
                        print("Parking completed, returning to normal mode")
                else:
                    # Chế độ điều khiển bình thường
                    if controller.car.state.current_speed == 0:
                        controller.car.set_speed(highest_priority_speed)
            
                    # Apply steering with improved response
                    if not controller.parking_mode and steering_angle is not None:
                        controller.car.set_steering(int(steering_angle))
            
            # Hiển thị overlay thông tin hệ thống lên khung hình chính
            # Tạo vùng mờ ở góc dưới bên phải cho thông tin hệ thống
            overlay = combined_frame.copy()
            cv2.rectangle(overlay, (frame_width-220, frame_height-150), (frame_width, frame_height), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, combined_frame, 0.4, 0, combined_frame)
            
            # Thêm thông tin hệ thống lên khung hình
            cv2.putText(combined_frame, f"Speed: {controller.car.state.current_speed}", 
                        (frame_width-210, frame_height-120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(combined_frame, f"Steering: {controller.car.state.steering_angle}", 
                        (frame_width-210, frame_height-90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(combined_frame, f"Lane: {controller.car.state.current_lane_type}", 
                        (frame_width-210, frame_height-60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                        
            # Hiển thị trạng thái đỗ xe hoặc dừng với màu khác nhau
            status_color = (0, 255, 0)  # Mặc định là màu xanh (RUNNING)
            status_text = "RUNNING"
            
            if controller.parking_mode:
                parking_state = controller.car.state.parking_state.name if controller.car.state.parking_state else "None"
                status_text = f"PARKING: {parking_state}"
                status_color = (0, 200, 200)  # Màu vàng
            elif stop_condition:
                status_text = "STOPPED"
                status_color = (0, 0, 255)  # Màu đỏ
            
            # Hiển thị trạng thái ở trên cùng không có nền
            cv2.putText(combined_frame, f"STATUS: {status_text}", 
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            # Hiển thị FPS to ở phía trên tương tự dòng status, không có nền đen
            # Hiển thị FPS Lane với font size lớn hơn
            cv2.putText(combined_frame, f"LANE FPS: {lane_fps:.1f}", 
                        (230, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
            # Hiển thị FPS Object với font size lớn hơn
            cv2.putText(combined_frame, f"OBJ FPS: {obj_fps:.1f}", 
                        (460, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Hiển thị thông tin streaming nếu đang kết nối
            if client_socket:
                cv2.putText(combined_frame, "STREAMING: ON", 
                            (frame_width-210, frame_height-30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            
            cv2.imshow("Autonomous System", combined_frame)
            
            # Gửi frame về server nếu có kết nối
            if client_socket:
                send_success = send_frame(client_socket, combined_frame)
                if not send_success and frame_count % 30 == 0:
                    print("Failed to send frame, attempting to reconnect...")
                    client_socket = connect_to_server(server_ip, server_port, max_retries=1)
            
            # Xử lý phím điều khiển
            key = cv2.waitKey(1)
            if key == ord('q'):
                print("Thoát chương trình...")
                break
            elif key == ord('b'):
                controller.car.brake()
                print("Phanh khẩn cấp!")
            elif key == ord('r'):
                controller.car.set_speed(DEFAULT_SPEED)
                print("Tiếp tục di chuyển")
                
    finally:
        controller.car.brake()
        controller.car.close()
        
        # Đóng kết nối socket nếu có
        if 'client_socket' in locals() and client_socket:
            client_socket.close()
            print("Socket connection closed")
            
        cv2.destroyAllWindows()

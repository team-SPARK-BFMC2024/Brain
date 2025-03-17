import time
import cv2
import numpy as np
import logging
import threading
import socket
from src.autonomous_controller.autonomous_controller import AutonomousController
from src.parking_handler.parking_handler import ParkingStates
from src.mode_controller.mode_controller import ModeController, OperationMode
from src.path_planning.path_controller import PathController

def run_autonomous_system(cap, lk, ld, video_streamer=None, server_ip=None, server_port=None, model_path=None):
    """
    Main function to run the autonomous system with support for different modes.
    
    Args:
        cap: Camera capture object
        lk: LaneKeeping object
        ld: LaneDetection object
        server_ip: IP address for streaming (optional)
        server_port: Port for streaming (optional)
        model_path: Path to the object detection model (optional)
    """
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # Create the main display window
    cv2.namedWindow("Autonomous System", cv2.WINDOW_NORMAL)
    
    print("System initializing...")
    print("Press 'q' to exit the program")
    
    # Initialize mode controller
    mode_controller = ModeController(port=8099)
    mode_controller.start()
    
    # Initialize autonomous controller
    controller = AutonomousController(frame_width, frame_height)
    
    # Initialize path planning controller
    path_controller = PathController()
    
    # Initialize streaming socket if needed
    stream_socket = None
    if server_ip and server_port:
        try:
            stream_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"Streaming enabled to {server_ip}:{server_port}")
        except Exception as e:
            print(f"Error setting up streaming socket: {e}")
            stream_socket = None
    
    # Control variables
    STOP_TIMEOUT = 1.0
    DEFAULT_SPEED = controller.car.base_speed
    last_stop_time = 0
    last_process_time = time.time()
    last_time = time.time()
    
    # Performance tracking variables
    frame_count = 0
    lane_frame_counter = 0
    last_lane_fps = 0
    last_obj_fps = 0
    
    # IMU data handling variables
    imu_data_buffer = ""
    last_imu_update = time.time()
    
    # System initialization
    system_ready = False
    initialization_frames = 10
    
    # Set up mode change handler
    def handle_mode_change(new_mode):
        nonlocal system_ready
        
        if new_mode == OperationMode.STOP:
            controller.car.brake()
            path_controller.stop_navigation()
            print("Vehicle stopped")
            
        elif new_mode == OperationMode.AUTO:
            # Start path planning navigation
            if not path_controller.navigation_started:
                path_controller.start_navigation()
            controller.car.set_speed(DEFAULT_SPEED)
            print("Auto mode activated - following planned path")
            
        elif new_mode == OperationMode.LEGACY:
            # Traditional lane following without path planning
            path_controller.stop_navigation()
            controller.car.set_speed(DEFAULT_SPEED)
            print("Legacy mode activated - following lanes")
            
        elif new_mode == OperationMode.MANUAL:
            # Stop autonomous functions but leave vehicle responsive
            path_controller.stop_navigation()
            print("Manual mode activated - awaiting control commands")
    
    # Register the mode change handler
    mode_controller.set_mode_change_callback(handle_mode_change)
    
    try:
        # Initialize the vehicle
        controller.car.set_power_state(30)
        time.sleep(0.5)
        
        # Ensure the vehicle is stopped at startup
        controller.car.brake()
        
        # Display loading message
        loading_frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
        cv2.putText(loading_frame, "LOADING OBJECT DETECTION MODEL...", 
                    (frame_width//2 - 220, frame_height//2), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imshow("Autonomous System", loading_frame)
        cv2.waitKey(1)
        
        # Pre-load object detector model
        dummy_frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
        controller.detector.detect(dummy_frame)
        print("Model loaded successfully!")
        
        # System initialization checks
        init_count = 0
        while init_count < initialization_frames:
            ret, frame = cap.read()
            if not ret:
                raise Exception("Could not read frame during initialization")
            
            # Display initialization progress
            display_frame = frame.copy()
            overlay = display_frame.copy()
            cv2.rectangle(overlay, (0, frame_height//2-60), (frame_width, frame_height//2+60), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, display_frame, 0.5, 0, display_frame)
            cv2.putText(display_frame, "SYSTEM INITIALIZING", 
                        (frame_width//2 - 150, frame_height//2 - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(display_frame, f"Progress: {init_count+1}/{initialization_frames}", 
                        (frame_width//2 - 100, frame_height//2 + 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Display initialization window
            cv2.imshow("Autonomous System", display_frame)
            
            key = cv2.waitKey(1)
            if key == ord('q'):
                print("Exiting program...")
                return
            
            init_count += 1
        
        # Display ready message
        ret, frame = cap.read()
        if ret:
            display_frame = frame.copy()
            overlay = display_frame.copy()
            cv2.rectangle(overlay, (0, frame_height//2-60), (frame_width, frame_height//2+60), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, display_frame, 0.5, 0, display_frame)
            cv2.putText(display_frame, "SYSTEM READY - AWAITING COMMANDS", 
                        (frame_width//2 - 220, frame_height//2), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Autonomous System", display_frame)
            cv2.waitKey(1)
            
            system_ready = True
            print("System ready - awaiting mode selection from web interface")
        
        # Main system loop
        while system_ready:
            loop_start_time = time.time()
            ret, frame = cap.read()
            
            if not ret:
                break
                
            current_time = time.time()
            delta_time = current_time - last_time
            last_time = current_time
            
            # Create display frame
            combined_frame = frame.copy()
            
            # Update vehicle distance tracking
            controller.car.state.update_distance(delta_time * 1000)  # Convert to ms
            
            frame_count += 1
            
            # Process frame only if enough time has passed (for performance)
            if current_time - last_process_time < 0.033:  # ~30fps max
                key = cv2.waitKey(1)
                if key == ord('q'):
                    print("Exiting program...")
                    break
                    
                # Display current frame without processing
                cv2.imshow("Autonomous System", combined_frame)
                
                # Check for simulated IMU data (in actual system, this would come from a serial port)
                # For demo purposes, we'll generate simulated IMU data
                if current_time - last_imu_update > 0.1:  # 10Hz IMU updates
                    # Simulate IMU data (in a real system this would come from hardware)
                    yaw = controller.car.state.steering_angle  # Use steering as proxy for yaw
                    sim_imu_data = f"@imu:0.0,0.0,{yaw},{controller.car.state.current_speed/1000.0},0.0,0.0\r\n"
                    
                    # Process the IMU data for path planning
                    if mode_controller.get_mode() == OperationMode.AUTO:
                        path_controller.process_imu_data(sim_imu_data)
                    
                    last_imu_update = current_time
                    
                continue
                
            last_process_time = current_time
            
            # Get current operation mode
            current_mode = mode_controller.get_mode()
            
            # Process based on current mode
            if current_mode == OperationMode.STOP:
                # In STOP mode, just display the frame without processing
                controller.car.brake()
                
            elif current_mode in [OperationMode.LEGACY, OperationMode.AUTO]:
                # For both LEGACY and AUTO modes, we detect lanes and objects
                
                # Lane detection processing (skip frames for performance)
                lane_results = None
                steering_angle = None
                lane_fps = last_lane_fps
                
                if not controller.parking_mode:
                    lane_frame_counter += 1
                    
                    if lane_frame_counter % 2 == 1:
                        lane_start_time = time.time()
                        lane_frame = frame.copy()
                        lane_results = ld.lanes_detection(lane_frame)
                        
                        if lane_results is not None:
                            # In AUTO mode, override lane keeping with path planning
                            if current_mode == OperationMode.AUTO and path_controller.is_active:
                                # Update path controller
                                steering_angle, speed_factor, reached_target = path_controller.update()
                                
                                # Apply path planning steering angle
                                controller.car.set_steering(int(steering_angle))
                                
                                # Adjust speed based on turn sharpness
                                if speed_factor is not None:                                
                                    if abs(steering_angle) > 15:  # For significant turns
                                        target_speed = 100 
                                    else:
                                        turn_ratio = min(1.0, abs(steering_angle) / 15)
                                        target_speed = int(250 - (150 * turn_ratio))  # Linear reduction from 250 to 100
    
                                    controller.car.set_speed(target_speed)

                                # Use lane detection output for visualization only
                                _, lane_overlay = lk.lane_keeping(lane_results)
                                
                            else:  # LEGACY mode - use lane keeping
                                steering_angle, lane_overlay = lk.lane_keeping(lane_results)
                                
                                if steering_angle is not None:
                                    controller.car.set_steering(int(steering_angle))
                                    controller.car.state.current_lane_type = controller.lane_analyzer.detect_lane_type(
                                        frame, lane_results)
                            
                            # Overlay lane visualization on display frame
                            if lane_overlay is not None:
                                # Only copy the parts that differ (lane markings)
                                diff = cv2.absdiff(lane_frame, lane_overlay)
                                mask = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
                                _, mask = cv2.threshold(mask, 20, 255, cv2.THRESH_BINARY)
                                
                                lane_roi = cv2.bitwise_and(lane_overlay, lane_overlay, mask=mask)
                                cv2.addWeighted(combined_frame, 1.0, lane_roi, 1.0, 0, combined_frame)
                        
                        lane_fps = 1.0 / (time.time() - lane_start_time)
                        last_lane_fps = lane_fps
                
                # Object detection processing
                obj_start_time = time.time()
                detections = controller.detector.detect(frame)
                
                # Process and visualize detections
                if isinstance(detections, (list, np.ndarray)) and len(detections) > 0:
                    for det in detections:
                        if len(det) >= 6:  # [x1, y1, x2, y2, conf, cls]
                            x1, y1, x2, y2, conf, cls = det
                            
                            # Convert to integers
                            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                            
                            # Get class name
                            class_name = controller.traffic_processor.CLASS_NAMES[int(cls)] if int(cls) < len(controller.traffic_processor.CLASS_NAMES) else f"Class {int(cls)}"
                            
                            # Create color based on class
                            color = (int(hash(class_name) % 255), 
                                     int(hash(class_name*2) % 255), 
                                     int(hash(class_name*3) % 255))
                            
                            # Draw bounding box
                            cv2.rectangle(combined_frame, (x1, y1), (x2, y2), color, 2)
                            
                            # Add label with semi-transparent background
                            text = f"{class_name}: {conf:.2f}"
                            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                            cv2.rectangle(combined_frame, (x1, y1-text_size[1]-5), (x1+text_size[0]+5, y1), color, -1)
                            cv2.putText(combined_frame, text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                obj_fps = 1.0 / (time.time() - obj_start_time)
                last_obj_fps = obj_fps
                
                # Process object detections for traffic rules in both LEGACY and AUTO modes
                highest_priority_speed = DEFAULT_SPEED
                stop_condition = False
                parking_action = None
                parking_speed = None
                parking_steering = None
                
                if isinstance(detections, (list, np.ndarray)) and len(detections) > 0:
                    # Convert to list if it's a numpy array
                    if isinstance(detections, np.ndarray):
                        detections = detections.tolist()
                    
                    # Sort detections by proximity (y2 coordinate)
                    detections.sort(key=lambda x: float(x[3]) if len(x) > 3 else 0, reverse=True)
                    
                    # Handle parking mode
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
                                
                        # If no parking spot found but in parking mode
                        if not parking_spot_found:
                            action, speed, steering = controller.parking_handler.process_parking(
                                controller.car.state, None, frame_width, frame_height)
                            parking_action = action
                            parking_speed = speed
                            parking_steering = steering
                    
                    # Process critical detections
                    for det in detections[:3]:  # Limit to 3 most important
                        if len(det) >= 6:
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
                
                # Apply control decisions based on detections and mode
                if not stop_condition and current_time - last_stop_time > STOP_TIMEOUT:
                    # Handle parking first if in parking mode
                    if controller.parking_mode and parking_action is not None:
                        controller.update_vehicle_state(parking_action, parking_speed, parking_steering)
                        
                        if parking_action == 'proceed':
                            controller.parking_mode = False
                            controller.car.state.reset_parking_state()
                            print("Parking completed, returning to normal mode")
                    elif current_mode == OperationMode.AUTO:
                        # In AUTO mode, speed is managed by path controller
                        pass
                    else:
                        # Normal speed control in LEGACY mode
                        if controller.car.state.current_speed == 0:
                            controller.car.set_speed(highest_priority_speed)
            
            # Streaming functionality
            if video_streamer:
                # Update frame in the video streamer
                video_streamer.update_frame(combined_frame)
            elif stream_socket and server_ip and server_port:
                try:
                    # Compress frame before sending
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
                    _, buffer = cv2.imencode('.jpg', combined_frame, encode_param)
                    stream_socket.sendto(buffer.tobytes(), (server_ip, server_port))
                except Exception as e:
                    print(f"Streaming error: {e}")
            
            # Add system information overlay
            # Create semi-transparent region for system info
            overlay = combined_frame.copy()
            cv2.rectangle(overlay, (frame_width-220, frame_height-150), (frame_width, frame_height), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, combined_frame, 0.4, 0, combined_frame)
            
            # Display system information
            cv2.putText(combined_frame, f"Speed: {controller.car.state.current_speed}", 
                        (frame_width-210, frame_height-120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(combined_frame, f"Steering: {controller.car.state.steering_angle}", 
                        (frame_width-210, frame_height-90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(combined_frame, f"Lane: {controller.car.state.current_lane_type}", 
                        (frame_width-210, frame_height-60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # Display mode information
            mode_text = "MODE: " + current_mode.name
            
            # Pick color based on mode
            if current_mode == OperationMode.STOP:
                mode_color = (0, 0, 255)  # Red
            elif current_mode == OperationMode.AUTO:
                mode_color = (0, 255, 0)  # Green
            elif current_mode == OperationMode.LEGACY:
                mode_color = (255, 255, 0)  # Yellow
            else:
                mode_color = (255, 255, 255)  # White
                
            cv2.putText(combined_frame, mode_text, 
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)
                        
            # Display status
            status_color = (0, 255, 0)  # Default green (RUNNING)
            status_text = "RUNNING"
            
            if controller.parking_mode:
                parking_state = controller.car.state.parking_state.name if controller.car.state.parking_state else "None"
                status_text = f"PARKING: {parking_state}"
                status_color = (0, 200, 200)  # Yellow
            elif stop_condition:
                status_text = "STOPPED"
                status_color = (0, 0, 255)  # Red
                
            cv2.putText(combined_frame, f"STATUS: {status_text}", 
                        (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                        
            # Display FPS information
            cv2.putText(combined_frame, f"LANE FPS: {lane_fps:.1f}", 
                        (230, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(combined_frame, f"OBJ FPS: {obj_fps:.1f}", 
                        (460, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
            # Add path planning info if in AUTO mode
            if current_mode == OperationMode.AUTO and path_controller.is_active:
                status = path_controller.get_navigation_status()
                cv2.putText(combined_frame, f"Path: {status['current_node']} -> {status['next_node']}", 
                            (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.putText(combined_frame, f"Dist to next: {status['distance_to_next']:.2f}m", 
                            (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            
            # Display the combined frame
            cv2.imshow("Autonomous System", combined_frame)
            
            # Process keyboard input
            key = cv2.waitKey(1)
            if key == ord('q'):
                print("Exiting program...")
                break
            elif key == ord('b'):
                controller.car.brake()
                print("Emergency brake!")
            elif key == ord('r'):
                controller.car.set_speed(DEFAULT_SPEED)
                print("Resume movement")
            elif key == ord('a'):  # Manual mode switching via keyboard for testing
                mode_controller.set_mode(OperationMode.AUTO)
            elif key == ord('l'):
                mode_controller.set_mode(OperationMode.LEGACY)
            elif key == ord('s'):
                mode_controller.set_mode(OperationMode.STOP)
                
    finally:
        # Clean shutdown
        controller.car.brake()
        time.sleep(0.2)
        controller.car.set_power_state(0)
        controller.car.close()
        
        # Stop the mode controller
        mode_controller.stop()
        
        # Close any open windows
        cv2.destroyAllWindows()
        
        # Close streaming socket if open
        if stream_socket:
            stream_socket.close()
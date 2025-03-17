import cv2
import logging
import argparse
import os
import socket
import threading
import time
from src.lane_detection.lane_detection import LaneDetection
from src.lane_detection.lane_keeping import LaneKeeping
from src.system_runner.system_runner import run_autonomous_system
from src.mode_controller.mode_controller import ModeController, OperationMode
from src.web_api.web_api_server import WebSocketServer
from src.path_planning.path_controller import PathController

# Setup logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger('Root logger')

def get_ip_address():
    """Get the device's IP address"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception as e:
        print(f"Could not get IP address: {e}")
        return "Unknown"

def run_web_server(port=8088, web_dir='web'):
    """
    Launch a simple web server to serve the web interface files.
    
    Args:
        port (int): Port to run the web server on
        web_dir (str): Directory containing web files
    """
    import http.server
    import socketserver
    
    # Create HTTP request handler
    class SimpleHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
        def do_GET(self):
            # Handle requests to web files
            if self.path == '/' or self.path == '/index.html':
                self.path = f'/{web_dir}/index.html'
            elif self.path.startswith(f'/{web_dir}/'):
                # Keep path as is
                pass
            super().do_GET()
            
        def log_message(self, format, *args):
            # Skip logging to keep console clean
            return
    
    # Ensure web directory exists
    if not os.path.exists(web_dir):
        os.makedirs(web_dir)
        print(f"Created web directory: {web_dir}")
    
    # Start HTTP server
    try:
        handler = SimpleHTTPRequestHandler
        with socketserver.TCPServer(("", port), handler) as httpd:
            print(f"Web server running at http://{get_ip_address()}:{port}")
            httpd.serve_forever()
    except Exception as e:
        print(f"Error running web server: {e}")

def main():
    # Display IP address information
    jetson_ip = get_ip_address()
    print(f"Jetson IP: {jetson_ip}")
    
    # Create parser to handle command line arguments
    parser = argparse.ArgumentParser(description='Autonomous vehicle system with web control')
    parser.add_argument('--model', type=str, default=None, help='Path to YOLO model (engine/pt)')
    parser.add_argument('--camera', type=int, default=0, help='Camera ID')
    parser.add_argument('--width', type=int, default=640, help='Camera width')
    parser.add_argument('--height', type=int, default=480, help='Camera height')
    parser.add_argument('--web-port', type=int, default=8088, help='Port for web server')
    parser.add_argument('--ws-port', type=int, default=8765, help='Port for WebSocket server')
    parser.add_argument('--no-web', action='store_true', help='Do not run web server')
    args = parser.parse_args()

    # Initialize mode controller
    mode_controller = ModeController()
    
    # Initialize WebSocket server instead of HTTP API server
    ws_server = WebSocketServer(mode_controller, port=args.ws_port)
    ws_server.start()
    print(f"WebSocket server started on port {args.ws_port}")

    # Run web server in a separate thread
    if not args.no_web:
        web_thread = threading.Thread(target=run_web_server, args=(args.web_port, 'web'))
        web_thread.daemon = True
        web_thread.start()
        print(f"Web interface available at http://{jetson_ip}:{args.web_port}")

    # Open camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        log.error("Could not open webcam")
        exit(1)

    # Set camera resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    try:
        # Read first frame to initialize objects
        ret, frame = cap.read()
        if not ret:
            raise Exception("Could not read initial frame")

        # Initialize lane detection objects
        lk = LaneKeeping(frame.shape[1], frame.shape[0], log, "455")
        ld = LaneDetection(frame.shape[1], frame.shape[0], "455", lk)

        # Setup optimized lane detection parameters 
        ld.square_pulses_min_height = 80
        ld.square_pulses_pix_dif = 10
        ld.square_pulses_min_height_dif = 20
        ld.square_pulses_allowed_peaks_width_error = 15

        # Initialize path controller for AUTO mode
        path_controller = PathController()
        
        # Setup callback for mode controller
        def handle_mode_change(new_mode):
            if new_mode == OperationMode.AUTO:
                # Start path planning
                path_controller.start_navigation()
                print("AUTO mode activated - Path planning enabled")
            elif new_mode == OperationMode.LEGACY:
                # Stop path planning, use lane following only
                path_controller.stop_navigation()
                print("LEGACY mode activated - Lane following only")
            elif new_mode == OperationMode.STOP:
                # Stop vehicle
                path_controller.stop_navigation()
                print("STOP mode activated - Vehicle stopped")
            elif new_mode == OperationMode.MANUAL:
                # Manual control mode
                path_controller.stop_navigation()
                print("MANUAL mode activated - Waiting for manual control")
        
        # Register callback
        mode_controller.set_mode_change_callback(handle_mode_change)

        # Setup callback to update data to WebSocket server
        def update_status_callback(data):
            # Update status to WebSocket server
            ws_server.update_status(data)
        
        # Run autonomous system
        run_autonomous_system(
            cap=cap, 
            lk=lk, 
            ld=ld, 
            mode_controller=mode_controller, 
            path_controller=path_controller,
            ws_server=ws_server,
            status_callback=update_status_callback,
            model_path=args.model
        )

    except Exception as e:
        log.error(f"Error: {e}")
    finally:
        # Clean up resources when finished
        cap.release()
        cv2.destroyAllWindows()
        
        # Stop servers
        ws_server.stop()
        
        print("System shutdown complete")

if __name__ == "__main__":
    main()
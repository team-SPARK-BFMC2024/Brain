# 🚗 SPARK - Self-driving Platform for Autonomous Research and Knowledge

SPARK is a self-driving platform designed for researching and developing autonomous driving algorithms on the Jetson Nano. The system combines lane detection, object recognition, path planning, and traffic rule processing to control the vehicle autonomously.

## ✨ Key Features

- Lane detection and keeping
- Traffic object recognition
- Traffic rule processing
- Path planning and navigation
- Autonomous parking
- Multiple operation modes (AUTO, LEGACY, MANUAL, STOP)
- Automatic speed adjustment during turns
- Web interface for control and monitoring
- Video streaming to a computer for monitoring

## 🔧 System Requirements

### Hardware
- NVIDIA Jetson Nano (4GB RAM)
- USB Camera or Raspberry Pi Camera
- Portable power supply/battery
- RC vehicle chassis (with servo and motor controllers)

### Software
- JetPack 4.6+ (L4T)
- Python 3.6+
- OpenCV 4.5+
- PyTorch 1.8+ (with CUDA)
- Ultralytics YOLO

Detailed dependencies are listed in the [requirements.txt](requirements.txt) file.

## 📥 Installation

1. Clone repository:
```bash
git clone https://github.com/team-SPARK-BFMC2024/Brain.git
cd Brain
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Hardware configuration:
   - Connect the camera to the Jetson Nano
   - Connect the vehicle controller to the Jetson Nano via serial port

## 🚀 Usage

### Running the autonomous vehicle (on Jetson Nano)

```bash
python main3.py
```

Options:
- `--camera`: Camera ID (default: 0)
- `--model`: Path to YOLO model (default: uses pre-installed model)
- `--width`: Camera frame width (default: 640)
- `--height`: Camera frame height (default: 480)
- `--web-port`: Web interface port (default: 8088)
- `--ws-port`: WebSocket server port (default: 8765)
- `--no-web`: Disable web interface

### Running the WebSocket server separately (for telemetry)

```bash
python websocket_server.py --port 8090
```

## 🎮 Operation Modes

The system supports multiple operation modes:

- **AUTO**: Full autonomous driving with path planning
- **LEGACY**: Lane following without path planning
- **MANUAL**: Remote control operation
- **STOP**: Safely halts the vehicle

Modes can be switched via the web interface or using keyboard shortcuts in the monitoring window:
- `a`: Switch to AUTO mode
- `l`: Switch to LEGACY mode
- `s`: Switch to STOP mode

## 🏎️ Speed Control

The system automatically adjusts speed based on turning angle:
- Normal driving: 25 cm/s (250 mm/s)
- During turns: Reduces to as low as 10 cm/s (100 mm/s)

This helps ensure safe navigation around corners and during complex maneuvers.

## 📁 Code Structure

```
SPARK/
├── main.py              # Legacy entry point
├── main2.py             # Alternative entry point
├── main3.py             # Main entry point with web interface
├── websocket_server.py  # WebSocket server for telemetry
├── monitoring           # Project documentation
├── requirements.txt     # Dependencies
├── config.ini           # Configuration parameters
├── README.md            # This file
├── Project status       # Progress reports
└── src/                 # Source code
    ├── autonomous_controller/
    │   └── autonomous_controller.py  # Main controller
    ├── car_controller/
    │   └── car_controller.py         # Hardware control
    ├── lane_analyzer/
    │   └── lane_analyzer.py          # Lane type detection
    ├── lane_detection/
    │   ├── lane_detection.py         # Lane detection
    │   └── lane_keeping.py           # Lane following
    ├── mode_controller/
    │   └── mode_controller.py        # Operation mode management
    ├── object_detector/
    │   └── object_detector.py        # Object detection (YOLO)
    ├── parking_handler/
    │   └── parking_handler.py        # Automated parking
    ├── path_planning/                # Navigation components
    │   ├── path_planner.py
    │   ├── path_controller.py
    │   ├── imu_integration.py
    │   └── imu_processor.py
    ├── system_runner/
    │   └── system_runner.py          # Main execution loop
    ├── traffic_rule_processor/
    │   └── traffic_rule_processor.py # Traffic rule decisions
    ├── utils/
    │   ├── vehicle_state.py          # Vehicle state tracking
    │   └── lane_detection_config.py  # Lane detection parameters
    └── web_api/
        └── web_api_server.py         # Web interface API
```

## ⚙️ Workflow

1. Camera captures images
2. Lane detection determines the route
3. Object detection identifies signs, obstacles, pedestrians
4. Path planning calculates optimal trajectory (in AUTO mode)
5. Traffic rule processing makes decisions
6. Vehicle control (speed and steering angle) based on decisions
7. Web interface and video stream provide monitoring and control

## 🌐 Web Interface

The system includes a web interface for monitoring and control:

1. Access the interface at `http://<jetson-ip-address>:8088`
2. Monitor vehicle telemetry and camera feed
3. Switch between operation modes
4. Control the vehicle manually if needed

## 📡 Network Streaming

The system supports streaming processed video for monitoring:

1. On a monitoring computer, run `websocket_server.py` to create a server
2. The video displayed will include processed information (lanes, detected objects, path planning)

## 🕹️ Manual Control

While running, you can use the following keys in the monitoring window:
- `q`: Exit program
- `b`: Emergency brake
- `r`: Continue running at default speed

## ⚙️ Advanced Configuration

Detailed parameters and configurations can be adjusted in these files:
- `config.ini`: Main configuration parameters
- `lane_detection_config.py`: Lane detection parameters
- `traffic_rule_processor.py`: Traffic rule processing
- `car_controller.py`: Hardware control
- `path_planner.py`: Path planning parameters

## ❓ Common Issues

1. **Cannot connect to the vehicle**: Check the serial port in `car_controller.py`
2. **Inaccurate lane detection**: Adjust parameters in `lane_detection_config.py`
3. **Web interface unavailable**: Ensure the correct IP address is being used
4. **Path planning errors**: Review map file in path_planning/maps directory

## 👥 Contributions

All contributions are welcome. Please create an issue or submit a pull request.

## 📜 License

MIT License

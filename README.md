# SPARK - Self-driving Platform for Autonomous Research and Knowledge

SPARK is a self-driving platform designed for researching and developing autonomous driving algorithms on the Jetson Nano. The system combines lane detection, object recognition, and traffic rule processing to control the vehicle autonomously.

## Key Features

- Lane Detection
- Traffic Object Recognition
- Traffic Rule Processing
- Autonomous Parking
- Video streaming to a computer for monitoring

## System Requirements

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

## Installation

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

## Usage

### Running the autonomous vehicle (on Jetson Nano)

```bash
python main.py
```

Options:
- `--camera`: Camera ID (default: 0)
- `--model`: Path to YOLO model (default: uses pre-installed model)
- `--width`: Camera frame width (default: 640)
- `--height`: Camera frame height (default: 480)
- `--server`: Streaming server IP (default: 192.168.163.162)
- `--port`: Streaming server port (default: 8089)
- `--no-stream`: Disable video streaming to laptop

### Running the streaming server (on laptop)

```bash
python streaming_server.py
```

Options:
- `--port`: Port to listen for connections (default: 8089)

## Code Structure

```
SPARK/
├── main.py                 
├── monitoring    
├── requirements.txt  
├── config.ini      
├── README.md   
├── Project status
    ├── project status 1
    ├── project status 2
    ├── project status 3
    ├── qualification             
└── src/                    
    ├── system_runner.py    
    ├── car_controller/     
    │   └── car_controller.py 
    ├── lane_analyzer/     
    │   └── lane_analyzer.py 
    ├── lane_detection/    
    │   ├── lane_detection.py 
    │   └── lane_keeping.py 
    ├── object_detector/   
    │   └── object_detector.py
    ├── traffic_rule_processor/ 
    │   └── traffic_rule_processor.py 
    ├── parking_handler/    
    │   └── parking_handler.py
    ├── autonomous_controller/
    │   └── autonomous_controller.py 
    └── utils/              
        ├── vehicle_state.py 
        └── lane_detection_config.py 
```

## Workflow

1. Camera captures images
2. Lane detection determines the route
3. Object detection identifies signs, obstacles, pedestrians
4. Traffic rule processing makes decisions
5. Vehicle control (speed and steering angle) based on decisions
6. (Optional) Processed video is transmitted to a laptop for monitoring

## Network Streaming

The system supports streaming processed video from the Jetson Nano to a laptop for monitoring:

1. On the laptop, run `streaming_server.py` to create a server that receives the stream
2. On the Jetson, run `main.py` to connect and send the stream
3. The video displayed on the laptop will include processed information (lanes, detected objects)

## Manual Control

While running, you can use the following keys:
- `q`: Exit program
- `b`: Emergency brake
- `r`: Continue running at default speed

## Advanced Configuration

Detailed parameters and configurations can be adjusted in these files:
- `lane_detection_config.py`: Lane detection parameters
- `traffic_rule_processor.py`: Traffic rule processing
- `car_controller.py`: Hardware control

## Common Issues

1. **Cannot connect to the vehicle**: Check the serial port in `car_controller.py`
2. **Inaccurate lane detection**: Adjust parameters in `lane_detection_config.py`
3. **Streaming errors**: Check IP and port, ensure both devices are on the same network

## Contributions

All contributions are welcome. Please create an issue or submit a pull request.

## License

MIT License

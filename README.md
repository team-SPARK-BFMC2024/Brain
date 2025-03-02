# SPARK - Self-driving Platform for Autonomous Research and Knowledge

SPARK là một nền tảng xe tự hành được thiết kế để nghiên cứu và phát triển các thuật toán tự lái trên Jetson Nano. Hệ thống kết hợp phát hiện làn đường, nhận diện đối tượng và xử lý luật giao thông để điều khiển xe một cách tự động.

## Tính năng chính

- Phát hiện làn đường (Lane Detection)
- Nhận diện đối tượng giao thông (Object Detection)
- Xử lý luật giao thông (Traffic Rule Processing)
- Đỗ xe tự động (Autonomous Parking)
- Streaming video về máy tính để giám sát

## Yêu cầu hệ thống

### Phần cứng
- NVIDIA Jetson Nano (4GB RAM)
- Camera USB hoặc Raspberry Pi Camera
- Pin/nguồn điện di động
- Khung xe RC (với bộ điều khiển servo và motor)

### Phần mềm
- JetPack 4.6+ (L4T)
- Python 3.6+
- OpenCV 4.5+
- PyTorch 1.8+ (với CUDA)
- Ultralytics YOLO

Chi tiết phụ thuộc được liệt kê trong file [requirements.txt](requirements.txt).

## Cài đặt

1. Clone repository:
```bash
git clone https://github.com/yourusername/spark.git
cd spark
```

2. Cài đặt các thư viện phụ thuộc:
```bash
pip install -r requirements.txt
```

3. Cấu hình phần cứng:
   - Kết nối camera với Jetson Nano
   - Kết nối bộ điều khiển xe với Jetson Nano thông qua cổng serial

## Sử dụng

### Chạy xe tự hành (trên Jetson Nano)

```bash
python main.py
```

Tùy chọn:
- `--camera`: ID của camera (mặc định: 0)
- `--model`: Đường dẫn đến model YOLO (mặc định: sử dụng model đã cài đặt sẵn)
- `--width`: Chiều rộng khung hình camera (mặc định: 640)
- `--height`: Chiều cao khung hình camera (mặc định: 480)
- `--server`: IP của server streaming (mặc định: 192.168.163.162)
- `--port`: Port của server streaming (mặc định: 8089)
- `--no-stream`: Không stream video về laptop

### Chạy server streaming (trên laptop)

```bash
python streaming_server.py
```

Tùy chọn:
- `--port`: Port để lắng nghe kết nối (mặc định: 8089)

## Cấu trúc mã nguồn

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

## Luồng hoạt động

1. Camera thu nhận hình ảnh
2. Phát hiện làn đường để xác định lộ trình
3. Phát hiện đối tượng để nhận diện biển báo, vật cản, người đi bộ
4. Xử lý luật giao thông để đưa ra quyết định
5. Điều khiển xe (tốc độ và góc lái) dựa trên các quyết định
6. (Tùy chọn) Truyền video đã xử lý về laptop để giám sát

## Network Streaming

Hệ thống hỗ trợ streaming video đã xử lý từ Jetson Nano về laptop để giám sát:

1. Trên laptop, chạy `streaming_server.py` để tạo server nhận stream
2. Trên Jetson, chạy `main.py` để kết nối và gửi stream
3. Video hiển thị trên laptop sẽ bao gồm thông tin đã xử lý (làn đường, đối tượng phát hiện)

## Điều khiển thủ công

Trong khi chạy, bạn có thể sử dụng các phím sau:
- `q`: Thoát chương trình
- `b`: Phanh khẩn cấp
- `r`: Tiếp tục chạy với tốc độ mặc định

## Cấu hình nâng cao

Các thông số và cấu hình chi tiết có thể điều chỉnh trong các file:
- `lane_detection_config.py`: Thông số nhận diện làn đường
- `traffic_rule_processor.py`: Xử lý luật giao thông
- `car_controller.py`: Điều khiển phần cứng

## Các vấn đề thường gặp

1. **Không kết nối được với xe**: Kiểm tra lại cổng serial trong `car_controller.py`
2. **Nhận diện làn đường không chính xác**: Điều chỉnh thông số trong `lane_detection_config.py`
3. **Lỗi streaming**: Kiểm tra IP và port, đảm bảo cả hai thiết bị đều trong cùng mạng

## Đóng góp

Mọi đóng góp đều được hoan nghênh. Vui lòng tạo issue hoặc pull request.

## Giấy phép

MIT License

import asyncio
import websockets
import json
import logging
import threading
import time
from src.mode_controller.mode_controller import OperationMode

class WebSocketServer:
    """
    WebSocket server để xử lý lệnh từ giao diện web.
    
    Server này nhận các lệnh từ giao diện web qua WebSocket và
    chuyển tiếp chúng đến mode_controller.
    """
    
    def __init__(self, mode_controller, port=8765):
        """
        Khởi tạo WebSocket server.
        
        Args:
            mode_controller: Mode controller để điều khiển trạng thái
            port (int): Port để lắng nghe kết nối WebSocket
        """
        self.logger = logging.getLogger('WebSocketServer')
        self.mode_controller = mode_controller
        self.port = port
        self.server = None
        self.is_running = False
        self.clients = set()
        self.server_thread = None
        
        # Khởi tạo từ điển ánh xạ từ chuỗi chế độ sang enum OperationMode
        self.mode_map = {
            'AUTO': OperationMode.AUTO,
            'LEGACY': OperationMode.LEGACY,
            'MANUAL': OperationMode.MANUAL,
            'STOP': OperationMode.STOP
        }
        
        # Giá trị mới nhất để gửi đến clients
        self.current_data = {
            'speed': 0,
            'steering': 0,
            'mode': 'STOP',
            'battery': 0,
            'yaw': 0,
            'position': {'x': 0, 'y': 0}
        }
    
    def start(self):
        """Khởi động WebSocket server trong một thread riêng."""
        if self.is_running:
            return
            
        self.is_running = True
        self.server_thread = threading.Thread(target=self._run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        # Khởi động thread để gửi dữ liệu cập nhật định kỳ
        self.update_thread = threading.Thread(target=self._send_updates)
        self.update_thread.daemon = True
        self.update_thread.start()
        
        self.logger.info(f"WebSocket server started on port {self.port}")
    
    def stop(self):
        """Dừng WebSocket server."""
        if not self.is_running:
            return
            
        self.is_running = False
        
        # Dọn dẹp loop và server asyncio
        if self.server:
            self.server.close()
        
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join(timeout=1.0)
            
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
            
        self.logger.info("WebSocket server stopped")
    
    def _run_server(self):
        """Chạy websocket server."""
        new_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(new_loop)
        
        start_server = websockets.serve(
            self._handle_connection, 
            host="0.0.0.0", 
            port=self.port,
            ping_interval=None  # Tắt ping tự động
        )
        
        self.server = new_loop.run_until_complete(start_server)
        
        try:
            new_loop.run_forever()
        except Exception as e:
            if self.is_running:  # Chỉ log nếu chưa dừng chủ động
                self.logger.error(f"WebSocket server error: {e}")
        finally:
            new_loop.close()
    
    async def _handle_connection(self, websocket, path):
        """Xử lý kết nối WebSocket mới."""
        client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        self.logger.info(f"New client connected: {client_info}")
        
        # Thêm client vào danh sách
        self.clients.add(websocket)
        
        try:
            # Gửi trạng thái hiện tại cho client mới
            await websocket.send(json.dumps(self.current_data))
            
            # Xử lý tin nhắn từ client
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if 'mode' in data:
                        mode_str = data['mode'].upper()
                        if mode_str in self.mode_map:
                            # Chuyển chế độ thông qua mode controller
                            self.mode_controller.set_mode(self.mode_map[mode_str])
                            self.logger.info(f"Mode changed to {mode_str}")
                            
                            # Gửi phản hồi về client
                            response = {'status': 'success', 'mode': mode_str}
                            await websocket.send(json.dumps(response))
                        else:
                            response = {'status': 'error', 'message': f'Unknown mode: {mode_str}'}
                            await websocket.send(json.dumps(response))
                    
                    elif 'power' in data:
                        power_level = int(data['power'])
                        # Xử lý level pin (KL0, KL15, KL30)
                        self.logger.info(f"Power level set to {power_level}")
                        # Gọi hàm xử lý power level ở đây
                        
                        # Gửi phản hồi
                        response = {'status': 'success', 'power': power_level}
                        await websocket.send(json.dumps(response))
                        
                except json.JSONDecodeError:
                    self.logger.warning(f"Invalid JSON from client: {message}")
                    await websocket.send(json.dumps({'status': 'error', 'message': 'Invalid JSON format'}))
                except Exception as e:
                    self.logger.error(f"Error processing message from client: {e}")
                    await websocket.send(json.dumps({'status': 'error', 'message': str(e)}))
        
        except websockets.exceptions.ConnectionClosed:
            self.logger.info(f"Client disconnected: {client_info}")
        finally:
            # Xóa client khỏi danh sách khi ngắt kết nối
            self.clients.remove(websocket)
    
    def update_status(self, data):
        """
        Cập nhật trạng thái hiện tại để gửi đến clients.
        
        Args:
            data (dict): Dữ liệu cập nhật (speed, steering, mode, etc.)
        """
        self.current_data.update(data)
    
    def _send_updates(self):
        """Gửi cập nhật định kỳ đến tất cả clients."""
        while self.is_running:
            if self.clients:  # Chỉ gửi khi có clients kết nối
                # Tạo bản sao của dữ liệu hiện tại
                data_to_send = self.current_data.copy()
                
                # Thêm timestamp
                data_to_send['timestamp'] = time.time()
                
                # Convert to JSON
                message = json.dumps(data_to_send)
                
                # Gửi đến tất cả clients
                asyncio.run(self._broadcast_message(message))
            
            # Chờ 100ms trước khi gửi cập nhật tiếp theo (10Hz)
            time.sleep(0.1)
    
    async def _broadcast_message(self, message):
        """
        Gửi tin nhắn đến tất cả clients.
        
        Args:
            message (str): Tin nhắn để gửi
        """
        if not self.clients:
            return
            
        # Tạo danh sách các coroutines để gửi tin nhắn
        tasks = []
        for client in self.clients.copy():  # Sử dụng bản sao để tránh lỗi khi danh sách thay đổi
            tasks.append(self._send_to_client(client, message))
            
        # Thực thi đồng thời tất cả các tasks
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
    
    async def _send_to_client(self, client, message):
        """
        Gửi tin nhắn đến một client cụ thể, xử lý lỗi nếu có.
        
        Args:
            client (websocket): Client để gửi tin nhắn
            message (str): Tin nhắn để gửi
        """
        try:
            await client.send(message)
        except websockets.exceptions.ConnectionClosed:
            # Client đã ngắt kết nối, sẽ được xóa trong _handle_connection
            pass
        except Exception as e:
            self.logger.error(f"Error sending message to client: {e}")
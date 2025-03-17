import json
import socket
import threading
import logging
from enum import Enum

class OperationMode(Enum):
    """Enum representing different operation modes for the autonomous vehicle."""
    STOP = 0
    MANUAL = 1
    AUTO = 2
    LEGACY = 3

class ModeController:
    """
    Handles different operation modes for the autonomous vehicle based on UI commands.
    
    This controller sets up a socket server to receive mode change commands from
    a web interface, and manages the current operation mode of the vehicle.
    """
    def __init__(self, port=8099):
        """
        Initialize the ModeController.
        
        Args:
            port (int): Port to listen for mode change commands
        """
        self.logger = logging.getLogger('ModeController')
        self.mode = OperationMode.STOP
        self.server_socket = None
        self.listen_thread = None
        self.is_running = False
        self.port = port
        self.on_mode_change_callback = None
        
    def start(self):
        """Start the command server to listen for mode change requests."""
        if self.is_running:
            return
            
        self.is_running = True
        self.listen_thread = threading.Thread(target=self._listen_for_commands)
        self.listen_thread.daemon = True
        self.listen_thread.start()
        self.logger.info(f"Mode controller started, listening on port {self.port}")
        
    def stop(self):
        """Stop the command server."""
        self.is_running = False
        if self.server_socket:
            self.server_socket.close()
        if self.listen_thread and self.listen_thread.is_alive():
            self.listen_thread.join(timeout=1.0)
        self.logger.info("Mode controller stopped")
        
    def set_mode_change_callback(self, callback):
        """
        Set a callback function that will be called when the mode changes.
        
        Args:
            callback: Function to call with new mode as argument when mode changes
        """
        self.on_mode_change_callback = callback
        
    def set_mode(self, mode):
        """
        Set the current operation mode.
        
        Args:
            mode (OperationMode): The new operation mode
        """
        if not isinstance(mode, OperationMode):
            raise ValueError(f"Mode must be an OperationMode enum value, got {type(mode)}")
            
        if self.mode != mode:
            self.logger.info(f"Changing mode from {self.mode.name} to {mode.name}")
            self.mode = mode
            
            # Call the callback if one is registered
            if self.on_mode_change_callback:
                self.on_mode_change_callback(mode)
                
    def get_mode(self):
        """Get the current operation mode."""
        return self.mode
        
    def _listen_for_commands(self):
        """Listen for mode change commands on a socket."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.port))
            self.server_socket.settimeout(1.0)  # 1 second timeout for accept()
            self.server_socket.listen(5)
            
            while self.is_running:
                try:
                    client_socket, addr = self.server_socket.accept()
                    self.logger.info(f"Connected to {addr}")
                    client_socket.settimeout(1.0)
                    
                    try:
                        data = client_socket.recv(1024).decode('utf-8')
                        if data:
                            self._process_command(data)
                    except socket.timeout:
                        pass
                    except Exception as e:
                        self.logger.error(f"Error processing command: {e}")
                    finally:
                        client_socket.close()
                        
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.is_running:
                        self.logger.error(f"Socket error: {e}")
                    break
                    
        except Exception as e:
            self.logger.error(f"Server error: {e}")
        finally:
            if self.server_socket:
                self.server_socket.close()
                
    def _process_command(self, data):
        """
        Process a command received from the socket.
        
        Args:
            data (str): Command data in JSON format
        """
        try:
            # Try to parse as JSON
            command = json.loads(data)
            
            if 'mode' in command:
                mode_str = command['mode'].upper()
                try:
                    # Convert string to enum
                    new_mode = OperationMode[mode_str]
                    self.set_mode(new_mode)
                except KeyError:
                    self.logger.error(f"Unknown mode: {mode_str}")
            else:
                self.logger.warning(f"Received command without mode: {command}")
                
        except json.JSONDecodeError:
            # Simple command format (just the mode name)
            mode_str = data.strip().upper()
            try:
                new_mode = OperationMode[mode_str]
                self.set_mode(new_mode)
            except KeyError:
                self.logger.error(f"Unknown mode: {mode_str}")

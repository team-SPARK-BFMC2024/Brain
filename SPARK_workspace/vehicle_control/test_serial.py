import serial
import time

# Set up the serial connection
# Replace '/dev/ttyAMA0' with your specific serial port (could be /dev/ttyUSB0, etc.)
serial_port = '/dev/ttyACM0'  # Serial port (adjust for your setup)
baud_rate = 115200  # Baud rate (adjust for your device)

# Open the serial port
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Ensure the serial port is open
if ser.is_open:
    print(f"Successfully connected to {serial_port} at {baud_rate} baud.")

response = ser.readline()  # Read the incoming data (one line at a time)
print(f"Received: {response.decode().strip()}")

ser.write(b"#kl:30;;\r\n")
# Wait for a response (if the device sends something back)
time.sleep(1)  # Wait for a second to receive data (adjust as needed)
if ser.in_waiting > 0:
    response = ser.readline()  # Read the incoming data (one line at a time)
    print(f"Received: {response.decode().strip()}")

# ser.write(b"#battery:1;;\r\n")
# # Wait for a response (if the device sends something back)
# time.sleep(1)  # Wait for a second to receive data (adjust as needed)
# if ser.in_waiting > 0:
#     response = ser.readline()  # Read the incoming data (one line at a time)
#     print(f"Received: {response.decode().strip()}")

# ser.write(b"#speed:60;;\r\n")
# # Wait for a response (if the device sends something back)
# time.sleep(1)  # Wait for a second to receive data (adjust as needed)
# if ser.in_waiting > 0:
#     response = ser.readline()  # Read the incoming data (one line at a time)
#     print(f"Received: {response.decode().strip()}")

# # Example: Send a simple command (e.g., 'Hello') to the serial device
# ser.write(b'Hello\n')

# # Wait for a response (if the device sends something back)
# time.sleep(1)  # Wait for a second to receive data (adjust as needed)
# if ser.in_waiting > 0:
#     response = ser.readline()  # Read the incoming data (one line at a time)
#     print(f"Received: {response.decode().strip()}")

# # Close the serial port
# ser.close()
# print(f"Closed connection to {serial_port}")

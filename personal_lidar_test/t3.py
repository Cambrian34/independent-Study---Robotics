import serial
import struct

# Define the data structure according to the frame format
frame_format = 'B B H H H 12H H H B'

# Calculate the CRC8 checksum
def calculate_crc8(data):
    crc = 0
    for byte in data:
        crc = crc ^ byte
        for _ in range(8):
            if (crc & 0x80):
                crc = (crc << 1) ^ 0x31
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

# Open serial connection to the ESP32 (replace with your actual serial port)
ser = serial.Serial('/dev/cu.usbserial-0001', 230400)

# Function to read LiDAR data and print it
def read_lidar_data():
    while True:
        if ser.in_waiting > 0:
            # Read the header byte and validate if it's the expected value (0x54)
            header = ser.read(1)
            if header == b'\x54':
                # Read the rest of the frame data (adjust size based on your LiDAR data structure)
                frame_data = ser.read(37)  # Assuming 37 bytes for the data
                unpacked_data = struct.unpack(frame_format, header + frame_data)

                header = unpacked_data[0]
                ver_len = unpacked_data[1]
                speed = unpacked_data[2]
                start_angle = unpacked_data[3]
                end_angle = unpacked_data[4]
                points = unpacked_data[5:17]  # 12 points in total
                timestamp = unpacked_data[17]
                crc8_received = unpacked_data[18]

                # Print received frame data
                print(f"Header: {header}")
                print(f"Version Length: {ver_len}")
                print(f"Speed: {speed}")
                print(f"Start Angle: {start_angle}")
                print(f"End Angle: {end_angle}")
                print(f"Timestamp: {timestamp}")
                print(f"Points: {points}")
                print(f"Received CRC8: {crc8_received}")

                # Calculate CRC8 from received data (excluding CRC byte)
                crc8_calculated = calculate_crc8([header, ver_len, speed, start_angle, end_angle] + list(points) + [timestamp])

                # Verify if the CRC8 is correct
                if crc8_received == crc8_calculated:
                    print("CRC8 is valid")
                else:
                    print("CRC8 is invalid")

# Start reading LiDAR data
read_lidar_data()
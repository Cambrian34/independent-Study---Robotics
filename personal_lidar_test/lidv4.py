import serial
import struct
import time

SERIAL_PORT = "/dev/tty.usbserial-0001"  # Replace with your actual port
LD06_BAUD_RATE = 230400

FRAME_HEADER = 0x54
FRAME_LENGTH = 42

def calculate_crc8(data):
    """ Compute CRC-8 checksum. Complete table is crucial! """
    CrcTable = [
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
        0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
        0x50, 0x0d, 0xda, 0x87, 0x39, 0x74, 0xa3, 0xee, 0xe2, 0xaf, 0x38, 0x75, 0xd4, 0x99, 0x4e, 0x03,
        0xb6, 0xfb, 0x6c, 0x21, 0x8f, 0xc2, 0x15, 0x58, 0x0c, 0x41, 0x96, 0xdf, 0x71, 0x3c, 0xe5, 0xa8,
        0x45, 0x08, 0xdf, 0x92, 0x2c, 0x61, 0xb6, 0xff, 0xf3, 0xbe, 0x69, 0x24, 0x8a, 0xc7, 0x10, 0x5d,
        0xe8, 0xa5, 0x72, 0x3f, 0x81, 0xcc, 0x1b, 0x56, 0x02, 0x4f, 0x98, 0xd5, 0x60, 0x2d, 0xba, 0xf7,
        0x52, 0x0f, 0xd8, 0x95, 0x2b, 0x66, 0xb1, 0xfc, 0xe4, 0xa9, 0x7e, 0x33, 0x8d, 0xc0, 0x17, 0x5a,
        0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0x57, 0x1a, 0xce, 0x83, 0x23, 0x6e, 0xb9, 0xf4
    ]

    crc = 0
    for byte in data:
        crc = CrcTable[(crc ^ byte) & 0xFF]
    return crc

try:
    ser = serial.Serial(SERIAL_PORT, LD06_BAUD_RATE, timeout=1)
    ser.reset_input_buffer()
    print(f"Connected to {SERIAL_PORT}")

    while True:
        try:
            header_found = False

            while not header_found:
                byte = ser.read(1)
                if not byte:
                    print("Timeout waiting for header.")
                    break
                if byte and byte[0] == FRAME_HEADER:  # Check if byte is not empty before indexing
                    header_found = True
                    break

            if not header_found:
                continue

            remaining_bytes = ser.read(FRAME_LENGTH - 1)
            frame_data = byte + remaining_bytes

            if len(frame_data) != FRAME_LENGTH:
                print(f"Incomplete frame. Expected {FRAME_LENGTH}, got {len(frame_data)}")
                ser.reset_input_buffer()
                continue

            header, data_length, speed, start_angle = struct.unpack("<BBHh", frame_data[:6])

            points = []
            for i in range(12):
                offset = 6 + i * 3
                try:
                    distance, confidence = struct.unpack("<HB", frame_data[offset:offset + 3])
                    points.append((distance, confidence))
                except struct.error as e:
                    print(f"Error unpacking point {i+1}: {e}. Raw data: {' '.join(format(x, '02X') for x in frame_data[offset:offset+3])}")
                    ser.reset_input_buffer()
                    continue

            end_angle, timestamp, crc8 = struct.unpack("<HHLB", frame_data[36:])

            calculated_crc = calculate_crc8(frame_data[:-1])
            if calculated_crc != crc8:
                print(f"CRC Mismatch! Expected: {crc8:02X}, Calculated: {calculated_crc:02X}")
                continue

            print(f"Frame received - Start Angle: {start_angle/100:.2f}°, End Angle: {end_angle/100:.2f}°, Speed: {speed}°/s")
            for i, (dist, conf) in enumerate(points):
                print(f"  Point {i+1}: Distance = {dist} mm, Confidence = {conf}")

        except serial.SerialException as e:
            print(f"Serial Error: {e}")
            break
        except struct.error as e:
            print(f"Struct Error: {e}")
            ser.reset_input_buffer()

except KeyboardInterrupt:
    print("\nStopping script.")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial connection closed.")
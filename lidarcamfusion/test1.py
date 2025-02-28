import cv2
import numpy as np
import serial
import pyLidar3
import time
import math

# Initialize Camera
cap = cv2.VideoCapture(0)  # Change index if needed

# Initialize LiDAR
port = "/dev/ttyUSB0"  # Adjust based on your setup
lidar = pyLidar3.YdLidarX4(port)

# Frame dimensions (adjust based on camera resolution)
WIDTH, HEIGHT = 640, 480
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2

# LiDAR parameters
LIDAR_MAX_RANGE = 4000  # mm
SCALE = HEIGHT / (2 * LIDAR_MAX_RANGE)  # Scale LiDAR data to fit frame

if lidar.Connect():
    print("LiDAR connected. Starting scan...")
    try:
        gen = lidar.StartScanning()
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Camera error")
                break

            # Convert frame to grayscale
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2BGR)

            # Read LiDAR data
            lidar_data = next(gen)

            for angle in range(0, 360):
                distance = lidar_data[angle]
                if distance > 0 and distance < LIDAR_MAX_RANGE:
                    # Convert polar to Cartesian
                    theta = math.radians(angle)
                    x = int(CENTER_X + distance * SCALE * math.cos(theta))
                    y = int(CENTER_Y - distance * SCALE * math.sin(theta))

                    # Draw LiDAR point on frame
                    cv2.circle(frame, (x, y), 2, (0, 255, 0), -1)

            # Display result
            cv2.imshow("LiDAR + Camera Fusion", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        lidar.StopScanning()
        lidar.Disconnect()
else:
    print("LiDAR connection failed.")
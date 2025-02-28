import cv2
import numpy as np
import PyLidar3 as pyLidar3
import time
import math
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Initialize Camera
cap = cv2.VideoCapture(0)  # Change index if needed

# Initialize LiDAR
port = "/dev/ttyUSB0"  # Adjust based on your setup
lidar = pyLidar3.YdLidarX4(port)

# Frame dimensions (adjust based on camera resolution)
WIDTH, HEIGHT = 320, 240  # Reduced resolution for faster processing
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2

# LiDAR parameters
LIDAR_MAX_RANGE = 4000  # mm
SCALE = HEIGHT / (2 * LIDAR_MAX_RANGE)  # Scale LiDAR data to fit frame

# Shared data between threads
lidar_data = [0 for _ in range(360)]
x = np.array([0 for _ in range(360)])
y = np.array([0 for _ in range(360)])
is_run = True

# PID controller parameters (you can add PID control if needed for the robot)
Kp = 1.0
Ki = 0.0
Kd = 0.0
setpoint = 1500  # desired distance from the wall in mm
integral = 0
previous_error = 0

# Function to read LiDAR data in a separate thread
def read_lidar():
    global lidar_data, x, y, is_run
    if lidar.Connect():
        print("LiDAR connected. Starting scan...")
        gen = lidar.StartScanning()
        while is_run:
            lidar_data = next(gen)
            for angle in range(0, 360):
                if lidar_data[angle] > 0 and lidar_data[angle] < LIDAR_MAX_RANGE:
                    x[angle] = lidar_data[angle] * math.cos(math.radians(angle))
                    y[angle] = lidar_data[angle] * math.sin(math.radians(angle))
            time.sleep(0.05)  # Reduce reading frequency
    else:
        print("LiDAR connection failed.")

# Function for the animation of LiDAR data in real-time
def all_realtime_animation():
    fig, ax = plt.subplots(nrows=1, figsize=(7, 7))

    def animate(i):
        ax.clear()
        ax.set_ylim(-9000, 9000)
        ax.set_xlim(-9000, 9000)
        ax.scatter(x, y, c='r', s=8)

    ani = animation.FuncAnimation(fig, animate, interval=4)
    plt.show()

# Main loop for capturing and processing frames with LiDAR data
def main_loop():
    global is_run

    # Start LiDAR data reading in a separate thread
    lidar_thread = threading.Thread(target=read_lidar, daemon=True)
    lidar_thread.start()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Camera error")
                break

            # Convert frame to grayscale and then back to BGR (to keep it in color)
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2BGR)

            # Overlay LiDAR points on the frame
            for angle in range(0, 360, 2):  # Skip some angles to reduce load
                distance = lidar_data[angle]
                if 0 < distance < LIDAR_MAX_RANGE:
                    # Convert polar to Cartesian
                    theta = math.radians(angle)
                    x_coord = int(CENTER_X + distance * SCALE * math.cos(theta))
                    y_coord = int(CENTER_Y - distance * SCALE * math.sin(theta))

                    # Draw LiDAR point on frame
                    cv2.circle(frame, (x_coord, y_coord), 2, (0, 255, 0), -1)

            # Display the fused result of Camera and LiDAR
            cv2.imshow("LiDAR + Camera Fusion", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        is_run = False
        cap.release()
        cv2.destroyAllWindows()
        lidar.StopScanning()
        lidar.Disconnect()

if __name__ == "__main__":
    # Start the LiDAR animation in a separate thread
    animation_thread = threading.Thread(target=all_realtime_animation, daemon=True)
    animation_thread.start()

    # Run the main loop for capturing frames
    main_loop()
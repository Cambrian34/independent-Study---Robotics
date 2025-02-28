import threading
import PyLidar3
import math    
import time
import numpy as np
import matplotlib.pyplot as plt

from mySerCommLibrary import SerialComm  # Importing the movement class

# LiDAR Configuration
port = "/dev/tty.usbserial-0001"  # Adjust the port as needed
obj = PyLidar3.YdLidarG4(port)

# PID controller parameters
Kp = 1.0
Ki = 0.0
Kd = 0.0
setpoint = 1500  # Desired distance from the wall in mm
integral = 0
previous_error = 0

plt.ion()  # Enable interactive mode for real-time plotting

# Initialize Serial Communication for movement
robot = SerialComm()
robot.initSerComm()  # Start handshaking with Arduino

def pid_control(error):
    """PID Controller to adjust robot movement"""
    global integral, previous_error
    integral += error
    derivative = error - previous_error
    output = Kp * error + Ki * integral + Kd * derivative
    previous_error = error
    return output 

# Visualization function for LiDAR data
def visualize_lidar(data):
    """Visualize LiDAR scan data in polar coordinates."""
    #angles = np.linspace(0, 360, len(data), endpoint=False)  # Angle range from 0 to 360
    #distances = np.array(data)  # LiDAR distances
    angles = np.array(list(data.keys()))
    distances = np.array(list(data.values()))
    
    plt.clf()
    plt.polar(np.deg2rad(angles), distances)
    plt.title("Real-Time LiDAR Scan")
    plt.draw()
    plt.pause(0.1)

def lidar_navigation():
    if obj.Connect():
        print(obj.GetDeviceInfo())
        gen = obj.StartScanning()
        while True:
            try:
                data = next(gen)
                visualize_lidar(data)  # Add visualization
                
                front_distance = data[0]  # 0° (front)
                right_distance = data[90]  # 90° (right side)

                # Compute error (difference from setpoint)
                error = setpoint - right_distance
                correction = pid_control(error)

                # Determine movement based on correction
                if abs(error) < 50:  # If within range, move forward
                    robot.moveForward(15)
                elif error > 0:  # Too far, turn left, then stop
                    robot.turnLeft(15)
                    time.sleep(0.5)  # Allow time for turning
                    robot.stopMoving()
                else:  # Too close, turn right, then stop
                    robot.turnRight(15)
                    time.sleep(0.5)  # Allow time for turning
                    robot.stopMoving()
                
                time.sleep(0.1)  # Small delay for stable control loop

            except Exception as e:
                print(f"Error: {e}")
                break
    else:
        print("Failed to connect to LiDAR.")

if __name__ == "__main__":
    try:
        lidar_thread = threading.Thread(target=lidar_navigation, daemon=True)
        lidar_thread.start()
    
        while True:
            time.sleep(1)  # Keep main thread alive
        
    except KeyboardInterrupt:
        print("\nStopping LiDAR...")
        obj.StopScanning()
        obj.Disconnect()
        robot.stopMoving()
        print("Shutdown complete.")

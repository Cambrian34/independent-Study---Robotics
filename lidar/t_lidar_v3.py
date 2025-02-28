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
                visualize_lidar(data)

                # Convert LiDAR readings from mm to cm
                front_distance_lidar = data.get(0, 3000) / 10  # LiDAR at 0° (front) in cm
                right_distance = data.get(90, 3000) / 10  # LiDAR at 90° (right wall) in cm

                # Get Ultrasonic front distance
                front_distance_ultrasonic = robot.readSonicCM(1)  # Assuming port 1

                # Merge sensor readings for better accuracy
                front_distance = min(front_distance_lidar, front_distance_ultrasonic)

                print(f"LiDAR Front: {front_distance_lidar:.1f} cm | Ultrasonic Front: {front_distance_ultrasonic} cm | Final Front Distance: {front_distance:.1f} cm | Right: {right_distance:.1f} cm")

                # Compute error for PID wall-following
                error = setpoint / 10 - right_distance  # Convert setpoint to cm
                correction = pid_control(error)

                # 🚨 Obstacle Avoidance with Both Sensors
                if front_distance < 30:  # If an obstacle is within 30cm
                    print("Obstacle detected! Stopping...")
                    robot.stopMoving()
                    time.sleep(0.2)

                    # Check left, right, and back clearances to decide which way to turn
                    left_clearance = data.get(135, 3000) / 10  # Left (135°)
                    right_clearance = data.get(45, 3000) / 10  # Right (45°)
                    left_back_clearance = data.get(225, 3000) / 10  # Left back (225°)
                    right_back_clearance = data.get(315, 3000) / 10  # Right back (315°)

                    # Print all clearance values for debugging
                    print(f"Left: {left_clearance:.1f} cm | Right: {right_clearance:.1f} cm | Left Back: {left_back_clearance:.1f} cm | Right Back: {right_back_clearance:.1f} cm")

                    # Decide where to turn based on available space
                    if left_clearance > right_clearance:
                        print("Turning Left...")
                        robot.turnLeft(15)
                    elif right_clearance > left_clearance:
                        print("Turning Right...")
                        robot.turnRight(15)
                    else:  # If both left and right are the same, check back distances
                        if left_back_clearance > right_back_clearance:
                            print("Turning Left (Back) ...")
                            robot.turnLeft(15)
                        else:
                            print("Turning Right (Back) ...")
                            robot.turnRight(15)

                    time.sleep(0.5)  # Allow time for turning
                    robot.stopMoving()

                # **Wall Following with PID**
                elif abs(error) < 5:  # Converted threshold to cm
                    robot.moveForward(15)
                elif error > 0:
                    robot.turnLeft(15)
                    time.sleep(0.5)
                    robot.stopMoving()
                else:
                    robot.turnRight(15)
                    time.sleep(0.5)
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

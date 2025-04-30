import threading
import PyLidar3
import math    
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


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


#lidar data 
lidar_data = {}

def visualize_lidar(data):
    """Stores LiDAR data for the animation function."""
    global lidar_data
    lidar_data = data  # Store the latest LiDAR data

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
    x = []
    y = []
    for angle, distance in data.items():
        if distance > 100:  # Filter out very small distances (noise)
            rad = math.radians(angle)
            x.append(distance * math.cos(rad))
            y.append(distance * math.sin(rad))
    
    plt.clf()
    plt.scatter(x, y, c='r', s=8)
    plt.xlim(-9000, 9000)
    plt.ylim(-9000, 9000)
    plt.title("Real-Time LiDAR Scan")
    plt.draw()
    plt.pause(0.1)

import matplotlib.animation as animation

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

def update(frame):
    """Updates the LiDAR visualization in the main thread."""
    global lidar_data
    ax.clear()
    
    x = []
    y = []
    for angle, distance in lidar_data.items():
        if distance > 100:  # Remove noise
            rad = math.radians(angle)
            x.append(distance * math.cos(rad))
            y.append(distance * math.sin(rad))
    
    ax.scatter(x, y, c='r', s=8)
    ax.set_xlim(-9000, 9000)
    ax.set_ylim(-9000, 9000)
    ax.set_title("Real-Time LiDAR Scan")

ani = animation.FuncAnimation(fig, update, interval=100)


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

                # Merge sensor readings for better accuracy
                front_distance = front_distance_lidar

                print(f"LiDAR Front: {front_distance_lidar:.1f} cm | Final Front Distance: {front_distance:.1f} cm | Right: {right_distance:.1f} cm")

                # Compute error for PID wall-following
                error = setpoint / 10 - right_distance  # Convert setpoint to cm
                correction = pid_control(error)

                # Obstacle Avoidance with Both Sensors
                if front_distance < 30:  # If an obstacle is within 30cm
                    print("Obstacle detected! Stopping...")
                    # robot.stop_robot()
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
                        # robot.turnLeft(15)
                    elif right_clearance > left_clearance:
                        print("Turning Right...")
                        # robot.turnRight(15)
                    else:  # If both left and right are the same, check back distances
                        if left_back_clearance > right_back_clearance:
                            print("Turning Left (Back) ...")
                            # robot.turnLeft(15)
                        else:
                            print("Turning Right (Back) ...")
                            # robot.turnRight(15)

                    time.sleep(0.5)  # Allow time for turning
                    # robot.stop_robot()

                # **Wall Following with PID**
                elif abs(error) < 5:  # Converted threshold to cm
                    # robot.moveForward(15)
                    print("Moving Forward")
                    pass
                elif error > 0:
                    # robot.turnLeft(15)
                    print("Turning Left")
                    time.sleep(0.5)
                    # robot.stop_robot()
                else:
                    print("Turning Right")
                    # robot.turnRight(15)
                    time.sleep(0.5)
                    # robot.stop_robot()

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

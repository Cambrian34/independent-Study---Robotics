import threading
import PyLidar3
import math    
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import asyncio

from mySerCommLibrary import SerialComm  # Importing the movement class

#downside of this code is that it is not able to run the animation and the lidar at the same time , due to cpu limitations
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

# Initialize Serial Communication for movement
robot = SerialComm()
robot.initSerComm()  # Start handshaking with Arduino

# Shared data storage for visualization
lidar_data = {}

def pid_control(error):
    """PID Controller to adjust robot movement"""
    global integral, previous_error
    integral += error
    derivative = error - previous_error
    output = Kp * error + Ki * integral + Kd * derivative
    previous_error = error
    return output 

# Function to store LiDAR data
def visualize_lidar(data):
    """Stores LiDAR data for the animation function."""
    global lidar_data
    lidar_data = data  # Store the latest LiDAR data

def lidar_navigation():
    """Handles LiDAR scanning and robot navigation."""
    if obj.Connect():
        print(obj.GetDeviceInfo())
        gen = obj.StartScanning()
        while True:
            try:
                data = next(gen)
                visualize_lidar(data)  # Update shared lidar_data dictionary

                # Convert LiDAR readings from mm to cm
                front_distance = data.get(0, 3000) / 10  # LiDAR at 0° (front) in cm
                right_distance = data.get(90, 3000) / 10  # LiDAR at 90° (right) in cm
                left_clearance = data.get(135, 3000) / 10  # Left (135°)
                right_clearance = data.get(45, 3000) / 10  # Right (45°)
                left_back_clearance = data.get(225, 3000) / 10  # Left back (225°)
                right_back_clearance = data.get(315, 3000) / 10  # Right back (315°)

                print(f"LiDAR Readings: Front={front_distance:.1f}cm | Right={right_distance:.1f}cm")
                print(f"Left={left_clearance:.1f}cm | Right={right_clearance:.1f}cm | Left Back={left_back_clearance:.1f}cm | Right Back={right_back_clearance:.1f}cm")

                # **Obstacle Avoidance Logic**
                if front_distance < 30:  # If an obstacle is within 30cm
                    print("Obstacle detected! Stopping...")
                    # robot.stop_robot()
                    time.sleep(0.2)

                    # Decide where to turn
                    if left_clearance > 30 and left_clearance > right_clearance:
                        print("Turning Left...")
                        # robot.turnLeft(20)
                    elif right_clearance > 30:
                        print("Turning Right...")
                        # robot.turnRight(20)
                    else:  # If both are blocked, check the back directions
                        if left_back_clearance > right_back_clearance:
                            print("Turning Left (Back) ...")
                            # robot.turnLeft(30)
                        else:
                            print("Turning Right (Back) ...")
                            # robot.turnRight(30)

                    time.sleep(0.5)  # Allow time for turning
                    #robot.moveForward(15)  # Resume movement

                # **Wall Following with PID**
                else:
                    error = setpoint / 10 - right_distance  # Convert setpoint to cm
                    correction = pid_control(error)

                    if abs(error) < 5:
                        # robot.moveForward(15)
                        print("Moving Forward")
                    elif error > 0:
                        # robot.turnLeft(min(20, abs(correction)))
                        print("Turning Left")
                    else:
                        # robot.turnRight(min(20, abs(correction)))
                        print("Turning Right")

                time.sleep(0.1)  # Small delay for stable control loop

            except Exception as e:
                print(f"Error: {e}")
                break
    else:
        print("Failed to connect to LiDAR.")
#  MATPLOTLIB VISUALIZATION IN MAIN THREAD  #

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

# ----------------- RUN THREADS & MAIN EVENT LOOP ----------------- #

if __name__ == "__main__":
    try:
        # Fix threading issue with asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        lidar_thread = threading.Thread(target=lidar_navigation, daemon=True)
        lidar_thread.start()
        
        plt.show()  # Keeps Matplotlib running in the main thread

    except KeyboardInterrupt:
        print("\nStopping LiDAR...")
        obj.StopScanning()
        obj.Disconnect()
        robot.stopMoving()
        print("Shutdown complete.")
import threading
"""
This script integrates LiDAR-based navigation and camera-based image classification for a robotic system.
Modules:
- threading: For running LiDAR navigation and camera classification concurrently.
- PyLidar3: For interfacing with the YdLidarG4 LiDAR sensor.
- math, time, numpy, matplotlib.pyplot, cv2: For various mathematical operations, timing, array manipulations, plotting, and computer vision tasks.
- mediapipe: For image classification using MediaPipe.
- mySerCommLibrary: For serial communication with the robot.
Classes:
- None
Functions:
- dealWithResult(resultname): Handles the robot's actions based on the classification result.
- pid_control(error): Computes the PID control output based on the error.
- visualize_lidar(data): Visualizes LiDAR data in real-time using a polar plot.
- lidar_navigation(): Manages LiDAR-based navigation, including real-time scanning and movement control.
- camera_classification(model, max_results, score_threshold, camera_id, width, height): Manages camera-based image classification using a specified model.
Usage:
- The script starts LiDAR navigation and camera classification in separate threads.
- The robot's movement is controlled based on LiDAR data and image classification results.
- The script can be stopped using a keyboard interrupt, which safely shuts down the system.
"""
import PyLidar3
import math    
import time
import numpy as np
import matplotlib.pyplot as plt
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
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


lock = threading.Lock()
# Image Classification Setup
def dealWithResult(resultname):
    with lock:
        if resultname == "red":
            print("Red - Pausing for 3 seconds")
            robot.stopMoving()
            time.sleep(3)
        elif resultname == "green":
            print("Green - Moving forward")
            robot.moveForward(15)
        elif resultname == "stop":
            print("Stop - Waiting for green light")
            robot.stopMoving()
        elif resultname == "yield":
            print("Yield - Moving slowly")
            robot.moveForward(5)
        elif resultname == "speed55":
            print("Speed 55 - Moving faster")
            robot.moveForward(25)
        elif resultname == "speed35":
            print("Speed 35 - Moving slower")
            robot.moveForward(10)
        elif resultname == "pedestrian":
            print("Pedestrian - Stopping until clear")
            robot.stopMoving()
        elif resultname == "right":
            print("Right - Preparing to turn")
            robot.turnRight(15)
            time.sleep(1)
            robot.moveForward(15)

# PID Controller
def pid_control(error):
    global integral, previous_error
    integral += error
    derivative = error - previous_error
    output = Kp * error + Ki * integral + Kd * derivative
    previous_error = error
    return output 

# LiDAR Visualization
def visualize_lidar(data):
    angles = np.array(list(data.keys()))
    distances = np.array(list(data.values()))
    plt.clf()
    plt.polar(np.deg2rad(angles), distances)
    plt.title("Real-Time LiDAR Scan")
    plt.draw()
    plt.pause(0.1)

# LiDAR Navigation
def lidar_navigation():
    if obj.Connect():
        print(obj.GetDeviceInfo())
        gen = obj.StartScanning()
        
        while True:
            try:
                data = next(gen)
                visualize_lidar(data)  # LiDAR visualization
                
                front_distance = data[0]  # 0° (front)
                right_distance = data[90]  # 90° (right side)
                
                # Read ultrasonic sensor (Assuming port is 1)
                ultrasonic_distance = int(robot.readSonicIN(1))

                print(f"Ultrasonic Distance: {ultrasonic_distance} mm")
                
                # Collision Avoidance: If an object is too close, stop
                if ultrasonic_distance < 5:
                    print("Obstacle detected! Stopping robot.")
                    robot.stopMoving()
                    time.sleep(0.5)  # Pause before re-evaluating
                    continue  # Skip further processing for this loop iteration

                # Compute error (distance from setpoint)
                error = setpoint - right_distance
                correction = pid_control(error)

                # Determine movement based on correction
                if abs(error) < 50:  # If within range, move forward
                    robot.moveForward(15)
                elif error > 0:  # Too far, turn left
                    robot.turnLeft(15)
                    time.sleep(0.5)
                    robot.stopMoving()
                else:  # Too close, turn right
                    robot.turnRight(15)
                    time.sleep(0.5)
                    robot.stopMoving()
                
                time.sleep(0.1)  # Small delay for stability

            except Exception as e:
                print(f"Error: {e}")
                break
    else:
        print("Failed to connect to LiDAR.")


# Camera-Based Image Classification
def camera_classification(model='model.tflite', max_results=1, score_threshold=0.0, camera_id=0, width=224, height=224):
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def save_result(result, unused_output_image, timestamp_ms):
        if result.classifications:
            category = result.classifications[0].categories[0]
            dealWithResult(category.category_name)

    base_options = python.BaseOptions(model_asset_path=model)
    options = vision.ImageClassifierOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.LIVE_STREAM,
        max_results=max_results,
        score_threshold=score_threshold,
        result_callback=save_result)
    classifier = vision.ImageClassifier.create_from_options(options)

    while cap.isOpened():
        success, image = cap.read()
        if not success:
            break
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
        classifier.classify_async(mp_image, time.time_ns() // 1_000_000)
        if cv2.waitKey(1) == 27:
            break
    classifier.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        lidar_thread = threading.Thread(target=lidar_navigation, daemon=True)
        camera_thread = threading.Thread(target=camera_classification, daemon=True)
        
        lidar_thread.start()
        camera_thread.start()
        
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping system...")
        obj.StopScanning()
        obj.Disconnect()
        robot.stopMoving()
        print("Shutdown complete.")
        plt.close()
        cv2.destroyAllWindows()
        lidar_thread.stop()
        camera_thread.stop()
        exit()


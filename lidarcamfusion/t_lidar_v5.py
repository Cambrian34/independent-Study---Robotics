import threading
import PyLidar3
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import asyncio
import logging

from mySerCommLibrary import SerialComm

# Configuration 
LIDAR_PORT = "/"
PID_KP = 1.0
PID_KI = 0.0
PID_KD = 0.0
OBSTACLE_DISTANCE_THRESHOLD = 30  # mm
ROBOT_SPEED = 15
VISUALIZATION_INTERVAL = 400 #ms

# Logging setup
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class LidarNavigation:
    def __init__(self, lidar_port, pid_kp, pid_ki, pid_kd,  obstacle_threshold, robot_speed):
        self.lidar = PyLidar3.YdLidarG4(lidar_port)
        self.robot = SerialComm()
        self.robot.initSerComm()
        self.pid_kp = pid_kp
        self.pid_ki = pid_ki
        self.pid_kd = pid_kd
        self.obstacle_threshold = obstacle_threshold
        self.robot_speed = robot_speed
        self.integral = 0
        self.previous_error = 0
        self.lidar_data = {}
        self.x_data = []
        self.y_data = []

    def pid_control(self, error):
        self.integral += error
        derivative = error - self.previous_error
        output = self.pid_kp * error + self.pid_ki * self.integral + self.pid_kd * derivative
        self.previous_error = error
        return output


    def navigate(self):
        if self.lidar.Connect():
            logging.info(self.lidar.GetDeviceInfo())
            gen = self.lidar.StartScanning()

            while True:
                try:
                    data = next(gen)

                    front_distance_lidar = data.get(0, 3000)

                    logging.info(f"LiDAR Front: {front_distance_lidar:.1f} mm")

                    if front_distance_lidar < self.obstacle_threshold:
                        logging.info("Obstacle detected! Turning...")

                        left_clearance = data.get(135, 3000)
                        right_clearance = data.get(45, 3000)
                        left_back_clearance = data.get(225, 3000)
                        right_back_clearance = data.get(315, 3000)

                        if left_clearance > right_clearance:
                            logging.info("Turning Left (Obstacle)")
                            #self.robot.turnLeft(15)
                        elif right_clearance > left_clearance:
                            logging.info("Turning Right (Obstacle)")
                            #self.robot.turnRight(15)

                        else:
                            if left_back_clearance > right_back_clearance:
                                logging.info("Turning Left (Back, Obstacle) ...")
                                #self.robot.turnLeft(15)
                            else:
                                logging.info("Turning Right (Back, Obstacle) ...")
                                #self.robot.turnRight(15)


                        time.sleep(0.5)

                    else:
                        logging.info("Moving Forward")
                        #self.robot.moveForward(15)

                    time.sleep(0.1)

                except Exception as e:
                    logging.error(f"Error: {e}")
                    break
        else:
            logging.error("Failed to connect to LiDAR.")




if __name__ == "__main__":
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        lidar_nav = LidarNavigation(LIDAR_PORT, PID_KP, PID_KI, PID_KD, OBSTACLE_DISTANCE_THRESHOLD, ROBOT_SPEED)

        lidar_thread = threading.Thread(target=lidar_nav.navigate, daemon=True)
        lidar_thread.start()


    except KeyboardInterrupt:
        logging.info("\nStopping LiDAR...")
        lidar_nav.lidar.StopScanning()
        lidar_nav.lidar.Disconnect()
        lidar_nav.robot.stop_robot()
        #stops the lidar_thread if it is still running
        if lidar_thread.is_alive():
            lidar_thread.join()

        logging.info("Shutdown complete.")
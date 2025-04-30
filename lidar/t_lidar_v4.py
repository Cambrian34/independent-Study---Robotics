import argparse
import sys
import time
import PyLidar3
import threading
import logging

from mySerCommLibrary import SerialComm
from movementclass import MovementController

# Configuration
LIDAR_PORT = "/"
OBSTACLE_DISTANCE_THRESHOLD = 80  # mm

# Logging setup
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class LidarNavigation:
    def __init__(self, lidar_port, obstacle_threshold, movementcontroller):
        self.lidar = PyLidar3.YdLidarG4(lidar_port)
        self.obstacle_threshold = obstacle_threshold
        self.movement_controller = movementcontroller

    def navigate(self):
        if self.lidar.Connect():
            logging.info(self.lidar.GetDeviceInfo())
            gen = self.lidar.StartScanning()

            while True:
                try:
                    data = next(gen)
                    front_distance_lidar = data.get(0, 3000)
                    left_front_clearance = data.get(15, 3000)
                    right_front_clearance = data.get(345, 3000)
                    logging.info(f"LiDAR Front: {front_distance_lidar:.1f} mm, Left Front: {left_front_clearance:.1f} mm, Right Front: {right_front_clearance:.1f} mm")

                    if (front_distance_lidar < self.obstacle_threshold or
                        left_front_clearance < self.obstacle_threshold or
                        right_front_clearance < self.obstacle_threshold):
                        logging.info("Obstacle detected! Backing up to create space.")
                        self.movement_controller.add_command("backward")
                        time.sleep(2.0)
                        self.movement_controller.add_command("stop")

                        left_clearance = data.get(135, 3000)
                        right_clearance = data.get(45, 3000)
                        left_back_clearance = data.get(225, 3000)
                        right_back_clearance = data.get(315, 3000)
                        

                        if left_clearance > right_clearance:
                            logging.info("Turning Left (Obstacle)")
                            self.movement_controller.add_command("left")
                        elif right_clearance > left_clearance:
                            logging.info("Turning Right (Obstacle)")
                            self.movement_controller.add_command("right")
                        else:
                            if left_back_clearance > right_back_clearance:
                                logging.info("Turning Left (Back, Obstacle) ...")
                                self.movement_controller.add_command("left")
                            else:
                                logging.info("Turning Right (Back, Obstacle) ...")
                                self.movement_controller.add_command("right")

                        time.sleep(0.5)
                        self.movement_controller.add_command("stop")

                    else:
                        logging.info("Moving Forward")
                        self.movement_controller.add_command("forward")

                    time.sleep(0.1)

                except KeyboardInterrupt:
                    print("\nStopping LiDAR...")
                    self.lidar.StopScanning()
                    self.lidar.Disconnect()
                    self.movement_controller.add_command("stop")
                    print("Shutdown complete.")

                except Exception as e:
                    logging.error(f"Error: {e}")
                    break
        else:
            logging.error("Failed to connect to LiDAR.")

def main():
    serial_comm = SerialComm()
    serial_comm.initSerComm()

    movement_controller = MovementController(serial_comm)

    lidar_navigation = LidarNavigation(
        lidar_port=LIDAR_PORT,
        obstacle_threshold=OBSTACLE_DISTANCE_THRESHOLD,
        movementcontroller=movement_controller
    )

    lidar_navigation.navigate()

if __name__ == "__main__":
    main()

import time
import PyLidar3
import logging
from mySerCommLibrary import SerialComm
from movementclass import MovementController

# Configuration
LIDAR_PORT = "/dev/ttyUSB0"  # Update this for your system
OBSTACLE_DISTANCE_THRESHOLD = 80  # mm
MIN_TURN_DISTANCE = 150  # mm
REVERSE_DURATION = 1.5  # Time in seconds for reversing before turning
MOVE_DELAY = 0.1  # Small delay to avoid rapid command switching

# Logging setup
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

class LidarNavigation:
    def __init__(self, lidar_port, obstacle_threshold, movement_controller):
        self.lidar = PyLidar3.YdLidarG4(lidar_port)
        self.obstacle_threshold = obstacle_threshold
        self.movement_controller = movement_controller

    def navigate(self):
        """Main navigation loop using LiDAR for obstacle detection."""
        if self.lidar.Connect():
            logging.info(self.lidar.GetDeviceInfo())
            scan_data = self.lidar.StartScanning()

            try:
                while True:
                    data = next(scan_data)

                    # Read distances
                    front_distance = data.get(0, 3000)
                    left_distance = data.get(45, 3000)
                    right_distance = data.get(315, 3000)
                    back_left = data.get(225, 3000)
                    back_right = data.get(135, 3000)

                    logging.info(
                        f"LiDAR -> Front: {front_distance} mm, Left: {left_distance} mm, "
                        f"Right: {right_distance} mm, Back-Left: {back_left} mm, Back-Right: {back_right} mm"
                    )

                    if front_distance < self.obstacle_threshold:
                        logging.info("Obstacle detected ahead! Evaluating best path...")

                        # Stop first
                        self.movement_controller.add_command("stop")

                        # Determine best turn direction
                        if left_distance > MIN_TURN_DISTANCE and right_distance > MIN_TURN_DISTANCE:
                            if left_distance > right_distance:
                                logging.info("Turning Left")
                                self.movement_controller.add_command("left")
                            else:
                                logging.info("Turning Right")
                                self.movement_controller.add_command("right")
                        elif left_distance > MIN_TURN_DISTANCE:
                            logging.info("Turning Left")
                            self.movement_controller.add_command("left")
                        elif right_distance > MIN_TURN_DISTANCE:
                            logging.info("Turning Right")
                            self.movement_controller.add_command("right")
                        else:
                            logging.info("Reversing before turning...")
                            self.movement_controller.add_command("backward")
                            time.sleep(REVERSE_DURATION)

                            # Decide turn after reversing
                            if back_left > back_right:
                                logging.info("Turning Left after Reversing")
                                self.movement_controller.add_command("left")
                            else:
                                logging.info("Turning Right after Reversing")
                                self.movement_controller.add_command("right")

                    else:
                        logging.info("Path clear. Moving forward.")
                        self.movement_controller.add_command("forward")

                    time.sleep(MOVE_DELAY)

            except KeyboardInterrupt:
                logging.info("Stopping navigation...")
                self.lidar.StopScanning()
                self.lidar.Disconnect()
                self.movement_controller.add_command("stop")
                logging.info("Shutdown complete.")

            except Exception as e:
                logging.error(f"Error: {e}")
                self.lidar.StopScanning()
                self.lidar.Disconnect()

        else:
            logging.error("Failed to connect to LiDAR.")

def main():
    serial_comm = SerialComm()
    serial_comm.initSerComm()

    movement_controller = MovementController(serial_comm)

    lidar_navigation = LidarNavigation(
        lidar_port=LIDAR_PORT,
        obstacle_threshold=OBSTACLE_DISTANCE_THRESHOLD,
        movement_controller=movement_controller
    )

    lidar_navigation.navigate()

if __name__ == "__main__":
    main()
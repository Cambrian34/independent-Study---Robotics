import argparse
import sys
import time
import PyLidar3
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks import vision
import threading
import logging

from mySerCommLibrary import SerialComm
from movementclass import MovementController

# Configuration
LIDAR_PORT = "/"
OBSTACLE_DISTANCE_THRESHOLD = 800  #mm

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

class ImageClassifierRobot:
    def __init__(self, model: str, max_results: int, score_threshold: float, camera_id: int, width: int, height: int):
        self.model = model
        self.max_results = max_results
        self.score_threshold = score_threshold
        self.camera_id = camera_id
        self.width = width
        self.height = height
        
        self.COUNTER = 0
        self.FPS = 0
        self.START_TIME = time.time()
        self.classification_result_list = []

        self.robot = SerialComm()
        self.robot.initSerComm()

        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def deal_with_result(self, result_name):
        actions = {
            "red": self.robot.stop_robot,
            "green": lambda: self.robot.moveForward(15),
            "stop": self.robot.stop_robot,
            "yield": lambda: self.robot.moveForward(10),
            "speed55": lambda: self.robot.moveForward(25),
            "speed35": lambda: self.robot.moveForward(20),
            "pedestrian": self.robot.stop_robot,
            "right": lambda: print("Turning right at the next intersection?"),
        }
        
        if result_name in actions:
            print(result_name)
            actions[result_name]()

    def save_result(self, result: vision.ImageClassifierResult, unused_output_image: mp.Image, timestamp_ms: int):
        if self.COUNTER % 10 == 0:
            self.FPS = 10 / (time.time() - self.START_TIME)
            self.START_TIME = time.time()
        
        self.classification_result_list.append(result)
        self.COUNTER += 1

    def run(self):
        base_options = python.BaseOptions(model_asset_path=self.model)
        options = vision.ImageClassifierOptions(base_options=base_options,
                                                running_mode=vision.RunningMode.LIVE_STREAM,
                                                max_results=self.max_results,
                                                score_threshold=self.score_threshold,
                                                result_callback=self.save_result)
        classifier = vision.ImageClassifier.create_from_options(options)
    
        # Initialize a variable to track the timestamp
        timestamp = 0
    
        while self.cap.isOpened():
            success, image = self.cap.read()
            if not success:
                sys.exit('ERROR: Unable to read from webcam.')
    
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
    
            # Increment the timestamp
            timestamp += 1
    
            # Use the incremented timestamp
            classifier.classify_async(mp_image, timestamp)
    
            if self.classification_result_list:
                for category in self.classification_result_list[0].classifications[0].categories:
                    self.deal_with_result(category.category_name)
                self.classification_result_list.clear()
    
            if cv2.waitKey(1) == 27:
                break
    
        classifier.close()
        self.cap.release()
    cv2.destroyAllWindows()
    @staticmethod
    def parse_args():
        parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        parser.add_argument('--model', required=False, default='model.tflite')
        parser.add_argument('--maxResults', required=False, default=1, type=int)
        parser.add_argument('--scoreThreshold', required=False, default=0.0, type=float)
        parser.add_argument('--cameraId', required=False, default=0, type=int)
        parser.add_argument('--frameWidth', required=False, default=224, type=int)
        parser.add_argument('--frameHeight', required=False, default=224, type=int)
        return parser.parse_args()


def main():
    parser = argparse.ArgumentParser(description="Robot with LiDAR Navigation and Image Classification")
    parser.add_argument('--model', type=str, required=True, help='Path to the image classification model file')
    args = parser.parse_args()

    serial_comm = SerialComm()
    serial_comm.initSerComm()

    movement_controller = MovementController(serial_comm)  # Initialize MovementController

    image_classification_robot = ImageClassifierRobot(
        model=args.model,
        max_results=5,
        score_threshold=0.5,
        camera_id=0,
        width=640,
        height=480,
        movementcontroller = movement_controller,
    )

    lidar_navigation = LidarNavigation(
        lidar_port=LIDAR_PORT,
        obstacle_threshold=OBSTACLE_DISTANCE_THRESHOLD,
        movementcontroller=movement_controller,  # Pass movement_controller instance
        classifier=image_classification_robot
    )

    lidar_navigation.navigate()

if __name__ == "__main__":
    main()
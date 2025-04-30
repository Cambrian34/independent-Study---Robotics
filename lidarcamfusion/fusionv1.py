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

# Configuration
LIDAR_PORT = "/"
OBSTACLE_DISTANCE_THRESHOLD = 30  # mm
ROBOT_SPEED = 15

# Logging setup
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class LidarNavigation:
    def __init__(self, lidar_port, obstacle_threshold, robot_speed, classifier):
        self.lidar = PyLidar3.YdLidarG4(lidar_port)
        self.robot = SerialComm()
        self.robot.initSerComm()
        self.obstacle_threshold = obstacle_threshold
        self.robot_speed = robot_speed
        self.classifier = classifier
        self.classifier_thread = threading.Thread(target=self.classifier.run, daemon=True) #Thread for image classification
        self.classifier_thread.start()

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
                        logging.info("Obstacle detected! Triggering image classification...")
                        # self.classifier.run() # Remove blocking call
                        self.classifier.process_result() #Process the result of the image classification.

                        left_clearance = data.get(135, 3000)
                        right_clearance = data.get(45, 3000)
                        left_back_clearance = data.get(225, 3000)
                        right_back_clearance = data.get(315, 3000)

                        if left_clearance > right_clearance:
                            logging.info("Turning Left (Obstacle)")
                            # self.robot.turnLeft(15)
                        elif right_clearance > left_clearance:
                            logging.info("Turning Right (Obstacle)")
                            # self.robot.turnRight(15)
                        else:
                            if left_back_clearance > right_back_clearance:
                                logging.info("Turning Left (Back, Obstacle) ...")
                                # self.robot.turnLeft(15)
                            else:
                                logging.info("Turning Right (Back, Obstacle) ...")
                                # self.robot.turnRight(15)

                        time.sleep(0.5)

                    else:
                        logging.info("Moving Forward")
                        # self.robot.moveForward(15)

                    time.sleep(0.1)

                except Exception as e:
                    logging.error(f"Error: {e}")
                    break
        else:
            logging.error("Failed to connect to LiDAR.")

class ImageClassification:
    def __init__(self, model: str, max_results: int, score_threshold: float, camera_id: int, width: int, height: int):
        self.model = model
        self.max_results = max_results
        self.score_threshold = score_threshold
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.classification_result_list = []
        self.COUNTER, self.FPS = 0, 0
        self.START_TIME = time.time()
        self.result_lock = threading.Lock() #Lock for thread safety

        self.robot = SerialComm()
        self.robot.initSerComm()

        base_options = python.BaseOptions(model_asset_path=self.model)
        options = vision.ImageClassifierOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.LIVE_STREAM,
            max_results=self.max_results,
            score_threshold=self.score_threshold,
            result_callback=self.save_result
        )
        self.classifier = vision.ImageClassifier.create_from_options(options)

    def deal_with_result(self, result_name):
        if result_name == "red":
            print("red")
            self.robot.stop_robot()
        elif result_name == "green":
            print("green")
            self.robot.moveForward(15)
        elif result_name == "stop":
            print("stop")
            self.robot.stop_robot()
        elif result_name == "yield":
            print("yield")
            self.robot.moveForward(10)
        elif result_name == "speed55":
            print("speed 55")
            self.robot.moveForward(25)
        elif result_name == "speed35":
            print("speed 35")
            self.robot.moveForward(20)
        elif result_name == "pedestrian":
            print("pedestrian")
            self.robot.stop_robot()
        elif result_name == "right":
            print("right")
            self.robot.moveForward(15)

    def save_result(self, result: vision.ImageClassifierResult, unused_output_image: mp.Image, timestamp_ms: int):
        global FPS, COUNTER, START_TIME

        if COUNTER % 10 == 0:
            FPS = 10 / (time.time() - START_TIME)
            START_TIME = time.time()

        with self.result_lock:
            self.classification_result_list.append(result)
        COUNTER += 1

    def run(self):
        cap = cv2.VideoCapture(self.camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        while cap.isOpened():
            success, image = cap.read()
            if not success:
                sys.exit('ERROR: Unable to read from webcam. Please verify your webcam settings.')

            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

            self.classifier.classify_async(mp_image, time.time_ns() // 1_000_000)

            if cv2.waitKey(1) == 27:
                break

        self.classifier.close()
        cap.release()
        cv2.destroyAllWindows()

    def process_result(self):
        with self.result_lock:
            if self.classification_result_list:
                for category in self.classification_result_list[0].classifications[0].categories:
                    self.deal_with_result(category.category_name)
                self.classification_result_list.clear()

def main():
    parser = argparse.ArgumentParser(description="Robot with LiDAR Navigation and Image Classification")
    parser.add_argument('--model', type=str, required=True, help='Path to the image classification model file')
    args = parser.parse_args()

    image_classification_robot = ImageClassification(
        model=args.model,
        max_results=5,
        score_threshold=0.5,
        camera_id=0,
        width=640,
        height=480
    )

    lidar_navigation = LidarNavigation(
        lidar_port=LIDAR_PORT,
        obstacle_threshold=OBSTACLE_DISTANCE_THRESHOLD,
        robot_speed=ROBOT_SPEED,
        classifier=image_classification_robot
    )

    lidar_navigation.navigate()

if __name__ == "__main__":
        main()
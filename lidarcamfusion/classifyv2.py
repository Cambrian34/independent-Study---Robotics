import argparse
import sys
import time
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mySerCommLibrary import SerialComm

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

if __name__ == '__main__':
    args = ImageClassifierRobot.parse_args()
    robot_classifier = ImageClassifierRobot(args.model, args.maxResults, args.scoreThreshold, args.cameraId, args.frameWidth, args.frameHeight)
    try:
        robot_classifier.run()
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Stopping the robot.")
        robot_classifier.robot.stop_robot()
        sys.exit(0)
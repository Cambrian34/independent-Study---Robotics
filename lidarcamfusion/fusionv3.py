import argparse
import sys
import time
import PyLidar3
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision 
import threading
import logging
import queue
import serial 

# --- Config ---
DEFAULT_MODEL_PATH = 'model.tflite'
DEFAULT_SERIAL_PORT = '/dev/ttyUSB1'
DEFAULT_LIDAR_PORT = '/dev/ttyUSB0'
DEFAULT_CAMERA_ID = 0
DEFAULT_FRAME_WIDTH = 640
DEFAULT_FRAME_HEIGHT = 480
DEFAULT_MAX_RESULTS = 1
DEFAULT_SCORE_THRESHOLD = 0.9 #could be lower?
DEFAULT_OBSTACLE_THRESHOLD = 500   #mm

# --- Serial Communication ---
class SerialComm:
    #some unused methods were pruned for now
    def __init__(self, baudrate=9600, port="/dev/ttyUSB0", timeout=1):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

    def initSerComm(self):
        #starts handshaking
        print("*** Press the GREEN button to start the robot ***")
        time.sleep(2)

        while True:
            print("--- Sending out handshaking signal ---")
            ack = self.cmdSend(1)
            if not ack:
                print("*** Try again ***")
                print("*** Press the GREEN button to start the robot ***")
            else:
                print("!!! Connected to the robot !!!")
                self.ser.readall()
                break

    def cmdSend(self, cmd):
        msg = str(cmd) + "\n"
        self.ser.write(msg.encode())
        ack_origin = self.ser.readline()
        ack = ack_origin[:-2].decode("utf-8")
        return ack

    def moveForward(self, power):
        print("Move forward with power:", power)
        ack = self.cmdSend(12)
        print("Arduino is now waiting for motor power input...")
        print(ack)
        time.sleep(0.3)
        motor_power = str(power)+"\n"
        self.ser.write(motor_power.encode())

    def moveBack(self, power):
        print("Move backward with power:", power)
        ack = self.cmdSend(12) 
        print("Arduino is now waiting for motor power input...")
        print(ack)
        time.sleep(0.3)
        motor_power = str(-power)+"\n" 
        self.ser.write(motor_power.encode())

    def turnLeft(self, power):
        print("Turn left with power:", power)
        ack = self.cmdSend(13)
        print("Arduino is now waiting for motor power input...")
        print(ack)
        time.sleep(0.3)
        motor_power = str(power)+"\n"
        self.ser.write(motor_power.encode())

    def turnRight(self, power):
        print("Turn right with power:", power)
        ack = self.cmdSend(14)
        print("Arduino is now waiting for motor power input...")
        print(ack)
        time.sleep(0.3)
        motor_power = str(power)+"\n"
        self.ser.write(motor_power.encode())

    def stop_robot(self):
        print("Stop")
        ack = self.cmdSend(5)
        print(ack)

# --- Logging Setup ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(threadName)s - %(message)s')

# --- MovementController ---
class MovementController:
    """
    A class to manage robot movement commands from multiple sources.
    It uses a queue to ensure commands are executed sequentially and avoid conflicts.
    """
    def __init__(self, serial_comm_instance: SerialComm):
       
        self.serial_comm = serial_comm_instance
        self.command_queue = queue.Queue()
        self._stop_event = threading.Event() # Event to signal the processing thread to stop
        self.thread = threading.Thread(target=self._process_commands, daemon=True, name="MovementThread")
        self.current_command = None
        self.is_running = False # Flag to indicate if processing has started
        logging.info("MovementController initialized.")

    def start(self):
        if not self.is_running:
            self.thread.start()
            self.is_running = True
            logging.info("MovementController thread started.")

    def add_command(self, command: str, priority: int = 0):
        logging.debug(f"Adding command: {command} with priority {priority}")
        self.command_queue.put((priority, command))

    def _process_commands(self):
        logging.info("Command processing loop starting.")
        while not self._stop_event.is_set():
            try:
                # Get command with a timeout to allow checking the stop_event
                priority, command = self.command_queue.get(timeout=0.1)
                self.current_command = command
                logging.info(f"Processing command: {command}")

                # --- Execute Command ---
                if command == "forward":
                    self.serial_comm.moveForward(15) 
                elif command == "forward_slow":
                    self.serial_comm.moveForward(10)
                elif command == "forward_medium":
                    self.serial_comm.moveForward(20)
                elif command == "forward_fast":
                    self.serial_comm.moveForward(25)
                elif command == "left":
                    self.serial_comm.turnLeft(15)
                    time.sleep(0.7)
                    self.serial_comm.stop_robot() 
                elif command == "right":
                    self.serial_comm.turnRight(15)
                    time.sleep(0.7) 
                    self.serial_comm.stop_robot()
                elif command == "backward":
                    self.serial_comm.moveBack(12)
                elif command == "stop":
                    self.serial_comm.stop_robot()
                else:
                    logging.warning(f"Movement Controller: Unknown command: {command}")

                self.command_queue.task_done()
                self.current_command = None

            except queue.Empty:
                continue
            except Exception as e:
                logging.error(f"Error processing command: {e}", exc_info=True)
                self.current_command = None # Reset current command on error

        logging.info("Command processing loop finished.")
        # Ensure robot is stopped when thread exits
        try:
            self.serial_comm.stop_robot()
            logging.info("Final stop command sent by MovementController.")
        except Exception as e:
            logging.error(f"Error sending final stop command: {e}")


    def stop(self):
        #Signals the command processing thread to stop
        logging.info("MovementController stop requested.")
        self._stop_event.set()
        
    def wait_for_completion(self):
        #Waits for the command queue to be empty
        self.command_queue.join()

    def get_current_command(self):
        #Returns the command currently being processed
        return self.current_command

# --- LidarNavigation ---
class LidarNavigation:
    def __init__(self, lidar_port: str, obstacle_threshold: int, movement_controller: MovementController):
        self.lidar_port = lidar_port
        self.obstacle_threshold = obstacle_threshold
        self.movement_controller = movement_controller
        self.lidar = None
        self._stop_event = threading.Event()
        self._lidar_connected = False

    def connect_lidar(self):
       #Attempts to connect to the LiDAR
        try:
            logging.info(f"Attempting to connect to LiDAR on port {self.lidar_port}")
            self.lidar = PyLidar3.YdLidarG4(self.lidar_port)
            if self.lidar.Connect():
                logging.info("LiDAR connected successfully.")
                logging.info(self.lidar.GetDeviceInfo())
                self._lidar_connected = True
                return True
            else:
                logging.error(f"Failed to connect to LiDAR on {self.lidar_port}")
                self._lidar_connected = False
                return False
        except Exception as e:
            logging.error(f"Exception connecting to LiDAR: {e}", exc_info=True)
            self._lidar_connected = False
            return False

    def navigate(self):
        """Main navigation loop using LiDAR data."""
        if not self._lidar_connected:
            logging.error("Cannot start navigation: LiDAR not connected.")
            return

        try:
            gen = self.lidar.StartScanning()
            logging.info("LiDAR scanning started.")

            while not self._stop_event.is_set():
                try:
                    data = next(gen)
                    # Get distances (provide default far distance if angle not found)
                    front_dist = data.get(0,10000)
                    left_front_dist = data.get(345,10000)
                    right_front_dist = data.get(15,10000) 

                    logging.debug(f"LiDAR Front: {front_dist:.0f} mm, "
                                  f"L-Front: {left_front_dist:.0f} mm, "
                                  f"R-Front: {right_front_dist:.0f} mm")

                    if (front_dist < self.obstacle_threshold or
                        left_front_dist < self.obstacle_threshold  or right_front_dist < self.obstacle_threshold ):

                        current_cmd = self.movement_controller.get_current_command()
                        if current_cmd not in ["backward", "left", "right", "stop"]: # Avoid interrupting avoidance maneuver
                            logging.warning(f"Obstacle detected! Front={front_dist:.0f}mm. Threshold={self.obstacle_threshold}mm. Overriding current command.")
                            # 1. Stop immediately (high priority)
                            self.movement_controller.add_command("stop", priority=-1)
                            time.sleep(0.2) 

                            # 2. Backup 
                            self.movement_controller.add_command("backward", priority=0)
                            time.sleep(1.5) 
                            self.movement_controller.add_command("stop", priority=0)
                            time.sleep(0.3) 

                            # 3. request new LiDAR data after backing up
                            try:
                                data = next(gen) # Fetch fresh data
                            except StopIteration:
                                logging.warning("LiDAR stopped unexpectedly during avoidance.")
                                break

                            # 4. Decide turn direction based on side clearance after backing up
                            # Wider angle checks for turning space
                            left_clearance = data.get(270,10000)
                            right_clearance = data.get(90, 10000) 
                            logging.info(f"Clearance check after backup: Left={left_clearance:.0f}mm, Right={right_clearance:.0f}mm")

                            if left_clearance > right_clearance and left_clearance > self.obstacle_threshold * 1.5:
                                logging.info("Turning Left (Obstacle Avoidance)")
                                self.movement_controller.add_command("left", priority=0)
                            elif right_clearance > left_clearance and right_clearance > self.obstacle_threshold * 1.5:
                                logging.info("Turning Right (Obstacle Avoidance)")
                                self.movement_controller.add_command("right", priority=0)
                            else:
                                logging.warning("Both sides are blocked after backup, stopping.")
                                self.movement_controller.add_command("stop", priority=0) 

                            time.sleep(1.0) 

                    else:
                        if self.movement_controller.get_current_command() is None:
                           logging.debug("Path clear, adding forward command.")
                           if self.movement_controller.get_current_command() != "forward":
                               self.movement_controller.add_command("forward", priority=2) 
                    time.sleep(0.1) 

                except StopIteration:
                    logging.warning("LiDAR scanner stopped.")
                    break 
                except Exception as e:
                    logging.error(f"Error in LiDAR navigation loop: {e}", exc_info=True)
                    time.sleep(1) 

        except Exception as e:
            logging.error(f"Error starting LiDAR scanning: {e}", exc_info=True)
        finally:
            self.stop() 

    def stop(self):
        """Stops the navigation loop and disconnects the LiDAR."""
        logging.info("Stopping LiDAR navigation...")
        self._stop_event.set()
        if self.lidar and self._lidar_connected:
            try:
                self.lidar.StopScanning()
                logging.info("LiDAR scanning stopped.")
                self.lidar.Disconnect()
                logging.info("LiDAR disconnected.")
                self._lidar_connected = False
            except Exception as e:
                logging.error(f"Error stopping/disconnecting LiDAR: {e}")
        self.lidar = None

# --- ImageClassifierRobot ---
class ImageClassifierRobot:
    def __init__(self, model_path: str, max_results: int, score_threshold: float,
                 camera_id: int, width: int, height: int, movement_controller: MovementController):
        self.model_path = model_path
        self.max_results = max_results
        self.score_threshold = score_threshold
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.movement_controller = movement_controller # Use the shared controller

        self._stop_event = threading.Event()
        self.classification_result_list = []
        self._result_lock = threading.Lock() # To safely access results list from callback

        # MediaPipe setup placeholders
        self.classifier = None
        self.cap = None

        # FPS calculation helpers
        self._frame_counter = 0
        self._fps = -1
        self._start_time = time.time()

    def _setup_mediapipe(self):
        """Initializes MediaPipe classifier."""
        try:
            base_options = python.BaseOptions(model_asset_path=self.model_path)
            options = vision.ImageClassifierOptions(
                base_options=base_options,
                running_mode=vision.RunningMode.LIVE_STREAM,
                max_results=self.max_results,
                score_threshold=self.score_threshold,
                result_callback=self._save_result # Use the internal callback
            )
            self.classifier = vision.ImageClassifier.create_from_options(options)
            logging.info("MediaPipe Image Classifier created.")
            return True
        except Exception as e:
            logging.error(f"Failed to create MediaPipe classifier: {e}", exc_info=True)
            return False

    def _setup_camera(self):
        """Initializes the camera capture."""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                logging.error(f"Failed to open camera with ID: {self.camera_id}")
                return False
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            logging.info(f"Camera {self.camera_id} opened ({self.width}x{self.height}).")
            return True
        except Exception as e:
            logging.error(f"Exception setting up camera: {e}", exc_info=True)
            return False

    def _save_result(self, result: vision.ImageClassifierResult, output_image: mp.Image, timestamp_ms: int):
        """Callback function to receive classification results."""
        # Calculate FPS
        self._frame_counter += 1
        if self._frame_counter % 10 == 0: # Calculate every 10 frames
            end_time = time.time()
            elapsed = end_time - self._start_time
            self._fps = 10 / elapsed if elapsed > 0 else 0
            self._start_time = end_time 

        # Store the latest result safely
        with self._result_lock:
            self.classification_result_list.clear() # Keep only the latest result
            self.classification_result_list.append(result)
        logging.debug(f"Result received. FPS: {self._fps:.1f}")

    def _process_latest_result(self):
        """Processes the latest classification result and issues commands."""
        latest_result = None
        with self._result_lock:
            if self.classification_result_list:
                latest_result = self.classification_result_list[0] # Get the only stored result

        if latest_result and latest_result.classifications:
             # Assuming the first classification list is the most relevant
            classifications = latest_result.classifications[0]
            if classifications.categories:
                # Get the top category (highest score)
                top_category = classifications.categories[0]
                result_name = top_category.category_name.lower() # Use lower case for matching
                score = top_category.score

                logging.info(f"Detected: {result_name} (Score: {score:.2f})")

                # --- Map detected objects to MovementController commands ---
                # Use priority to let obstacle avoidance override signs if necessary
                command = None
                priority = 2 # Default priority for signs

                if result_name == "red" or result_name == "stop" or result_name == "pedestrian":
                    command = "stop"
                    priority = 1 # Higher priority than default forward
                elif result_name == "green":
                    command = "forward_medium"
                elif result_name == "yield":
                    command = "forward_slow"
                elif result_name == "speed55": # Assuming this means faster
                    command = "forward_fast"
                elif result_name == "speed35": # Assuming this means medium
                     command = "forward_medium"
                elif result_name == "right":
                    logging.info("Right turn sign detected - action not implemented yet.")
                    pass 

                if command:
                    # Only add command if the robot isn't already avoiding an obstacle
                    current_mc_cmd = self.movement_controller.get_current_command()
                    if current_mc_cmd not in ["backward", "left", "right"]: # Don't override avoidance
                        logging.debug(f"Camera issuing command: {command}")
                        self.movement_controller.add_command(command, priority=priority)
                    else:
                        logging.debug(f"Camera detected {result_name}, but obstacle avoidance ({current_mc_cmd}) is active.")
            else:
                 logging.debug("No categories found in classification.")
        else:
            logging.debug("No classification result to process.")


    def run(self):
        """Main loop for capturing frames and running classification."""
        if not self._setup_mediapipe() or not self._setup_camera():
            logging.error("Setup failed. Exiting ImageClassifierRobot run.")
            self.stop() 
            return

        timestamp_ms = 0
        while not self._stop_event.is_set():
            if not self.cap.isOpened():
                logging.error("Camera is not open. Stopping classification.")
                break

            success, frame = self.cap.read()
            if not success:
                logging.warning("Failed to grab frame from camera.")
                time.sleep(0.5) # Wait a bit before retrying
                continue

            # Convert the frame received from OpenCV to a MediaPipe’s Image object.
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

            # Get current time for timestamp
            timestamp_ms = int(time.time() * 1000)

            # Run classification asynchronously
            try:
                self.classifier.classify_async(mp_image, timestamp_ms)
            except Exception as e:
                logging.error(f"Error during classify_async: {e}", exc_info=True)
                time.sleep(1) # Avoid rapid error loops

            # Process the latest result received by the callback
            self._process_latest_result()

            # Display the frame? can be slow
            # cv2.putText(frame, f"FPS: {self._fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            # cv2.imshow('Robot View', frame)
            # if cv2.waitKey(1) == 27: # ESC key
            #     self._stop_event.set() # Signal stop if ESC is pressed in window

            # Small delay to prevent hogging CPU if capture/processing is very fast
            time.sleep(0.01)

        logging.info("Image classification loop finished.")
        self.stop() # Clean up resources

    def stop(self):
        """Stops the classification loop and releases resources."""
        logging.info("Stopping image classification...")
        self._stop_event.set()

        # Close MediaPipe classifier
        if self.classifier:
            try:
                self.classifier.close()
                logging.info("MediaPipe classifier closed.")
            except Exception as e:
                logging.error(f"Error closing classifier: {e}")
            self.classifier = None

        # Release camera
        if self.cap and self.cap.isOpened():
            self.cap.release()
            logging.info("Camera released.")
            self.cap = None

        # Close OpenCV windows if any were opened
        # cv2.destroyAllWindows()


# --- Main Execution ---
def main():
    parser = argparse.ArgumentParser(
    description="Run Robot with LiDAR Navigation and Image Classification.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    
    parser.add_argument('--model', type=str,
                        default=DEFAULT_MODEL_PATH,
                        help='Path to the TFLite image classification model file.')
    parser.add_argument('--serial-port', type=str,
                        default=DEFAULT_SERIAL_PORT,
                        help='Serial port for robot motor controller communication.')
    parser.add_argument('--lidar-port', type=str,
                        default=DEFAULT_LIDAR_PORT,
                        help='Serial port for the LiDAR sensor.')
    parser.add_argument('--camera-id', type=int,
                        default=DEFAULT_CAMERA_ID,
                        help='ID of the camera device.')
    parser.add_argument('--frame-width', type=int,
                        default=DEFAULT_FRAME_WIDTH,
                        help='Width of the camera frame for processing.')
    parser.add_argument('--frame-height', type=int,
                        default=DEFAULT_FRAME_HEIGHT,
                        help='Height of the camera frame for processing.')
    parser.add_argument('--max-results', type=int,
                        default=DEFAULT_MAX_RESULTS,
                        help='Maximum classification results from the model.')
    parser.add_argument('--score-threshold', type=float,
                        default=DEFAULT_SCORE_THRESHOLD,
                        help='Score threshold for displaying classification results.')
    parser.add_argument('--obstacle-threshold', type=int,
                        default=DEFAULT_OBSTACLE_THRESHOLD,
                        help='Obstacle detection distance threshold in mm for LiDAR.')


    args = parser.parse_args()

    # --- Initialization ---
    threads = []
    lidar_nav = None
    image_classifier = None
    movement_controller = None
    serial_comm = None

    try:
        # 1. Initialize Serial Communication 
        logging.info(f"Initializing serial communication on {args.serial_port}...")
        serial_comm = SerialComm(port=args.serial_port) # Use the provided class
        serial_comm.initSerComm() # Call initSerComm to establish connection
        logging.info("Serial communication initialized successfully.")

        # 2. Initialize Movement Controller 
        logging.info("Initializing Movement Controller...")
        movement_controller = MovementController(serial_comm)
        movement_controller.start() # Start the command processing thread
        logging.info("Movement Controller initialized and started.")

        # 3. Initialize Image Classifier 
        logging.info("Initializing Image Classifier Robot...")
        image_classifier = ImageClassifierRobot(
            model_path=args.model,
            max_results=args.max_results,
            score_threshold=args.score_threshold,
            camera_id=args.camera_id,
            width=args.frame_width,
            height=args.frame_height,
            movement_controller=movement_controller,
        )
        logging.info("Image Classifier Robot initialized.")

        # 4. Initialize LiDAR Navigation 
        logging.info("Initializing LiDAR Navigation...")
        lidar_nav = LidarNavigation(
            lidar_port=args.lidar_port,
            obstacle_threshold=args.obstacle_threshold,
            movement_controller=movement_controller,
        )
        # Attempt to connect LiDAR before starting thread
        if not lidar_nav.connect_lidar():
            logging.warning("Cannot connect to LiDAR.")
            #stop program if lidar cannot be connected to
            sys.exit(1) 
        logging.info("LiDAR Navigation initialized.")

        # --- Start Threads ---
        logging.info("Starting worker threads...")

        # Camera Thread
        if image_classifier:
            cam_thread = threading.Thread(target=image_classifier.run, name="CameraThread", daemon=True)
            cam_thread.start()
            threads.append(cam_thread)
            logging.info("Camera thread started.")

        # LiDAR Thread
        if lidar_nav and lidar_nav._lidar_connected: # Only start if connected
            lidar_thread = threading.Thread(target=lidar_nav.navigate, name="LidarThread", daemon=True)
            lidar_thread.start()
            threads.append(lidar_thread)
            logging.info("LiDAR thread started.")

        # Keep main thread alive, wait for KeyboardInterrupt
        while True:
            if lidar_nav and lidar_nav._lidar_connected and not lidar_thread.is_alive():
                logging.warning("LiDAR thread seems to have stopped unexpectedly.")
            if image_classifier and not cam_thread.is_alive():
                 logging.warning("Camera thread seems to have stopped unexpectedly.")
            time.sleep(1) # Keep main thread alive

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Initiating shutdown...") 
        logging.info("KeyboardInterrupt received. Stopping all components.")

    except Exception as e:
        logging.critical(f"An unhandled exception occurred in main: {e}", exc_info=True)

    finally:
        logging.info("--- Shutdown Sequence ---")

        # Signal threads to stop first
        if image_classifier:
            logging.info("Requesting Image Classifier stop...")
            image_classifier.stop()
        if lidar_nav:
            logging.info("Requesting LiDAR Navigation stop...")
            lidar_nav.stop()

        # Stop the movement controller's processing loop
        if movement_controller:
            logging.info("Requesting Movement Controller stop...")
            # Send a final stop command with high priority before stopping the controller loop
            movement_controller.add_command("stop", priority=-2)
            time.sleep(0.5) # Give a moment for the stop command to potentially be processed
            movement_controller.stop()


        # Final stop robot contingency
        if serial_comm:
             try:
                 # Ensure robot is stopped 
                 serial_comm.stop_robot() # Use the correct stop method
                 logging.info("Serial communication actions completed for shutdown.")
             except Exception as e:
                 logging.error(f"Error during serial communication shutdown: {e}")


        logging.info("Shutdown complete.")
        print("Robot shutdown finished.")


if __name__ == "__main__":
    main()
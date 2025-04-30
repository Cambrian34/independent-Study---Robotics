import queue
import threading
import logging
import time 

from mySerCommLibrary import SerialComm

class MovementController:
    """
    A class to manage robot movement commands from multiple sources.
    It uses a queue to ensure commands are executed sequentially and avoid conflicts.
    """

    def __init__(self, serial_comm):
       
        self.serial_comm = serial_comm
        self.command_queue = queue.Queue()
        self.thread = threading.Thread(target=self._process_commands, daemon=True)
        self.thread.start()
        self.current_command = None 

    def add_command(self, command, priority=0):
        """
        Adds a movement command to the queue.
        """
        self.command_queue.put((priority, command))

    def _process_commands(self):
        """
        Processes commands from the queue in a separate thread.
        """
        while True:
            priority, command = self.command_queue.get()
            self.current_command = command

            if command == "forward":
                self.serial_comm.moveForward(12)
                logging.info("Movement Controller: Moving Forward")
            elif command == "left":
                self.serial_comm.turnLeft(15)
                time.sleep(0.7)
                logging.info("Movement Controller: Turning Left")
            elif command == "right":
                self.serial_comm.turnRight(15)
                time.sleep(0.7)
            elif command == "stop":
                self.serial_comm.stop_robot()
                logging.info("Movement Controller: Stopping")
            elif command == "forward_slow":
                self.serial_comm.moveForward(10)
                logging.info("Movement Controller: Moving Forward Slowly")
            elif command == "forward_fast":
                self.serial_comm.moveForward(25)
                logging.info("Movement Controller: Moving Forward Fast")
            elif command == "backward":
                self.serial_comm. moveBack(12)
                logging.info("Movement Controller: Moving backward")

            else:
                logging.warning(f"Movement Controller: Unknown command: {command}")

            self.command_queue.task_done()
            self.current_command = None

    def get_current_command(self):
       
        return self.current_command
import time
import serial
import threading

class SerialComm:
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
        print("Move forward")

        ack = self.cmdSend(12)

        # check the output we get from the controller
        print("Arduino is now waiting for motor power input...")
        print(ack)

        #timer for 1 seconds
        time.sleep(0.3)
        # Send the motor power as plain data, not as a command
        motor_power = str(power)+"\n"  # Send motor power value

        self.ser.write(motor_power.encode())  # Send motor power directly to Arduino

        # Wait for Arduino response
        ack = self.ser.readline().decode("utf-8").strip()
        #print(f"Motor power set: {ack}")
        print(ack)

    def moveBack(self, power):
        print("Move backward")
        ack = self.cmdSend(12)

        # check the output we get from the controller
        print("Arduino is now waiting for motor power input...")
        print(ack)

        #timer for 1 seconds
        time.sleep(0.3)
        # Send the motor power as plain data, not as a command
        motor_power = str(-power)+"\n"  # Send motor power value

        self.ser.write(motor_power.encode())  # Send motor power directly to Arduino

        # Wait for Arduino response
        ack = self.ser.readline().decode("utf-8").strip()
        #print(f"Motor power set: {ack}")
        print(ack)

    def turnLeft(self, power):
        print("Turn left")
        ack = self.cmdSend(17)
        # check the output we get from the controller
        print("Arduino is now waiting for motor power input...")
        print(ack)

        #timer for 1 seconds
        time.sleep(1)
        # Send the motor power as plain data, not as a command
        motor_power = str(power)+"\n"  # Send motor power value

        self.ser.write(motor_power.encode())  # Send motor power directly to Arduino

        # Wait for Arduino response
        ack = self.ser.readline().decode("utf-8").strip()
        #print(f"Motor power set: {ack}")
        print(ack)

    def turnRight(self, power):
        print("Turn right")
        ack = self.cmdSend(16)
        # check the output we get from the controller
        print("Arduino is now waiting for motor power input...")
        print(ack)

        #timer for 1 seconds
        time.sleep(0.3)
        # Send the motor power as plain data, not as a command
        motor_power = str(power)+"\n"  # Send motor power value

        self.ser.write(motor_power.encode())  # Send motor power directly to Arduino

        # Wait for Arduino response
        ack = self.ser.readline().decode("utf-8").strip()
        #print(f"Motor power set: {ack}")

        print(ack)

    def stop_robot(self):
        print("Stop")
        ack = self.cmdSend(5)
        print(ack)

    def turn_around(self):
        print("Turn around")
        ack = self.cmdSend(13)
        print(ack)

    def readSonicIN(self, port):
        print("Read Sonic INCHES")
        ack = self.cmdSend(11)
        # check the output we get from the controller
        print("Arduino is now waiting for port input...")
        print(ack)

        #timer for 0.5 seconds
        time.sleep(0.5)
        # Send the motor power as plain data, not as a command
        motor_power = str(port)+"\n"  # Send motor power value

        self.ser.write(motor_power.encode())  # Send motor power directly to Arduino

        # Wait for Arduino response
        ack = self.ser.readline().decode("utf-8").strip()
        #print(f"Motor power set: {ack}")
        print(ack)
        return ack

    def readSonicCM(self, port):
        # print("Read Sonic CM")
        ack = self.cmdSend(4)
        # check the output we get from the controller
        # print("Arduino is now waiting for port input...")
        # print(ack)

        #timer for 0.5 seconds
        time.sleep(0.5)
        # Send the port as plain data, not as a command
        ser_port = str(port)+"\n"  # Send port value

        self.ser.write(ser_port.encode())  # Send port directly to Arduino

        # Wait for Arduino response
        ack = self.ser.readline().decode("utf-8").strip()
        #print(f"Motor power set: {ack}")

        print("Sonic Distance (CM):", int(ack))
        return int(ack)

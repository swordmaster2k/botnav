import sys
import time
import serial
import socket
import threading

from threading import Thread
from threading import Timer

from proxy import Proxy
from model.robot import Robot
from events import OdometryReport, ScanResult
from bluetooth_connection import BluetoothConnection

class Main(threading.Thread):
    def __init__(self):
        # Bluetooth.
        connection = BluetoothConnection("00:00:12:06:56:83", 0x1001)

        # USB Serial.
        #connection = serial.Serial("/dev/ttyACM0", 115200)

        self.proxy = Proxy(self, connection)
        self.robot = Robot(self.proxy)

        Thread.__init__(self)

    def run(self):
		
        while True:
			
            command = input()
            
            if command[0] == "w":
                    self.robot.go_forward()
            elif command[0] == "s":
                    self.robot.go_backward()
            elif command[0] == "a":
                    self.robot.rotate_left()
            elif command[0] == "d":
                    self.robot.rotate_right()
            elif command[0] == "q":
                    self.robot.halt()
            elif command[0] == "e":
                    if command == "end":
                        break;
                
                    self.robot.scan()
            elif command[0] == "z":
                    self.robot.reset()
            elif command[0] == "r":
                    if len(command) < 5:
                        print("Invalid heading format rX.XX: " + command)
                        continue

                    self.robot.rotate_to(float(command[1:5]))
            else:
                    print("Unknown command: " + command)
	
    def pop_event(self):
        # Attempt to get the lock and then process 
        # any events.
        with self.proxy.mutex:
            if len(self.proxy.events) < 1:
                return

            event = self.proxy.events.pop()
            print(event.to_string())

            if isinstance(event, OdometryReport):
                didChange = self.robot.update_odometry(event)
            elif isinstance(event, ScanResult):
                True # Do something...
	
main = Main()
main.start()
main.proxy.start()

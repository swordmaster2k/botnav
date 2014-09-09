import sys
import time
import serial
import socket
import threading

from threading import Thread
from threading import Timer

from proxy import Proxy
from model.map import Map
from model.robot import Robot
from events import OdometryReport, ScanResult, StateEvent
from bluetooth_connection import BluetoothConnection

class Main(threading.Thread):
    def __init__(self):
        # Bluetooth.
        connection = BluetoothConnection("00:00:12:06:56:83", 0x1001)

        # USB Serial.
        #connection = serial.Serial("/dev/ttyACM1", 115200)

        self.proxy = Proxy(self, connection)
        self.robot = Robot(self.proxy)
        self.robot.x = 1.15
        self.robot.y = 1.15

        self.map = Map(self.robot, 2.3, 2.3, 0.23)

        Thread.__init__(self)

    def run(self):
		
        while True:
			
            command = input()

            if len(command) < 1:
                continue
            
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
            elif command[0] == "p":
                self.robot.ping()
            elif command[0] == "z":
                self.robot.reset()
            elif command[0] == "r":
                if len(command) < 5:
                    print("Invalid heading format rX.XX: " + command)
                    continue

                self.robot.rotate_to(float(command[1:5]))
            elif command[0] == "f":
                result = command.split(",", 3)

                if len(result) == 3:
                    self.robot.face(float(result[1]), float(result[2]))
            elif command[0] == "v":
                global verbose
                verbose = not verbose
            elif command[0] == "h":
                print("x: " + str(self.robot.x) + " y: " + str(self.robot.y) + " heading: " + str(self.robot.heading))
            elif command[0] == "t":
                result = command.split(",", 3)

                if len(result) == 3:
                    self.robot.go_to(float(result[1]), float(result[2]))
            elif command[0] == "g":
                self.map.print_map()
            else:
                print("Unknown command: " + command)
	
    def pop_event(self):
        # Attempt to get the lock and then process 
        # any events.
        with self.proxy.mutex:
            if len(self.proxy.events) < 1:
                return

            event = self.proxy.events.pop()
            global verbose

            if verbose:
                print(event.to_string())

            if isinstance(event, OdometryReport):
                didChange = self.robot.update_odometry(event)
            elif isinstance(event, ScanResult):
                cells = self.map.ping_to_cells(float(event.readings[0])) # Just take 1 reading for now.
                self.map.update_map(cells)
            elif isinstance(event, StateEvent):
                self.robot.state = event.state

verbose = False
main = Main()
main.start()
main.proxy.start()

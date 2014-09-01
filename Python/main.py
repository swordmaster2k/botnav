import sys
import time
import serial
import socket
import threading

from threading import Thread

from proxy import Proxy
from model.robot import Robot
from bluetooth_connection import BluetoothConnection

class Main(threading.Thread):
    def __init__(self):
        # Bluetooth.
        connection = BluetoothConnection("00:00:12:06:56:83", 0x1001)

        # USB Serial.
        #connection = serial.Serial("/dev/ttyACM0", 115200)

        self.proxy = Proxy(connection)
        self.robot = Robot(self.proxy)

        Thread.__init__(self)

    def run(self):
        while True:
			
            command = input()
            
            if command == "w":
                    self.robot.go_forward()
            elif command == "s":
                    self.robot.go_backward()
            elif command == "a":
                    self.robot.rotate_left()
            elif command == "d":
                    self.robot.rotate_right()
            elif command == "q":
                    self.robot.halt()
            elif command == "e":
                    self.robot.scan()
            elif command == "z":
                    self.robot.reset()
            elif command == "end":
                    break
            else:
                    print("Unknown command: " + command)
            

            # Attempt to get the lock and then process 
            # any events.
            with self.proxy.mutex:
                if len(self.proxy.events) < 1:
                    continue

                event = self.proxy.events.pop()
                print(event.to_string())

main = Main()
main.start()
main.proxy.start()

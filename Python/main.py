import sys
import time
import serial
import threading

from threading import Thread
from proxy import Proxy

class Main(threading.Thread):
    def __init__(self):
        port = serial.Serial("/dev/ttyACM0", 115200)
        self.proxy = Proxy(port)
        
        Thread.__init__(self)

    def run(self):
        command = raw_input("Enter command:")

        while command != "end":
            command = raw_input()

            with self.proxy.mutex:
                if len(self.proxy.events) < 1:
                    continue
                
                event = self.proxy.events.pop()
                print(event.to_string())

main = Main()
main.start()
main.proxy.start()
    

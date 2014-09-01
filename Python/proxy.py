import threading

from threading import Thread
from events import OdometryReport
from events import ScanResult

'''
Handles all communication to and from a robot on a dedicated thread.

It also pools all of the information coming from the robot into an event
queue.

Compatible with any from of connection as long as they implement write()
and readline().
'''
class Proxy(Thread):
    def __init__(self, listener, port):
        self.mutex = threading.Lock()
        self.listener = listener
        self.connection = port
        self.events = []
        
        Thread.__init__(self)
        self.daemon = True

    def send(self, message):
        self.connection.write(message)

    def run(self):
        while not self.connection.closed:
            data = self.connection.readline()

            if data[0] == 'o':
                parameters = data.split(',')

                if len(parameters) != 4:
                    continue
                
                report = OdometryReport(round(float(parameters[1]), 2),
                                        round(float(parameters[2]), 2),
                                        round(float(parameters[3]), 2))

                with self.mutex:
                    self.events.append(report)

                self.listener.pop_event()
            elif data[0] == 's':
                parameters = data.split(',')

                if len(parameters) > 2:
                    continue

                result = ScanResult(parameters)

                with self.mutex:
                    self.events.append(result)

                self.listener.pop_event()
            else:
                print(data)

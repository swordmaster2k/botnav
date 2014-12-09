import threading

from threading import Thread
from events import OdometryReport, ScanResult, StateEvent

'''
Handles all communication to and from a robot on a dedicated thread.

It also pools all of the information coming from the robot into an event
queue.

Compatible with any from of connection as long as they implement write()
and readline().
'''


class Proxy(Thread):
    '''

    '''

    def __init__(self, connection):
        self.listeners = []
        self.connection = connection

        Thread.__init__(self)
        self.daemon = True

    '''
    Sends a message to the robot by writing it to the output file of
    the connection.
    '''

    def send(self, message):
        try:
            self.connection.write(message)
        except:
            return

    '''
    Reads lines from the connection parsing any useful information.
    '''

    def run(self):
        if self.connection == "DUMMY":
            return

        while not self.connection.closed:
            try:
                data = self.connection.readline()

                if data[0] == 'o':
                    parameters = data.split(',')

                    if len(parameters) != 4:
                        continue

                    event = OdometryReport(
                        float(parameters[1]),
                        float(parameters[2]),
                        float(parameters[3]))

                    for listener in self.listeners:
                        listener.handle_event(event)
                elif data[0] == 's':
                    parameters = data.split(',')

                    if len(parameters) < 2:
                        continue

                    parameters.remove('s')

                    event = ScanResult(parameters)

                    for listener in self.listeners:
                        listener.handle_event(event)

                elif (data == "Going Forward\n" or
                              data == "Going Backward\n" or
                              data == "Turning Right\n" or
                              data == "Turning Left\n" or
                              data == "Halted\n" or
                              data == "Scanning\n" or
                              data == "Travelled\n"):
                    event = StateEvent(data.strip('\n'))

                    for listener in self.listeners:
                        listener.handle_event(event)
                else:
                    print(data)
            except Exception as ex:
                print(ex)
                continue

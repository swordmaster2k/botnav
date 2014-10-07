import threading

from threading import Thread

'''
Handles all communication to and from a robot on a dedicated thread.

Simply passes on the data recieved to any listener objects.

Compatible with any from of connection as long as they implement write()
and readline().
'''
class Proxy(Thread):
	'''

	'''
	def __init__(self, socket):
		self.listeners = []
		self.socket = socket

		Thread.__init__(self)

	'''
	Sends a message to the robot by writing it to the output file of 
	the connection.
	'''
	def send(self, message):
		self.socket.write(message)

	'''
	Reads lines from the connection parsing any useful information.
	'''
	def run(self):
		while not self.socket.closed:
			try:
				event = self.socket.readline()
						
				for listener in self.listeners:
					listener.handle_event(event)
			except:
				continue

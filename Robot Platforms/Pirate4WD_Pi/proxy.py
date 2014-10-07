import socket
import threading

from threading import Thread

'''
Handles all communication to and from a robot on a dedicated thread.

Polls the data collected into a command queue.

Compatible with any from of connection as long as they implement write()
and readline().
'''
class Proxy(Thread):
	'''

	'''
	def __init__(self, socket):
		self.socket = socket
		
		self.command_queue = []
		self.command_mutex = threading.Lock()

		Thread.__init__(self)

	'''
	Sends a message to the robot by writing it to the output file of 
	the connection.
	'''
	def send(self, message):
		if not message.endswith('\n'): # Messages are terminated by '\n'.
			message += '\n'
			
		self.socket.write(message)

	'''
	Reads lines from the connection parsing any useful information.
	'''
	def run(self):
		while not self.socket.closed:
			try:
				command = self.socket.readline()
				
				if len(command) > 0:		
					with self.command_mutex:
						self.command_queue.append(command)
			except IOError as err:
				print(str(err))
				continue
			except ValueError as err:
				print(str(err))
				continue

import os
import struct
import select

from threading import Thread

'''

'''
class MouseEvent:
	def __init__(self, left, middle, right, x, y):
		self.left_button = left
		self.middle_button = middle
		self.right_button = right
		self.x = x
		self.y = y

'''

'''
class Mice(Thread):
	'''
	
	'''
	def __init__(self, pirate, exit_mutex):
		self.stream = self.open_mice()
		self.pirate = pirate
		self.exit_mutex = exit_mutex
		
		Thread.__init__(self)

	'''
	Opens the mice input file on linux at /dev/input/mice in read binary 
	mode and returns the file object. 
	
	Returns False if the operation fails.
	'''
	def open_mice(self):
		try:
			# Open mice input file in binary mode.
			return open("/dev/input/mice", "rb")
		except:
			return False

	'''
	Closes the mice input file if it exists and is open.
	'''
	def close_mice(self):
		if stream == False:
			return

		if not self.stream.closed:
			self.stream.close()

	'''
	Gets the latest event from mice.
	'''
	def get_mice_event(self):
		try:
			read, write, expectional = select.select([self.stream], [], [], 0)
		
			if self.stream in read:
				buf = self.stream.read(3);
				button = buf[0];
				left_button = button & 0x1;
				middle_button = (button & 0x4) > 0;
				right_button = (button & 0x2) > 0;
				x, y = struct.unpack("bb", buf[1:]);
					
				return MouseEvent(left_button, middle_button, 
					right_button, x, y)
			else:
				return -1
		except:
			print ("Issue with reading data from mice.")
			return -1
	
	'''
	Run this on its own thread because the call to get_mice_event() will
	block if there is no data to read.
	'''		
	def run(self):
		if not self.stream == False:
			while not self.stream.closed:
				mouse_event = self.get_mice_event()
				if mouse_event != -1:
					self.pirate.update_odometry(mouse_event)
				
		self.exit_mutex.acquire()

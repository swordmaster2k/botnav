import math
import time
import threading
import _thread as thread
import RPi.GPIO as io

from threading import Thread
from connection import IPConnection
from motor import MotorController
from proxy import Proxy
from mice import Mice

CPI = 800.0
INCH_TO_METER = 0.0254
MESSAGE_RATE = 0.5 # ms.

COMMANDS[] = ['w', 's', 'a', 'd', 'q', 'r', 'e', 't', 'p', 'c']

io.setmode(io.BCM)

class Pirate(Thread):
	def __init__(self):
		self.proxy = Proxy(IPConnection("10.42.0.1", 50001))
		self.proxy.listeners.append(self)
		
		self.mice_exit_mutex = thread.allocate_lock()
		
		self.motor_controller = MotorController()
		self.mice = Mice(self, self.mice_exit_mutex)
		
		self.x = 0.0
		self.y = 0.0
		self.heading = 1.57
		
		self.looping = False
		self.interrupted = False
		
		Thread.__init__(self)
	
	'''
	
	'''
	def handle_event(self, event):
		self.process_command(event)
	
	'''
	
	'''	
	def process_command(self, command):
		if COMMANDS.count(command[0]) >= 1:
			if self.looping:
				self.interrupted = True
		
			state = ""
			
			if command[0] == 'w':	
				self.motor_controller.go_forward()
				state = "Going Forward\n"
			elif command[0] == 's':
				self.motor_controller.go_backward()
				state = "Going Backward\n"
			elif command[0] == 'a':
				self.motor_controller.rotate_left()
				state = "Turning Left\n"
			elif command[0] == 'd':
				self.motor_controller.rotate_right()
				state = "Turning Right\n"
			elif command[0] == 'q':
				self.motor_controller.halt()
				state = "Halted\n"
			elif command[0] == 'r':
				self.face(command)
			elif command[0] == 'e':
				self.scan()
			elif command[0] == 't':
				self.travel(command)
			elif command[0] == 'p':
				self.ping()
			elif command[0] == 'c':
				self.change_odometry(command)
			
			if state != "":	
				self.proxy.send(state)
	
	'''
	
	'''	
	def update_odometry(self, mouse_event):
		self.x += (mouse_event.x / CPI) * INCH_TO_METER
		self.y += (mouse_event.y / CPI) * INCH_TO_METER
		self.update_heading()
	
	'''
	
	'''
	def update_heading(self):
		return
		
	'''
	
	'''
	def send_odometry(self):
		self.proxy.send("o,%.2f" % self.x + ",%.2f" % self.y + ",%.2f" % self.heading + "\n")
	
	'''
	
	'''	
	def change_odometry(self, x, y, heading):
		return
		
	'''
	
	'''
	def face(self, heading):
		if len(command) == 5:
			command.strip('r')
			heading = float(command)
		
			if heading >= 0.0 and heading <= 6.27:
				angle = heading - self.heading
				
				if angle < -3.14:
					angle += 6.28
				elif angle > 3.14:
					angle -= 6.28
				
				left_buffer = angle + 0.01
				right_buffer = angle - 0.01
				
				if left_buffer > 6.28:
					left_buffer -= 6.28
					
				if right_buffer < 6.28:
					right_buffer += 6.28
				
				message_time = time.time()
					
				if angle < 0:
					self.motor_controller.rotate_right()
				if angle > 0:
					self.motor_controller.rotate_left()
					
				while self.heading < right_buffer or self.heading > left_buffer:
					self.looping = True
					
					if self.interrupted:
						self.looping= False
						self.interrupted = False
						break
					
					update_heading() # Perhaps update on own thread?
					
					if time.time() - message_time >= MESSAGE_RATE:
						self.send_odometry()
						message_time = time.time()
						
				self.motor_controller.halt()
				
				self.proxy.send("Current Heading: %.2f" % self.heading + "\n")
		
	'''
	
	'''
	def scan(self):
		return
	
	'''
	
	'''
	def ping(self):
		return
		
	'''
	
	'''
	def travel(self, command):
		if len(command) == 5:
			command.strip('t')
			distance = float(command)
			
			if distance > 0:
				start_x = self.x
				start_y = self.y
				travelled = 0.0
				distance -= 0.09 # Stop 0.09m short for decceleration.
				
				message_time = time.time()
				
				self.motor_controller.go_forward()
				
				while True:
					self.looping = True
					
					if self.interrupted:
						self.looping= False
						self.interrupted = False
						break
					
					travelled = math.sqrt(pow((start_x - self.x), 2) + 
					pow((start_y - self.y), 2))
					
					if travelled >= distance:
						break
						
					if time.time() - message_time >= MESSAGE_RATE:
						self.send_odometry()
						message_time = time.time()
				
				self.motor_controller.halt()
				self.proxy.send("travelled: %.2f" % travelled)

	'''
	
	'''
	def run(self):
		self.proxy.start()
		self.mice.start()

		message_time = time.time()
		
		while True:
			try:
				if time.time() - message_time >= MESSAGE_RATE:
					self.send_odometry()
					message_time = time.time()
			except Exception:
				print("Exception in main loop!")
				break
		
		self.proxy.socket.close()	
		self.mice.stream.close()
		
		print("waiting for mice thread")
		
		# Wait for mice thread to exit.
		while not self.mice_exit_mutex.locked():
			continue 
			
		io.cleanup()
		
		print("finished")
					
pirate = Pirate()
pirate.start()

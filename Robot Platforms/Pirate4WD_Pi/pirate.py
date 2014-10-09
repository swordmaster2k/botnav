'''

'''
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

CPI = 800.0 # CPI of mice attached to robot.
INCH_TO_METER = 0.0254
MESSAGE_RATE = 0.5 # 500ms.

# All possible commands for the robot.
COMMANDS = ['w', 's', 'a', 'd', 'q', 'r', 'e', 'p', 't', 'c']

io.setmode(io.BCM)

'''

'''
class Pirate(Thread):
	'''
	
	'''
	def __init__(self):
		# Create the communications thread.
		self.proxy = Proxy(IPConnection("10.42.0.1", 50001))
		
		# When the Mice thread is finished it will signal this
		# thread by acquiring the mutex.
		self.mice_exit_mutex = thread.allocate_lock()
		
		self.motor_controller = MotorController()
		
		# Create the Mice thread for reading x and y data.
		self.mice = Mice(self, self.mice_exit_mutex)
		
		self.x = 0.0
		self.y = 0.0
		self.heading = 1.57
		self.offset = 0.0 # Offset to record when the odometry has been forced.
		
		Thread.__init__(self)
	
	'''
	Processes a command recieved down the communications channel, if
	it is valid an action is taken.
	'''	
	def process_command(self, command):
		if COMMANDS.count(command[0]) < 1: # Valid command?
			return
		
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
			self.proxy.send(state) # Inform host of state change.
	
	'''
	Updates the robot's odometry (x, y, heading). Currently called from
	the Mice thread when there is movement.
	'''	
	def update_odometry(self, mouse_event):
		self.x += (mouse_event.x / CPI) * INCH_TO_METER
		self.y += (mouse_event.y / CPI) * INCH_TO_METER
		
		# Insert MPU6050 code here!
		
	'''
	Sends the robot's current odometry to the host.
	'''
	def send_odometry(self):
		self.proxy.send("o,%.2f" % self.x + ",%.2f" % self.y + ",%.2f" % self.heading + "\n")
	
	'''
	Changes the robot's odometry by force.
	'''	
	def change_odometry(self, x, y, heading):
		self.x = x
		self.y = x
		self.offset = self.heading - heading
		self.heading = heading
		
	'''
	Rotates the robot to face a particular heading.
	'''
	def face(self, command):
		if len(command) == 5:
			command = command.strip("r\n")
			heading = float(command)
		
			if heading >= 0.0 and heading <= 6.27:
				angle = heading - self.heading
				
				# Some wrap around.
				if angle < -3.14:
					angle += 6.28
				elif angle > 3.14:
					angle -= 6.28
				
				# Get with 0.5 degree of target heading.
				left_buffer = angle + 0.01
				right_buffer = angle - 0.01
				
				if left_buffer > 6.28:
					left_buffer -= 6.28
					
				if right_buffer < 6.28:
					right_buffer += 6.28
				
				message_time = time.time()
				
				# Our context is based on a standard circle with the
				# angle increasing from right to left.	
				if angle < 0:
					self.motor_controller.rotate_right()
				if angle > 0:
					self.motor_controller.rotate_left()
				
				interrupted = False
					
				while self.heading < right_buffer or self.heading > left_buffer:
					# Check for any commands that could cancel this.
					with self.proxy.command_mutex:
						if len(self.proxy.command_queue) > 0:
							interrupted = True
							break
					
					if time.time() - message_time >= MESSAGE_RATE:
						self.send_odometry()
						message_time = time.time()
						
				self.motor_controller.halt()
				self.proxy.send("Current Heading: %.2f" % self.heading + "\n")
				
				if interrupted:
					print("interrupted during rotation!")
					with self.proxy.command_mutex:
						self.process_command(self.proxy.command_queue.pop())
		
	'''
	
	'''
	def scan(self):
		return
	
	'''
	
	'''
	def ping(self):
		return
		
	'''
	Drives the robot forward a certain distance.
	'''
	def travel(self, command):
		print(command)
		if len(command) >= 3:
			command = command.strip("t\n")
			distance = float(command)
			
			if distance > 0:
				start_x = self.x
				start_y = self.y
				travelled = 0.0
				#distance -= 0.09 # Stop 0.09m short for decceleration.
				
				message_time = time.time()
				
				self.motor_controller.go_forward()
				
				interrupted = False
				
				while True:
					# Check for any commands that could cancel this.
					with self.proxy.command_mutex:
						if len(self.proxy.command_queue) > 0:
							interrupted = True
							break
					
					travelled = math.sqrt(pow((start_x - self.x), 2) + 
					pow((start_y - self.y), 2))
					
					if travelled >= distance:
						break
						
					if time.time() - message_time >= MESSAGE_RATE:
						self.send_odometry()
						message_time = time.time()
				
				self.motor_controller.halt()
				self.proxy.send("travelled: %.2f" % travelled + "\n")
				
				if interrupted:
					print("interrupted during travel!")
					with self.proxy.command_mutex:
						self.process_command(self.proxy.command_queue.pop())

	'''
	
	'''
	def run(self):
		self.proxy.start()
		self.mice.start()

		message_time = time.time()
		
		while True:
			try:
				command = ""
				
				# Check the command poll.
				with self.proxy.command_mutex:
					if len(self.proxy.command_queue) > 0:
						command = self.proxy.command_queue.pop()
				
				# If there is something then process it.
				if command != "":
					self.process_command(command)		
				
				if time.time() - message_time >= MESSAGE_RATE:
					self.send_odometry()
					message_time = time.time()
			except Exception as err:
				print(str(err))
				break
		
		self.proxy.socket.close()	
		self.mice.stream.close()
		
		print("Waiting for Mice thread to exit...")
		
		# Wait for Mice thread to exit.
		while not self.mice_exit_mutex.locked():
			continue 
		
		print("Cleaning up GPIO!")
		self.motor_controller.clean_up()	
		io.cleanup()
		
		print("Exiting Now!")
					
pirate = Pirate()
pirate.start()

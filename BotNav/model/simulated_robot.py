import math
import time

from .robot import Robot
from events import ScanResult

'''
A generic Robot class which (may) represent(s) a hardware robot that 
implements the communications interface defined by the robonav tool.
It is possible to use this class for simulations where no hardware
robot exists.

It can communicate with a hardware robot using Bluetooth, WiFi, 
Ethernet, Serial, InfraRed, etc. using the abstracted connection
approach.

This class keeps track of the robots position, orientation, the path it
has traversed, physical dimensions, state, and the cell resolution
it is operating in.

It does not matter if the robot is a wheeled, tracked, bipod, etc. as 
long as the hardware conforms to the generic interface required by
the robonav tool. 
'''
class SimulatedRobot(Robot):
	'''
	Initialises the robot using the connection specified.
	'''
	def __init__(self, connection):
		# Data connection to robot.
		self.connection = connection

		# Odometry, x and y are cell based.
		self.x = 0
		self.y = 0
		self.heading = 1.57

		# List of visited points.
		self.path = []
		self.path.append([self.x, self.y])

		# Physical dimensions in meters.
		self.width = 0.18
		self.lenght = 0.23

		# State string.
		self.state = ""

		# Size of the cells we are operating in.
		self.cell_size = 0.15

	'''
	Instructs the robot to go forward.
	'''
	def go_forward(self):
		return

	'''
	Instructs the robot to go backward.
	'''
	def go_backward(self):
		return

	'''
	Instructs the robot to rotate left.
	''' 
	def rotate_left(self):
		return

	'''
	Instructs the robot to rotate right.
	'''
	def rotate_right(self):
		return

	'''
	Instructs the robot to halt.
	'''
	def halt(self):
		return

	'''
	Instructs the robot to begin a scan.
	'''
	def scan(self):
		return

	'''
	Instructs the robot to ping.
	'''
	def ping(self):
		# Send a dummy scan result with a distance of 0.
		# Will have no effect.
		for listener in self.connection.listeners:
			listener.handle_event(ScanResult([0]))

	'''
	Instructs the robot to reset itself.
	'''
	def reset(self):
		self.x = 0
		self.y = 0
		self.heading = 1.57
		self.path = []
		self.state = ""

	'''
	Instructs the robot to update its odometry with the new parameters.
	'''
	def change_odometry(self, x, y, heading):
		x = round(x * self.cell_size, 2)
		y = round(y * self.cell_size, 2)

	'''
	Instructs the robot to rotate to face the specified heading.
	'''
	def rotate_to(self, heading):
		if (not (heading >= 0.0 and heading <= 6.28)):
			print("heading not within bounds: " + str(heading))
			return self.heading
		elif (self.heading == heading):
			print("already at heading: " + str(heading))
			return self.heading

		print("rotate_to: " + str(heading))

		self.heading = heading

		return heading

	'''
	Instructs the robot to travel a straight line distance.
	'''
	def travel_distance(self, distance):
		print("travel_distance: " + str(round(distance * self.cell_size, 2)))

		self.x = math.floor(self.x + (distance * math.cos(self.heading)))
		self.y = math.floor(self.y + (distance * math.sin(self.heading)))

	'''
	Instructs the robot to face a point.
	'''
	def face(self, x, y):
		dx = x - self.x
		dy = y - self.y

		alpha = math.atan2(dy, dx)
		beta = alpha - self.heading

		if beta < 0:
			beta += 6.28
		elif beta >= 6.28:
			beta -= 6.28

		heading = self.heading + beta

		if heading < 0:
			heading += 6.28
		elif heading >= 6.28:
			heading -= 6.28

		self.heading = heading

		return heading

	'''
	Instructs the robot to go a point.
	'''
	def go_to(self, x, y):
		self.x = x
		self.y = y
		
		self.state = "Travelled"
		
	'''
	Returns a boolean value to the caller indicating if there has 
	been a change.

	The update stores the x and y coordinates in meters so they must 
	be converted.
	'''
	def update_odometry(self, update):
		changed = False

		if self.x != (update.x / self.cell_size):
			self.x = (update.x / self.cell_size)
			changed = True

		if self.y != (update.y / self.cell_size):
			self.y = (update.y / self.cell_size)
			changed = True

		if self.heading != update.heading:
			self.heading = update.heading
			changed = True

		if self.heading < 0:
			self.heading += 6.28
		elif self.heading >= 6.28:
			self.heading -= 6.28

		if changed:
			self.path.append([self.x, self.y])

		return changed


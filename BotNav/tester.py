import time
import threading

from threading import Thread

from proxy import Proxy
from model.map import Map
from planner import Planner
from model.robot import Robot
from algorithm.gridnav import GridNav
from connection.bluetooth_connection import BluetoothConnection
from events import OdometryReport, ScanResult, StateEvent
from model.simulated_robot import SimulatedRobot

class Tester(threading.Thread):
	def __init__(self, map_file):
		'''
		self.proxy = Proxy(BluetoothConnection("00:00:12:06:56:83", 0x1001))
		self.proxy.listeners.append(self)
		
		self.robot = Robot(self.proxy)
		'''
		
		self.proxy = Proxy("DUMMY") # Dummy connection.
		self.proxy.listeners.append(self)
		
		self.robot = SimulatedRobot(self.proxy)
		
		self.open_map(map_file)
		
		self.algorithm = GridNav(self.map)
		self.planner = Planner(self.map, self.algorithm, self.proxy)
		
		Thread.__init__(self)

	def open_map(self, path):
		infile = open(path, 'r')
		
		# Do not bother with any validation for now.
		self.grid_size = float(infile.readline())
		self.cell_size = float(infile.readline())
		
		# Addition of the boundary.
		self.grid_size += self.cell_size
		self.grid_size = int(self.grid_size / self.cell_size)
		
		self.robot.cell_size = self.cell_size
		self.map = Map(self.robot, self.grid_size * self.cell_size, self.cell_size)
	
		y = self.grid_size - 1
	
		while y >= 0:
			line = infile.readline()
		
			for x in range(self.grid_size):
				if line[x] == "#":
					self.map.grid[x][y].state = 2
				elif line[x] == "R":
					# Put the robot in the center of the cell.
					self.robot.change_odometry(round(x + 0.5, 2), 
						round(y + 0.5, 2), 1.57)
					
					print("Waiting for odometry change...")
					
					# Wait for odometry change to take affect.	
					while (self.robot.x != round(x + 0.5, 2) and 
							self.robot.y != round(y + 0.5, 2)):
						continue
						
				elif line[x] == "G":
					self.map.goal_x = x
					self.map.goal_y = y
		
			y -= 1
		
		infile.close()
				
	def handle_event(self, event):
		if isinstance(event, OdometryReport):
			didChange = self.robot.update_odometry(event)                 
		elif isinstance(event, StateEvent):
			self.robot.state = event.state
			
	def run(self):
		self.proxy.start()
		
		command = ""
	
		while command != "quit":
			if self.planner.finished:
				break

			command = input()
	
			if command == "begin":
				if not self.planner.finished:
					self.planner.start()
			elif command == "quit":
				self.planner.finished = True
				
if __name__ == '__main__':
	import sys
	
	if len(sys.argv) == 2:
		# Just assume the information is correct for now.
		map_file = sys.argv[1]
	
		tester = Tester(map_file)
		tester.start()

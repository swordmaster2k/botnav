import time
import threading

from threading import Thread

from proxy import Proxy
from model.map import Map
from planner import Planner
from model.robot import Robot
from algorithm.gridnav import GridNav
from bluetooth_connection import BluetoothConnection
from events import OdometryReport, ScanResult, StateEvent

grid_size = 11
cell_size = 0.3 # meters

class Tester(threading.Thread):
	def __init__(self):
		self.proxy = Proxy(self, BluetoothConnection("00:00:12:06:56:83", 0x1001))
		self.robot = Robot(self.proxy)
		
		self.map = Map(self.robot, grid_size * cell_size, cell_size) # 10x10 grid.
		self.open_map("maps/test.map")
		
		self.algorithm = GridNav(self.map)
		
		self.planner = Planner(self.map, self.algorithm)
		
		Thread.__init__(self)

	def open_map(self, path):
		infile = open(path, 'r')
	
		y = grid_size - 1
	
		while y >= 0:
			line = infile.readline()
		
			x = 0
			while x < grid_size:
				if line[x] == "#":
					self.map.grid[x][y].state = 2
				elif line[x] == "R":
					# Put the robot in the center of the cell.
					self.robot.change_odometry(round(x + 0.5, 2), round(y + 0.5, 2), 1.57)
				elif line[x] == "G":
					self.map.goal_x = x
					self.map.goal_y = y
				
				x += 1
		
			y -= 1
		
		infile.close()
	
	def pop_event(self):
		# Attempt to get the lock and then process 
		# any events.
		with self.proxy.mutex:
			if len(self.proxy.events) < 1:
				return

		event = self.proxy.events.pop()
		
		print(event.to_string())

		if isinstance(event, OdometryReport):
			didChange = self.robot.update_odometry(event)
		elif isinstance(event, ScanResult):
			cells = self.map.ping_to_cells(float(event.readings[0])) # Just take 1 reading for now.
			updated_cells = self.map.update_map(cells)
				        
			if len(updated_cells) > 0:
				self.path_planner.update_occupancy(updated_cells)
				self.path_planner.print_occupancy_grid()                    
		elif isinstance(event, StateEvent):
			self.robot.state = event.state
			
	def run(self):
		command = ""
	
		while command != "quit":
			if self.planner.finished:
				break

			command = input()
	
			if command == "start":
				if not self.planner.finished:
					self.planner.start()
			elif command == "quit":
				self.planner.finished = True
					
tester = Tester()
tester.start()
tester.proxy.start()

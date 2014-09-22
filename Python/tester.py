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

grid_size = 22
cell_size = 0.15 # meters

class Tester(threading.Thread):
	def __init__(self):
		self.proxy = Proxy(BluetoothConnection("00:00:12:06:56:83", 0x1001))
		self.proxy.listeners.append(self)
		
		self.robot = Robot(self.proxy)
		self.robot.cell_size = cell_size
		
		self.map = Map(self.robot, grid_size * cell_size, cell_size)
		self.open_map("maps/bedroom15.map")
		
		self.algorithm = GridNav(self.map)
		
		self.planner = Planner(self.map, self.algorithm, self.proxy)
		
		Thread.__init__(self)

	def open_map(self, path):
		infile = open(path, 'r')
	
		y = grid_size - 1
	
		while y >= 0:
			line = infile.readline()
		
			for x in range(grid_size):
				if line[x] == "#":
					self.map.grid[x][y].state = 2
				elif line[x] == "R":
					# Put the robot in the center of the cell.
					self.robot.change_odometry(round(x + 0.5, 2), 
						round(y + 0.5, 2), 1.57)
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
					
tester = Tester()
tester.start()
tester.proxy.start()

import math
import time
import threading

from threading import Thread

class Planner(threading.Thread):
	def __init__(self, map, algorithm):
		self.map = map
		self.robot = self.map.robot
		self.algorithm = algorithm
		self.finished = False
		
		Thread.__init__(self)
		
	def run(self):
		self.algorithm.replan()
		self.algorithm.print_cost_grid()
		self.algorithm.print_occupancy_grid()
		
		x_difference = self.map.goal_x - self.robot.x
		y_difference = self.map.goal_y - self.robot.y
		
		if x_difference < 0:
			x_difference = -x_difference
			
		if y_difference < 0:
			y_difference = -y_difference
		
		print("cell x: %.2f" % self.robot.x + ", cell y: %.2f" % self.robot.y)
		print("x: %.2f" % (self.robot.x * self.map.cell_size) + ", y: %.2f" % (self.robot.y * self.map.cell_size))
		print("\n---------------------------------------------------------------------------------------------\n")
	
		while (x_difference > 0.5 or y_difference > 0.5):
			if self.finished:
				break
				
			print(math.ceil(self.robot.x) != self.map.goal_x)
			print(math.ceil(self.robot.y) != self.map.goal_y)
		
			#'''
			#Step 3: Scan the immediate area for obstacles and free space.
			#'''
			#affected_cells = scan_area()
			#
			#'''
			#Step 4: Update the map if necessary.
			#'''
			#if len(affected_cells) > 0:
			#	updated_cells = my_map.update_map(affected_cells)
			#	if len(updated_cells) > 0:
			#		gridnav.update_occupancy_grid(updated_cells)
			#		
			#		'''
			#		Step 5: Recompute the plan if necessary.
			#		'''
			#		gridnav.replan()

			'''
			Step 6: Check the plan for a new direction.
			'''
			new_point = self.algorithm.check_plan()
			new_point[0] = round(self.robot.x + new_point[0], 2)
			new_point[1] = round(self.robot.y + new_point[1], 2)
			self.robot.go_to(new_point[0], new_point[1])
			
			while self.robot.state != "Travelled":
				continue
				
			self.robot.state = "Halted" # Reset the state.
	
			# Print some information.
			self.algorithm.print_occupancy_grid()
			print("cell x: %.2f" % self.robot.x + ", cell y: %.2f" % self.robot.y)
			print("x: %.2f" % (self.robot.x * self.map.cell_size) + ", y: %.2f" % (self.robot.y * self.map.cell_size))
			print("\n---------------------------------------------------------------------------------------------\n")
			
			x_difference = self.map.goal_x - self.robot.x
			y_difference = self.map.goal_y - self.robot.y
		
			if x_difference < 0:
				x_difference = +x_difference
			
			if y_difference < 0:
				y_difference = +y_difference
				
		self.robot.halt()
		
		if not self.finished:
			self.finished = True

import time

from model.map import Map
from model.robot import Robot
from algorithm.gridnav import GridNav

grid_size = 11
cell_size = 0.3 # meters

# Position expressed in cells.
my_robot = Robot(0)

# Position expressed in cells.
my_map = Map(my_robot, grid_size * cell_size, cell_size) # 10x10 grid.

def open_map(path):
	infile = open(path, 'r')
	
	y = grid_size - 1
	
	while y >= 0:
		line = infile.readline()
		
		x = 0
		while x < grid_size:
			if line[x] == "#":
				my_map.grid[x][y].state = 2
			elif line[x] == "R":
				# Put the robot in the center of the cell.
				my_robot.x = x + 0.5
				my_robot.y = y + 0.5
			elif line[x] == "G":
				my_map.goal_x = x
				my_map.goal_y = y
				
			x += 1
		
		y -= 1
		
	infile.close()

open_map('maps/test.map')

'''
Step 1: Initialise the grid.
'''
gridnav = GridNav(my_map)

'''
Step 2: Plan
'''
gridnav.replan()

# Print some information.
gridnav.print_occupancy_grid()
print("cell x: %.2f" % my_robot.x + ", cell y: %.2f" % my_robot.y)
print("x: %.2f" % (my_robot.x * cell_size) + ", y: %.2f" % (my_robot.y * cell_size))
print("\n---------------------------------------------------------------------------------------------\n")

while (int(my_robot.x + 0.5) != my_map.goal_x) or (int(my_robot.y + 0.5) != my_map.goal_y):
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
	new_point = gridnav.check_plan()
	my_robot.x += new_point[0] # x component.
	my_robot.y += new_point[1] # y component.
	
	#time.sleep(0.25) # Wait for the robot to finish travelling...
	
	# Print some information.
	gridnav.print_occupancy_grid()
	print("cell x: %.2f" % my_robot.x + ", cell y: %.2f" % my_robot.y)
	print("x: %.2f" % (my_robot.x * cell_size) + ", y: %.2f" % (my_robot.y * cell_size))
	
	print("\n---------------------------------------------------------------------------------------------\n")
	
gridnav.print_cost_grid()


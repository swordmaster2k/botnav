import time

from model.map import Map
from model.robot import Robot
from algorithm.gridnav import GridNav

cell_size = 0.3

# Position expressed in meters.
my_robot = Robot(0)
my_robot.x = 2.4
my_robot.y = 0.3

# Position expressed in cells.
my_map = Map(my_robot, 3.0, 0.3)
my_map.goal_x = 1
my_map.goal_y = 8

gridnav = GridNav(my_map)
gridnav.replan()
gridnav.check_plan()
gridnav.print_cost_grid()

while (int((my_robot.x / cell_size) + 0.5) != my_map.goal_x) or (int((my_robot.y / cell_size) + 0.5) != my_map.goal_y):
	new_point = gridnav.check_plan()
	my_robot.x = cell_size * new_point[0] # x component.
	my_robot.y = cell_size * new_point[1] # y component.
	
	time.sleep(0.5) # Wait for the robot to finish travelling...
	
	print("x: %f" % my_robot.x + ", y: %f" % my_robot.y)

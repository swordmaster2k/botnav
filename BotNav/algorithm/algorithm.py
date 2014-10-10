'''
Super class that all path planning algorithms should inherit from. 
The sub classes should provide the logic behind the unimplemented
methods here.
'''
class Algorithm:
	'''
	Default constructor simply assigns the map attribute.
	'''
	def __init__(self, map):
		self.map = map
		self.robot = self.map.robot
		self.path = []
		
	'''
	Subclass should implement the logic behind calculating its cost
	grid and gradients to the goal.
	'''
	def replan(self):
		raise NotImplementedError
		
	'''
	Subclass should use this method to get the next move for the robot.
	
	Expected to return a point [x, y].
	'''
	def check_plan(self):
		raise NotImplementedError
	
	'''
	Pops the next point from the current path.
	
	Returns the next point to travel too, or -1 if there are none.
	'''	
	def pop_next_point(self):
		try:
			return self.path.pop(0)
		except IndexError as err:
			print(str(err))
			return -1
		
	'''
	Subclass should use this method to update the state of its
	occupancy grid based on the updated cells provided.
	'''
	def update_occupancy_grid(self, cells):
		raise NotImplementedError
	
	'''
	Should print the cost grid to standard output.
	'''	
	def print_cost_grid(self):
		raise NotImplementedError
	
	'''
	Should print the occupancy grid to standard output.
	'''	
	def print_occupancy_grid(self):
		raise NotImplementedError

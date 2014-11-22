'''
Need to expose the path to C code.

Ability to update cells that are either occupied or not.
'''
class DStarLite(Algorithm):
	'''
	Default constructor simply assigns the map attribute.
	'''
	def __init__(self, map):
		Algorithm.__init__(self, map)
		
	'''
	use underlying computeshortestpath in C version of D* Lite
	'''
	def plan(self):
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
	probably need to work it out with updatemaze in C version
	'''
	def update_occupancy_grid(self, cells):
		raise NotImplementedError
	
	'''
	not implemented in D* Lite handle this Error
	'''	
	def print_cost_grid(self):
		raise NotImplementedError
	
	'''
	not implemented in D* Lite handle this Error
	'''	
	def print_occupancy_grid(self):
		raise NotImplementedError
	
	'''
	redirect to C implementation
	'''	
	def print_path(self):
		raise NotImplementedError

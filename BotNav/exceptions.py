'''
Raised when no path to the goal can be found.
'''
class NoPathException(Exception):
	def __init__(self, value):
		self.value = value
	
	def __str__(self):
		return self.value

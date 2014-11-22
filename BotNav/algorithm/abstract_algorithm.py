"""
Super class that all path planning algorithms should inherit from.
The sub classes should provide the logic behind the unimplemented
methods here.
"""


class AbstractAlgorithm:
    """
    Default constructor simply assigns the map attribute.
    """

    def __init__(self, map_state):
        self.map_state = map_state
        self.robot = self.map_state.robot

        self.path = []  # Contains a list of (x, y) points in cell units.

        # For debugging.
        self.vertex_accesses = 0  # Number of times any vertex is accessed.
        self.time_taken = 0  # Total execution time in seconds of every planning stage.

    '''
    Subclass should implement the logic behind calculating its cost
    grid and gradients to the goal.

    time_taken will have its value incremented here using += operator
    just before the function returns.
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

    '''
    Should print the contents of the robots path.
    '''

    def print_path(self):
        raise NotImplementedError

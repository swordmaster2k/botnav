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
        self.time_taken = 0  # The sum of every planning execution time in seconds.
        self.total_plan_steps = 0  # Total number of calls to plan.

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

    def print_cost_grid(self, out):
        raise NotImplementedError

    '''
    Should print the occupancy grid to standard output.
    '''

    def print_occupancy_grid(self, out):
        raise NotImplementedError

    '''
    Should print the contents of the robots path.
    '''

    def print_path(self, out):
        raise NotImplementedError

    '''
    Prints the final state of all debugging information to a stream.
    '''

    def print_debug(self, out):
        out.write(('-' * 120) + "\n\n")
        out.write("Total Planning Steps: " + str(self.total_plan_steps) + "\n")
        out.write("Total Vertices: " + str(self.map_state.cells_square ** 2) + "\n\n")

        out.write("Vertex Accesses: " + str(self.vertex_accesses) + "\n")
        out.write("Average: " + str(self.vertex_accesses / self.total_plan_steps) + "\n\n")

        out.write("Total Time Taken: " + str(self.time_taken) + "s\n")
        out.write("Average: " + str(self.time_taken / self.total_plan_steps) + "s\n")

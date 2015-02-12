

class AbstractAlgorithm:
    """
    Super class that all path planning algorithms should inherit from.
    The sub classes should provide the logic behind the unimplemented
    methods here.
    """

    def __init__(self, map_state):
        """
        Default constructor simply assigns the map attribute.

        :param map_state:
        :return:
        """

        self.planner_name = "Abstract"

        self.map_state = map_state
        self.robot = self.map_state.robot

        self.path = []  # Contains a list of (x, y) points in cell units.
        self.do_smooth_path = True  # Enable global path smoothing?

        # For debugging.
        self.vertex_accesses = 0  # Number of times any vertex is accessed.
        self.time_taken = 0  # The sum of every planning execution time in seconds.
        self.total_plan_steps = 0  # Total number of calls to plan.

    '''

    '''

    def plan(self):
        """
        Subclass should implement the logic behind calculating its cost
        grid and gradients to the goal.

        time_taken will have its value incremented here using += operator
        just before the function returns.

        :return:
        """

        raise NotImplementedError

    def pop_next_point(self):
        """
        Pops the next point from the current path.

        Returns the next point to travel too, or -1 if there are none.

        :return:
        """

        try:
            return self.path.pop(0)
        except IndexError as err:
            print(str(err))
            return -1

    def update_occupancy_grid(self, cells):
        """
        Subclass should use this method to update the state of its
        occupancy grid based on the updated cells provided.

        :param cells:
        :return:
        """

        raise NotImplementedError

    def smooth(self, weight_data=0.1, weight_smooth=0.1,
               tolerance=0.000001):
        """
        Taken from Sebastian Thrun's Udacity program code.

        :param weight_data:
        :param weight_smooth:
        :param tolerance:
        :return:
        """

        if len(self.path) < 5:
            return

        spath = [[0 for row in range(len(self.path[0]))]for col in range(len(self.path))]

        for i in range(len(self.path)):
            for j in range(len(self.path[0])):
                spath[i][j] = self.path[i][j]

        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(self.path) - 1):
                for j in range(len(self.path[0])):
                    aux = spath[i][j]

                    spath[i][j] += weight_data * (self.path[i][j] - spath[i][j])

                    spath[i][j] += weight_smooth * (spath[i - 1][j] + spath[i + 1][j] - (2.0 * spath[i][j]))
                    if i >= 2:
                        spath[i][j] += 0.5 * weight_smooth * (2.0 * spath[i - 1][j] - spath[i - 2][j] - spath[i][j])
                    if i <= len(self.path) - 3:
                        spath[i][j] += 0.5 * weight_smooth * (2.0 * spath[i + 1][j] - spath[i + 2][j] - spath[i][j])

            change += abs(aux - spath[i][j])

    def print_path(self, stream):
        """
        Prints the points in the robots path.

        :param stream: output stream
        :return: none
        """

        path = "path: "

        for i in range(len(self.path)):
            path += str(self.path[i])

            if i != len(self.path) - 1:
                path += "->"

        stream.write(path + "\n\n")

    def print_cost_grid(self, stream):
        """
        Should print the cost grid to standard output.

        :param stream:
        :return:
        """

        raise NotImplementedError

    def print_occupancy_grid(self, stream):
        """
        Should print the occupancy grid to standard output.

        :param stream:
        :return:
        """

        raise NotImplementedError

    def print_debug(self, stream):
        """
        Prints the final state of all debugging information to a stream.

        :param stream:
        :return:
        """

        stream.write(('-' * 120) + "\n\n")
        stream.write("Planner: " + self.planner_name + "\n\n")

        stream.write("Total Planning Steps: " + str(self.total_plan_steps) + "\n")
        stream.write("Total Vertices: " + str(self.map_state.cells_square ** 2) + "\n\n")

        stream.write("Vertex Accesses: " + str(self.vertex_accesses) + "\n")
        stream.write("Average: " + str(self.vertex_accesses / self.total_plan_steps) + "\n\n")

        stream.write("Total Planning Time (seconds): " + str(self.time_taken) + "\n")
        stream.write("Average Planning Time (seconds): " + str(self.time_taken / self.total_plan_steps) + "\n\n")

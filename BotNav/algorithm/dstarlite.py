import time
import dstarlite  # Import dstarlite C module.

from .abstract_algorithm import AbstractAlgorithm


class DStarLite(AbstractAlgorithm):
    def __init__(self, map_state):
        AbstractAlgorithm.__init__(self, map_state)

    @staticmethod
    def setup(self, file_path):
        """

        :param file_path:
        :return:
        """

        dstarlite.setup(file_path)

    def plan(self):
        self.total_plan_steps += 1
        start_time = time.process_time()

        self.path = dstarlite.plan()
        self.vertex_accesses = dstarlite.getvertexaccesses()

        self.time_taken += round(time.process_time() - start_time, 3)

    def update_robot_position(self):
        dstarlite.updaterobotposition(self.robot.x, self.robot.y)

    def update_occupancy_grid(self, cells):
        for cell in cells:
            '''
            If it's a cell on the boundary or the goal ignore it
            and continue.
            '''
            if ((cell.x == 0) or (cell.x == (self.map_state.cells_square - 1)) or
                    (cell.y == 0) or (cell.y == (self.map_state.cells_square - 1))):
                continue
            elif cell.x == self.map_state.goal_x and cell.y == self.map_state.goal_y:
                continue

            # Redirect to C module which is storing all the state information.
            dstarlite.updatecelloccupancy(cell.x, cell.y, cell.state)

    def print_path(self, stream):
        dstarlite.printpath(stream)

    def print_cost_grid(self, stream):
        dstarlite.printcostgrid(stream)

    def print_occupancy_grid(self, stream):
        dstarlite.printoccupancygrid(stream)

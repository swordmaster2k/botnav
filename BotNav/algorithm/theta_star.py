import math
import time

from .abstract_algorithm import AbstractAlgorithm


class Node:
    def __init__(self, x, y, walkable=True):
        self.x = x
        self.y = y
        self.g = 1000000
        self.h = 1000000
        self.f = 1000000
        self.evaluations = 0
        self.previous = None
        self.walkable = walkable

    def __str__(self):
        return "x: " + str(self.x) + " y: " + str(self.y) + " g: " + str(self.g) + " h: " + str(self.h) + \
               " f: " + str(self.f) + " evaluations: " + str(self.evaluations)

    def __gt__(self, other):
        return self.f < other.f

    def __lt__(self, other):
        return self.f > other.f


class ThetaStar(AbstractAlgorithm):
    def __init__(self, map_state):
        AbstractAlgorithm.__init__(self, map_state)
        self.planner_name = "Theta*"

        self.BIG_COST = 1000000

        self.open = []
        self.closed = []

        self.straight_cost = 1
        self.diagonal_cost = 2

        self.start_node = None
        self.goal_node = None
        self.nodes = None

    def get_cell_x(self, x):
        return int(x / self.map_state.cell_size)

    def get_cell_y(self, y):
        return int(y / self.map_state.cell_size)

    def setup_nodes(self):
        """
        Setup each node in the grid.

        :return: none
        """
        # +1 here as nodes sit a cell corners meaning that there are n + 1 nodes where n is the number of cells.
        for x in range(self.map_state.cells_square):
            column = []
            for y in range(self.map_state.cells_square):
                if x == self.map_state.goal_x and y == self.map_state.goal_y:  # Goal
                    column.append(self.goal_node)
                elif x == self.map_state.robot.x and y == self.map_state.robot.y:  # Start
                    column.append(self.start_node)
                elif self.map_state.grid[x][y].state == 2:
                    column.append(Node(x, y, False))
                else:
                    column.append(Node(x, y))
            self.nodes.append(column)

    def plan(self):
        self.open = []
        self.closed = []

        self.start_node = Node(self.robot.get_cell_x(), self.robot.get_cell_y())
        self.goal_node = Node(self.map_state.goal_x, self.map_state.goal_y)

        self.nodes = []
        self.setup_nodes()

        self.start_node.previous = self.start_node
        self.start_node.g = 0
        self.start_node.h = self.euclidean(self.start_node)
        self.start_node.f = self.start_node.h

        self.total_plan_steps += 1
        start_time = time.process_time()

        self.compute_shortest_path()

        self.time_taken += round(time.process_time() - start_time, 3)

        if self.do_smooth_path:
            self.smooth()

    def compute_shortest_path(self):
        node = self.start_node

        while node != self.goal_node:
            start_x = max(0, node.x - 1)
            end_x = min(self.map_state.cells_square - 1, node.x + 1)
            start_y = max(0, node.y - 1)
            end_y = min(self.map_state.cells_square - 1, node.y + 1)

            for x in range(int(start_x), int(end_x + 1)):
                for y in range(int(start_y), int(end_y + 1)):
                    neighbour = self.nodes[x][y]

                    self.vertex_accesses += 1

                    if neighbour == node or not neighbour.walkable:
                        continue

                    previous = node

                    if self.raytrace(node.previous, neighbour):
                        cost = self.euclidean(node.previous, neighbour)
                        g = node.previous.g + cost
                        h = self.euclidean(neighbour)
                        f = g + h
                        previous = node.previous
                    else:
                        if node.x != neighbour.x or node.y != neighbour.y:
                            cost = self.diagonal_cost
                        else:
                            cost = self.straight_cost

                        g = node.g + cost
                        h = self.euclidean(neighbour)
                        f = g + h

                    if self.open.count(neighbour) > 0 or self.closed.count(neighbour) > 0:
                        if neighbour.f > f:
                            neighbour.f = f
                            neighbour.g = g
                            neighbour.h = h
                            neighbour.previous = previous
                    else:
                        neighbour.f = f
                        neighbour.g = g
                        neighbour.h = h
                        neighbour.previous = previous
                        self.open.insert(0, neighbour)

            self.closed.insert(0, node)

            if len(self.open) == 0:
                print("no path found")
                return

            self.open.sort()
            node = self.open.pop()

        self.build_path()

    def build_path(self):
        self.path = []
        node = self.goal_node.previous
        self.path.insert(0, (self.goal_node.x, self.goal_node.y))

        while node != self.start_node:
            self.path.insert(0, (node.x, node.y))
            node = node.previous

    def euclidean(self, node, end_tile=None):
        if end_tile is None:
            end_tile = self.goal_node

        dx = node.x - end_tile.x
        dy = node.y - end_tile.y

        return math.sqrt(dx * dx + dy * dy) * self.straight_cost

    def raytrace(self, start, end):
        x0 = start.x
        y0 = start.y
        x1 = end.x
        y1 = end.y
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        x = x0
        y = y0

        if x1 > x0:
            x_inc = 1
        else:
            x_inc = -1

        if y1 > y0:
            y_inc = 1
        else:
            y_inc = -1

        error = dx - dy

        n = dx + dy

        while n > 0:
            node = self.nodes[int(x)][int(y)]

            if not node.walkable:
                return False

            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx

            n -= 1

        return True

    def print_cost_grid(self, stream):
        """
        Prints the contents of the cost grid to the standard output.

        :param stream: output stream
        :return:
        """

        y = self.map_state.cells_square - 1
        footer = ""
        rows = ""
        start_spacing = ""
        end_spacing = ""
        grid = self.map_state.grid

        while y >= 0:
            if y < 10:
                rows += str(y) + "  "
            elif 10 <= y < 100:
                rows += str(y) + " "
            else:
                rows += str(y)

            cell = (self.map_state.cells_square - 1) - y

            if cell < 1:
                start_spacing = "        "
                end_spacing = "     "
            elif cell < 10:
                start_spacing = "    "
                end_spacing = "     "
            elif 10 < cell < 100:
                start_spacing = "   "
                end_spacing = "     "
            elif 100 < cell < 1000:
                start_spacing = "  "
                end_spacing = "      "

            footer += start_spacing + str((self.map_state.cells_square - 1) - y) + end_spacing

            for x in range(self.map_state.cells_square):
                cost = self.nodes[x][y].h
                padding = ""

                if cost < 10:
                    padding = "  "
                elif cost < 100:
                    padding = " "

                rows += "[ %.2f" % cost + padding + " ]"

            y -= 1

            if y >= 0:
                rows += "\n\n"

        stream.write(rows + "\n")
        stream.write(footer + "\n\n")

    def print_occupancy_grid(self, stream):
        """
        Prints the contents of the occupancy grid to the standard output.

        :param stream: output stream
        :return: none
        """

        y = self.map_state.cells_square - 1
        footer = ""
        rows = ""
        start_spacing = ""
        end_spacing = ""
        grid = self.map_state.grid

        while y >= 0:
            if y < 10:
                rows += str(y) + "  "
            elif 10 <= y < 100:
                rows += str(y) + " "
            else:
                rows += str(y)

            cell = (self.map_state.cells_square - 1) - y

            if cell < 1:
                start_spacing = "       "
                end_spacing = "    "
            elif cell < 10:
                start_spacing = "    "
                end_spacing = start_spacing
            elif 10 < cell < 100:
                start_spacing = "   "
                end_spacing = "    "
            elif 100 < cell < 1000:
                start_spacing = "  "
                end_spacing = "     "

            footer += start_spacing + str((self.map_state.cells_square - 1) - y) + end_spacing

            for x in range(self.map_state.cells_square):
                symbol = "     "

                if x == int(self.robot.get_cell_x()) and y == int(self.robot.get_cell_y()):
                    symbol = "ROBOT"
                elif x == self.map_state.goal_x and y == self.map_state.goal_y:
                    symbol = "GOAL "
                elif not self.nodes[x][y].walkable:
                    symbol = "#####"
                else:
                    for point in self.path:
                        if (math.floor(point[0]) == x and
                                math.floor(point[1]) == y):
                            symbol = "  *  "

                rows += "[ " + symbol + " ]"

            y -= 1

            if y >= 0:
                rows += "\n\n"

        stream.write(rows + "\n")
        stream.write(footer + "\n\n")
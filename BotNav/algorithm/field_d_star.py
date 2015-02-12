import math
import time

from .abstract_algorithm import AbstractAlgorithm

SQRT_2 = 1.414213562


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = 1000000
        self.rhs = 1000000
        self.key = [None, None]
        self.evaluations = 0

    def __str__(self):
        return "x: " + str(self.x) + " y: " + str(self.y) + " g: " + str(self.g) + " rhs: " + str(self.rhs) + \
               " evaluations: " + str(self.evaluations)


class FieldDStar(AbstractAlgorithm):
    """

    """

    def __init__(self, map_state):
        AbstractAlgorithm.__init__(self, map_state)
        self.planner_name = "Field D*"

        self.BIG_COST = 1000000
        self.NEIGHBOUR_DIRECTIONS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1,), (1, -1)]

        self.s_goal = Node(map_state.goal_x, map_state.goal_y)
        self.s_start = Node(map_state.robot.get_cell_x(), map_state.robot.get_cell_y())

        self.nodes = []
        self.setup_nodes()

        self.setup_occupancy_grid()

        self.open_list = []

        self.key(self.s_start)

        self.s_goal.g = self.BIG_COST
        self.s_goal.rhs = 0
        self.key(self.s_goal)
        self.s_goal.key[1] = 0

        self.open_list.append(self.s_goal)

    def setup_nodes(self):
        """
        Setup each node in the grid.

        :return: none
        """

        # +1 here as nodes sit a cell corners meaning that there are n + 1 nodes where n is the number of cells.
        for x in range(self.map_state.cells_square + 1):
            column = []
            for y in range(self.map_state.cells_square + 1):
                if x == self.s_goal.x and y == self.s_goal.y:  # Goal
                    column.append(self.s_goal)
                elif x == self.s_start.x and y == self.s_start.y:  # Start
                    column.append(self.s_start)
                else:
                    column.append(Node(x, y))
            self.nodes.append(column)

    def setup_occupancy_grid(self):
        """
        Setup each cell in the occupancy grid based on their content i.e.
        it contains the goal, an obstacle, or free space.

        :return: none
        """

        for x in range(self.map_state.cells_square):
            for y in range(self.map_state.cells_square):
                '''
                If it's a cell on the boundary or occupied set it to a high cost.
                '''
                if ((x == 0) or (x == (self.map_state.cells_square - 1)) or
                        (y == 0) or (y == (self.map_state.cells_square - 1))) or \
                        self.map_state.grid[x][y].state == 2:
                    data = 1
                else:
                    data = 1

                self.map_state.grid[x][y].data = data

    @staticmethod
    def get_manhattan(s, s1):
        return abs(s.y - s1.y) + abs(s.x - s1.x)

    @staticmethod
    def get_distance(s, s1):
        return round(math.sqrt((s.y - s1.y) ** 2 + (s.x - s1.x) ** 2), 3)

    def h(self, s):
        # Heuristic based on straight line (euclidean) distance between two points.
        return round(math.sqrt((s.y - self.s_start.y) ** 2 + (s.x - self.s_start.x) ** 2), 3)

    def key(self, s):
        if s.g < s.rhs:
            s.key[0] = s.g + self.h(s)
            s.key[1] = s.g
        else:
            s.key[0] = s.rhs + self.h(s)
            s.key[1] = s.rhs

    def get_neighbours(self, s):
        neighbours = []

        for direction in self.NEIGHBOUR_DIRECTIONS:
            try:
                neighbours.append(self.nodes[s.x + direction[0]][s.y + direction[1]])
            except IndexError:
                pass

        return neighbours

    def get_consecutive_neighbours(self, s):
        consecutive_neighbours = []

        for i in range(len(self.NEIGHBOUR_DIRECTIONS)):
            try:
                if i == len(self.NEIGHBOUR_DIRECTIONS) - 1:  # Edge s8 -> s1
                    x1 = s.x + self.NEIGHBOUR_DIRECTIONS[i][0]
                    y1 = s.y + self.NEIGHBOUR_DIRECTIONS[i][1]
                    x2 = s.x + self.NEIGHBOUR_DIRECTIONS[0][0]
                    y2 = s.y + self.NEIGHBOUR_DIRECTIONS[0][1]

                    if x1 > -1 and y1 > -1 and x2 > -1 and y2 > -1:
                        consecutive_neighbours.append(
                            (self.nodes[s.x + self.NEIGHBOUR_DIRECTIONS[i][0]][s.y + self.NEIGHBOUR_DIRECTIONS[i][1]],
                             self.nodes[s.x + self.NEIGHBOUR_DIRECTIONS[0][0]][s.y + self.NEIGHBOUR_DIRECTIONS[0][1]]))
                else:  # All other edges.
                    x1 = s.x + self.NEIGHBOUR_DIRECTIONS[i][0]
                    y1 = s.y + self.NEIGHBOUR_DIRECTIONS[i][1]
                    x2 = s.x + self.NEIGHBOUR_DIRECTIONS[i + 1][0]
                    y2 = s.y + self.NEIGHBOUR_DIRECTIONS[i + 1][1]

                    if x1 > -1 and y1 > -1 and x2 > -1 and y2 > -1:
                        consecutive_neighbours.append(
                            (self.nodes[s.x + self.NEIGHBOUR_DIRECTIONS[i][0]][s.y + self.NEIGHBOUR_DIRECTIONS[i][1]],
                             self.nodes[s.x + self.NEIGHBOUR_DIRECTIONS[i + 1][0]][
                                 s.y + self.NEIGHBOUR_DIRECTIONS[i + 1][1]]))

            except IndexError:
                pass
            except AttributeError:
                pass

        return consecutive_neighbours

    @staticmethod
    def is_diagonal_neighbour(s, sa):
        x = s.x - sa.x
        y = s.y - sa.y

        if x == 1 or x == -1:
            if y == 1 or y == -1:
                return True

        return False

    def get_cell_cost(self, x, y):
        try:
            return self.map_state.grid[x][y].data
        except IndexError:
            return self.BIG_COST

    def compute_cost(self, s, sa, sb):
        self.vertex_accesses += 1
        s.evaluations += 1

        if self.is_diagonal_neighbour(s, sa):
            s1 = sb
            s2 = sa
        else:
            s1 = sa
            s2 = sb

        # Get mid point of diagonal neighbours s and s2.
        mid_x = (s.x + s2.x) / 2
        mid_y = (s.y + s2.y) / 2

        # Map mid_x and mid_y to a cell cost, if the x and y is out of
        # bounds c == LARGE.
        c = self.get_cell_cost(math.floor(mid_x), math.floor(mid_y))

        # Get the difference between (s, s2) and (s, s1) mid-points.
        difference_x = mid_x - ((s.x + s1.x) / 2)
        difference_y = mid_y - ((s.y + s1.y) / 2)

        if s.x == s1.x:
            if difference_x > 0:
                # b is the traversal cost of the cell to the left.
                b = self.get_cell_cost(math.floor(mid_x) - 1, math.floor(mid_y))
            else:
                # b is the traversal cost of the cell to the right.
                b = self.get_cell_cost(math.floor(mid_x) + 1, math.floor(mid_y) - 1)
        elif s.y == s1.y:
            if difference_y > 0:
                # b is the traversal cost of the cell immediately below.
                b = self.get_cell_cost(math.floor(mid_x), math.floor(mid_y) - 1)
            else:
                # b is the traversal cost of the cell immediately above.
                b = self.get_cell_cost(math.floor(mid_x), math.floor(mid_y) + 1)
        else:
            b = self.BIG_COST

        if min(c, b) == self.BIG_COST:
            vs = self.BIG_COST
        elif s1.g <= s2.g:
            vs = min(c, b) + s1.g
        else:
            f = s1.g - s2.g

            if f <= b:
                if c <= f:
                    vs = c * SQRT_2 + s2.g
                else:
                    y = min(f / (math.sqrt(c ** 2 - f ** 2)), 1)

                    vs = c * math.sqrt(1 + y ** 2) + f * (1 - y) + s2.g
            else:
                if c <= b:
                    vs = c * SQRT_2 + s2.g
                else:
                    x = 1 - min(b / (math.sqrt(c ** 2 - b ** 2)), 1)

                    vs = c * math.sqrt(1 + ((1 - x) ** 2)) + (b * x) + s2.g

        if vs > self.BIG_COST:
            vs = self.BIG_COST

        return round(vs, 3)

    def update_state(self, s):
        if s.g == self.BIG_COST:
            s.g = self.get_manhattan(s, self.s_goal)

        if s is not self.s_goal:
            rhs = s.rhs

            for edge in self.get_consecutive_neighbours(s):
                cost = self.compute_cost(s, edge[0], edge[1])  # s, sa, sb

                if cost < rhs:
                    rhs = cost

            if rhs < s.rhs:
                s.rhs = rhs
                print("lowered cost of " + str(s) + "\n")

        if self.open_list.count(s) > 0:
            self.open_list.remove(s)

        if s.g != s.rhs:
            self.key(s)

            if len(self.open_list) == 0:
                self.open_list.append(s)
            else:
                for i in range(len(self.open_list)):
                    if s.key[0] > self.open_list[i].key[0] and s.key[1] > self.open_list[i].key[1]:
                        self.open_list.insert(i, s)
                        return

                self.open_list.append(s)

    def plan(self):
        self.total_plan_steps += 1
        start_time = time.process_time()

        self.compute_shortest_path()

        self.time_taken += round(time.process_time() - start_time, 3)

        if self.do_smooth_path:
            self.smooth()

    def compute_shortest_path(self):
        while (self.open_list[0].key[0] < self.s_start.key[0] and self.open_list[0].key[1] < self.s_start.key[1]) or \
                self.s_start.rhs != self.s_start.g:
            s = self.open_list.pop()

            if s.g > s.rhs:
                s.g = s.rhs

                for neighbour in self.get_neighbours(s):
                    self.update_state(neighbour)
            else:
                s.g = self.BIG_COST

                for neighbour in self.get_neighbours(s):
                    self.update_state(neighbour)

            if len(self.open_list) == 0:
                break

            s = self.open_list.pop()

        for column in self.nodes:
            for node in column:
                print(str(node))

        # Extract the path.
        while self.s_start.x != self.s_goal.x or self.s_start.y != self.s_goal.y:
            consecutive_neighbours = self.get_consecutive_neighbours(Node(int(round(self.s_start.x, 0)),
                                                                          int(round(self.s_start.y, 0))))

            if len(consecutive_neighbours) > 0:
                lowest_edge_cost = self.BIG_COST
                lowest_pair = None

                for edge in consecutive_neighbours:
                    if edge[0].g + edge[1].g < lowest_edge_cost:
                        lowest_edge_cost = edge[0].g + edge[1].g
                        lowest_pair = edge

                if lowest_pair is not None:
                    if lowest_pair[0] == self.s_goal:
                        self.s_start.x = lowest_pair[0].x
                        self.s_start.y = lowest_pair[0].y
                    elif lowest_pair[1] == self.s_goal:
                        self.s_start.x = lowest_pair[1].x
                        self.s_start.y = lowest_pair[1].y
                    else:

                        #print(str(lowest_pair[0]))
                        #print(str(lowest_pair[1]))

                        # Always ensure that s1 has the highest g-value.
                        if lowest_pair[0].g < lowest_pair[1].g:
                            s1 = lowest_pair[1]
                            s2 = lowest_pair[0]
                        else:
                            s1 = lowest_pair[0]
                            s2 = lowest_pair[1]

                        f = s1.g - s2.g
                        #print("f: " + str(f))
                        #print('\n')

                        x_difference = s1.x - s2.x
                        y_difference = s1.y - s2.y
                        x_shift = 0
                        y_shift = 0

                        if x_difference != 0:
                            if x_difference < 0:
                                x_shift = f
                            else:
                                x_shift = -f
                        else:
                            if y_difference < 0:
                                y_shift = f
                            else:
                                y_shift = -f

                        if x_shift != 0:
                            self.s_start.x += x_shift
                            self.s_start.y = s2.y
                        elif y_shift != 0:
                            self.s_start.x = s2.x
                            self.s_start.y += y_shift

                    self.path.append((self.s_start.x, self.s_start.y))

    def update_occupancy_grid(self, cells):
        """
        Updates the occupancy grid based on the cells that were updated on
        the map.

        NOTE: NEEDS WORK BUT CURRENTLY NOT BEING USED!!!

        :param cells: affected cells
        :return: none
        """

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

            occupancy = self.map_state.grid[cell.x][cell.y].data.occupancy

            if cell.state == 0 and occupancy != self.EMPTY:
                self.map_state.grid[cell.x][cell.y].data.occupancy = self.EMPTY
            elif cell.state == 1 and occupancy != self.EMPTY:
                self.map_state.grid[cell.x][cell.y].data.occupancy = self.EMPTY
            elif cell.state == 2 and occupancy != self.FULL:
                self.map_state.grid[cell.x][cell.y].data.occupancy = self.FULL

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
                cost = grid[x][y].data
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
                elif grid[x][y].state == 2:
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

    def print_debug(self, stream):
        """
        Prints the final state of all debugging information to a stream.

        :param stream:
        :return:
        """

        stream.write(('-' * 120) + "\n\n")
        stream.write("Planner: " + self.planner_name + "\n\n")

        stream.write("Total Planning Steps: " + str(self.total_plan_steps) + "\n")
        stream.write("Total Vertices: " + str(len(self.nodes) * len(self.nodes[0])) + "\n\n")

        stream.write("Vertex Accesses: " + str(self.vertex_accesses) + "\n")
        stream.write("Average: " + str(self.vertex_accesses / self.total_plan_steps) + "\n\n")

        stream.write("Total Planning Time (seconds): " + str(self.time_taken) + "\n")
        stream.write("Average Planning Time (seconds): " + str(self.time_taken / self.total_plan_steps) + "\n\n")
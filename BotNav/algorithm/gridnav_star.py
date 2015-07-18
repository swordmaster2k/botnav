"""
    gridnav.py

    Version 3.0 - more efficient.  A grid-based navigation algorithm for
    mobile robots.

    Python port of the original C implementation by Tucker Balch, ported
    by Joshua Michael Daly. Modified to build the entire path and catch
    exceptions where there is no path to the goal.

    Copyright (C) 1995 Tucker Balch
    Copyright (C) 2014 Joshua Michael Daly

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
"""

import time
import math

from .abstract_algorithm import AbstractAlgorithm
from exceptions import NoPathException


class GridNavStar(AbstractAlgorithm):
    """
    A grid based navigation algorithm for mobile robots. Closest to the
    Shortest Path solution presented by Dijkstra.
    """

    def __init__(self, map_state):
        """
        Initialises the GridNav algorithm with its default settings.

        :param map_state: state space of the map we are operating in
        :return: a new GridNav planner
        """

        AbstractAlgorithm.__init__(self, map_state)

        self.planner_name = "grid_nav_star"

        # GridNav constants.
        self.NEIGHBOUR_DIRECTIONS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1,), (1, -1)]
        self.BIG_COST = 500
        self.STRAIGHT_COST = 1.0
        self.DIAGONAL_COST = 1.4142136

        # Expressed in cells, move 0.15m in real world units.
        self.MAX_VELOCITY = 0.15 / self.map_state.cell_size

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

    @staticmethod
    def generate_dict_key(s):
        return "(" + str(s.x) + ", " + str(s.y) + ")"

    def setup_nodes(self):
        """
        Setup each node in the grid.

        :return: none
        """

        # +1 here as nodes sit a cell corners meaning that there are n + 1 nodes where n is the number of cells.
        for x in range(self.map_state.cells_square):
            column = []
            for y in range(self.map_state.cells_square):
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
                if ((x == 0) or (x == (self.map_state.cells_square - 1)) or
                        (y == 0) or (y == (self.map_state.cells_square - 1))):
                    self.nodes[x][y].occupied = True
                elif x == self.map_state.goal_x and y == self.map_state.goal_y:
                    self.nodes[x][y].occupied = False
                elif self.map_state.grid[x][y].state == 2:
                    self.nodes[x][y].occupied = True

    @staticmethod
    def get_manhattan(s, s1):
        return float(abs(s.y - s1.y) + abs(s.x - s1.x))

    @staticmethod
    def has_common_axis(s, sa):
        return s.x == sa.x or s.y == sa.y

    @staticmethod
    def is_diagonal_neighbour(s, sa):
        x = s.x - sa.x
        y = s.y - sa.y

        if abs(x) == abs(y):
            return True

        return False

    def cost(self, s, sa):
        xd = abs(s.x - sa.x)
        yd = abs(s.y - sa.y)
        scale = 1.0

        if xd + yd > 1:
            scale = self.DIAGONAL_COST

        return s.cost * scale

    def h(self, s, sa):
        """
        Heuristic based on diagonal cost between two points.
        :param s:
        :return:
        """
        dx = abs(s.x - sa.x)
        dy = abs(s.y - sa.y)
        return self.STRAIGHT_COST * (dx + dy) + (self.DIAGONAL_COST - 2 * self.STRAIGHT_COST) * min(dx, dy)

    def key(self, s):
        if s.g < s.rhs:
            s.key[0] = s.g + self.h(s, self.s_start)
            s.key[1] = s.g
        else:
            s.key[0] = s.rhs + self.h(s, self.s_start)
            s.key[1] = s.rhs

    def get_neighbours(self, s):
        neighbours = []

        for direction in self.NEIGHBOUR_DIRECTIONS:
            try:
                neighbours.append(self.nodes[s.x + direction[0]][s.y + direction[1]])
            except IndexError:
                pass

        return neighbours

    def update_state(self, s):
        self.vertex_accesses += 1
        s.evaluations += 1

        if s is not self.s_goal:
            for neighbour in self.get_neighbours(s):
                if not neighbour.occupied:
                    rhs = self.cost(s, neighbour) + neighbour.g

                    if rhs < s.rhs:
                        s.rhs = rhs

            if self.open_list.count(s) > 0:
                self.open_list.remove(s)

            if s.g != s.rhs:
                self.key(s)
                self.open_list.append(s)
                self.open_list.sort()

    def plan(self):
        """
        Compute the cost grid based on the map represented in the occupancy grid.

        :return: none
        """

        self.total_plan_steps += 1
        start_time = time.process_time()

        self.compute_shortest_path()

        x = int(self.robot.get_cell_x())
        y = int(self.robot.get_cell_y())

        # When there has been a change to the plan rebuild the path.
        self.path = []

        try:
            self.build_path(x, y)
        except RuntimeError as err:
            if str(err) == "maximum recursion depth exceeded in comparison":
                raise NoPathException("No path to Goal!")

        self.time_taken += round(time.process_time() - start_time, 3)

        #if self.do_smooth_path:
        #    self.smooth()

    def compute_shortest_path(self):
        while (self.open_list[0].key[0] < self.s_start.key[0] and self.open_list[0].key[1] < self.s_start.key[1]) or \
                self.s_start.rhs != self.s_start.g:
            k_old = [self.open_list[0].key[0], self.open_list[0].key[1]]
            s = self.open_list.pop(0)
            self.key(s)

            if k_old[0] < s.key[0] and k_old[1] < s.key[1]:
                self.open_list.append(s)
                self.open_list.sort()
            elif s.g > s.rhs:
                s.g = s.rhs

                for neighbour in self.get_neighbours(s):
                    if not neighbour.occupied:
                        self.update_state(neighbour)
            else:
                s.g = self.BIG_COST

                for neighbour in self.get_neighbours(s):
                    if not neighbour.occupied:
                        self.update_state(neighbour)

            self.key(self.s_start)

            if len(self.open_list) == 0:
                break

    def build_path(self, x, y):
        """
        Recursively build the path based on the robot's position x and y,
        until the final point is less than 0.5 away in both x and y.

        If the robot ends up surround by obstacles this could throw a
        recursive error.

        :param x: cell x
        :param y: cell y
        :return: none
        """

        i = x - 1
        x_part = 0
        y_part = 0
        low_x = 0
        low_y = 1
        obstacle_warning = False

        # Find the x and y parts of the gradient.
        while i <= x + 1:
            j = y - 1
            while j <= y + 1:
                if i < x:
                    x_part += self.nodes[i][j].rhs
                elif i > x:
                    x_part -= self.nodes[i][j].rhs

                if j < y:
                    y_part += self.nodes[i][j].rhs
                elif j > y:
                    y_part -= self.nodes[i][j].rhs

                # If the cost at one of the cells is BIG_COST then there is
                # an obstacle there and we should use a different approach
                # for computing the heading.
                if self.nodes[i][j].occupied:
                    obstacle_warning = True
                    break
                j += 1

            if obstacle_warning:
                break
            i += 1

        if not obstacle_warning:
            # Check for "knife edge" conditions. This is where the x or y component
            # is zero because the robot is on a "ridge". This code will nudge the
            # robot to one side of the ridge.

            # y knife edge.
            if y_part == 0.0 and self.nodes[x][y].rhs > self.nodes[x][y - 1].rhs:
                y_part = 2 * (self.nodes[x][y - 1].rhs - self.nodes[x][y].rhs)

            # x knife edge.
            if x_part == 0.0 and self.nodes[x][y].rhs > self.nodes[x - 1][y].rhs:
                x_part = 2 * (self.nodes[x - 1][y].rhs - self.nodes[x][y].rhs)
        else:
            # Use the lowest cost neighboring cell as the heading instead
            # of the gradient. This is in the case of a nearby obstacle.
            low_cost = self.nodes[x][y].rhs
            i = x - 1

            while i <= x + 1:
                j = y - 1
                while j <= y + 1:
                    if self.nodes[i][j].rhs < low_cost:
                        low_cost = self.nodes[i][j].rhs
                        low_x = i
                        low_y = j
                    j += 1
                i += 1

            # Use the direction towards the lowest cost cell as the heading.
            x_part = low_x - x
            y_part = low_y - y

        heading = math.atan2(y_part, x_part)

        next_x = round(x + (self.MAX_VELOCITY * math.cos(heading)), 2)
        next_y = round(y + (self.MAX_VELOCITY * math.sin(heading)), 2)

        self.path.append([next_x, next_y])

        x_difference = self.map_state.goal_x - next_x
        y_difference = self.map_state.goal_y - next_y

        if 0.5 >= x_difference >= -0.5 and 0.5 >= y_difference >= -0.5:
            return
        else:
            '''
            Ensure that the next more will be into another cell
            otherwise we'll end up in an infinite loop.

            Moves towards the neighbouring cell with the lowest cost
            '''
            if int(math.floor(next_x)) == x or int(math.floor(next_y)) == y:
                lowest = [x - 1, y + 1]  # First direction, top left cell from current.

                # All other surrounding cells.
                directions = [(0, 1), (1, 1), (-1, 0), (1, 0), (-1, -1), (0, -1), (1, -1)]

                for direction in directions:
                    current_x = x + direction[0]
                    current_y = y + direction[1]
                    if (self.nodes[current_x][current_y].rhs <
                            self.nodes[lowest[0]][lowest[1]].rhs):
                        lowest = [current_x, current_y]

                if self.nodes[lowest[0]][lowest[1]].rhs == self.BIG_COST:
                    # No path to goal.
                    raise NoPathException("No path to Goal!")

                next_x = lowest[0]
                next_y = lowest[1]

            self.build_path(int(math.floor(next_x)), int(math.floor(next_y)))

    def update_occupancy_grid(self, cells):
        """
        Updates the occupancy grid based on the cells that were updated on
        the map.

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

            #if cell.state == 0 and occupancy != self.EMPTY:
            #    self.map_state.grid[cell.x][cell.y].data.occupancy = self.EMPTY
            #elif cell.state == 1 and occupancy != self.EMPTY:
            #    self.map_state.grid[cell.x][cell.y].data.occupancy = self.EMPTY
            #elif cell.state == 2 and occupancy != self.FULL:
            #    self.map_state.grid[cell.x][cell.y].data.occupancy = self.FULL

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
                cost = self.nodes[x][y].rhs
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
                elif self.nodes[x][y].occupied:
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

# ----------------------------------------------------------------------#
# Inner Classes                                            		        #
# ----------------------------------------------------------------------#


class Node(object):
    """
    A linked list node object that contains the cells current x, y, its
    key in the form of a cost value, and the next nodes index.
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = 500
        self.rhs = 500
        self.occupied = False
        self.key = [self.g, self.rhs]
        self.evaluations = 0
        self.cost = 1.0

    def __eq__(self, other):
        if hasattr(other, "x") and hasattr(other, "y"):
            return self.x == other.x and self.y == other.y

    def __ne__(self, other):
        if hasattr(other, "x") and hasattr(other, "y"):
            return self.x != other.x and self.y != other.y

    def __gt__(self, other):
        if hasattr(other, "key"):
            return self.x > other.x or self.y > other.y

    def __lt(self, other):
        if hasattr(other, "key"):
            return self.x < other.x or self.y < other.y

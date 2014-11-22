'''
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
'''

import math

from .abstract_algorithm import AbstractAlgorithm
from exceptions import NoPathException

'''
A grid based navigation algorithm for mobile robots. Closest to the 
Shortest Path soultion presented by Dijkstra.
'''


class GridNav(AbstractAlgorithm):
    '''
    Initialises the GridNav algorithm with its default settings.
    '''

    def __init__(self, map):
        AbstractAlgorithm.__init__(self, map)

        # GridNav constants.
        self.BIG_COST = 500  # Highest cost for a cell.
        self.EMPTY = -1
        self.FULL = 1
        self.SCHEDULED = 0
        self.NOT_SCHEDULED = 1

        # Expressed in cells, move 0.15m in real world units.
        self.MAX_VELOCITY = 0.15 / self.map_state.cell_size

        # Keep track of computations.
        self.cell_count = 0

        # Linked list stuff.
        self.open_list = []
        self.free_head = 0
        self.open_head = self.EMPTY

        self.setup_open_list()
        self.setup_occupancy_grid()

    '''
    Setup the open list with default Nodes.
    '''

    def setup_open_list(self):
        for i in range(self.map_state.cells_square * self.map_state.cells_square):
            self.open_list.append(Node(0, 0, self.BIG_COST, i + 1))

        self.open_list[(self.map_state.cells_square * self.map_state.cells_square)
                       - 1].next_node = self.EMPTY

        self.free_head = 0
        self.open_head = self.EMPTY

    '''
    Setup each cell in the occpancy grid based on their content i.e.
    it contains the goal, an obstacle, or free space.
    '''

    def setup_occupancy_grid(self):
        for x in range(self.map_state.cells_square):
            for y in range(self.map_state.cells_square):
                data = GridNavData(self)

                '''
                If it's a cell on the boundary, set it up to be an obstacle,
                and scheduled. This will prevent expanding nodes out of bounds.
                '''
                if ((x == 0) or (x == (self.map_state.cells_square - 1)) or
                        (y == 0) or (y == (self.map_state.cells_square - 1))):
                    data.occupancy = self.FULL
                    data.scheduled = self.SCHEDULED
                elif x == self.map_state.goal_x and y == self.map_state.goal_y:
                    data.cost = 0
                elif self.map_state.grid[x][y].state == 2:  # Occupied
                    data.occupancy = self.FULL

                self.map_state.grid[x][y].data = data

    '''
    Get a free node of the free list.
    '''

    def get_node(self):
        if self.free_head == self.EMPTY:
            print("get_node: out of free nodes.\n")
            return

        i = self.free_head
        self.free_head = self.open_list[self.free_head].next_node

        return i

    '''
    Pop the lowest-cost node off the open list.
    '''

    def pop_node(self):
        if self.open_head == self.EMPTY:
            i = 0
            x = 0
            y = 0
        else:
            i = self.open_head

            # Get the values.
            x = self.open_list[i].x
            y = self.open_list[i].y

            # Reset the pointers.
            self.open_head = self.open_list[i].next_node
            self.open_list[i].next_node = self.free_head
            self.free_head = i

        return [x, y, i]

    '''
    Put a node on the open list. In sorted order by cost.
    '''

    def insert_node(self, x, y, key):
        if self.map_state.grid[x][y].data.scheduled == self.NOT_SCHEDULED:
            i = self.get_node()
            self.open_list[i].x = x
            self.open_list[i].y = y
            self.open_list[i].key = key

            if self.open_head == self.EMPTY:
                self.open_list[i].next_node = self.EMPTY
                self.open_head = i
            else:
                last = self.EMPTY
                current = self.open_head

                while (self.open_list[current].key < key) and (current != self.EMPTY):
                    last = current
                    current = self.open_list[current].next_node

                if current == self.open_head:
                    self.open_head = i
                    self.open_list[i].next_node = current
                else:
                    self.open_list[last].next_node = i
                    self.open_list[i].next_node = current

            self.map_state.grid[x][y].data.scheduled = self.SCHEDULED

    '''
    Calculate the cost of travelling from a cell to the goal.

    Returns False if no change, True otherwise.
    '''

    def cell_cost(self, x, y):
        self.cell_count += 1

        # Check if the current cell is an obstacle.
        if self.map_state.grid[x][y].data.occupancy == self.FULL:
            self.map_state.grid[x][y].data.cost = self.BIG_COST
            return False

        # Check if it's the goal.
        if (x == self.map_state.goal_x) and (y == self.map_state.goal_y):
            self.map_state.grid[x][y].data.cost = 0
            return False

        low_x = x
        low_y = y
        temp_cost = 0
        low_cost = self.map_state.grid[x][y].data.cost

        # Look at neighbors to find lowest potential cost - low_cost.
        i = x - 1
        while i <= x + 1:
            j = y - 1
            while j <= y + 1:
                if (i == x) or (j == y):
                    # If horizontal or vertical neighbor, cost is 1.0.
                    temp_cost = self.map_state.grid[i][j].data.cost + 1
                else:
                    # If diagonal neighbor, cost is SQRT2.
                    temp_cost = self.map_state.grid[i][j].data.cost + 1.4142136  # SQRT2

                if temp_cost < low_cost:  # If lowest cost so far, reset tracking variables.
                    low_cost = temp_cost
                    low_x = i
                    low_y = j
                j += 1
            i += 1

        # Reset cost if appropriate, and return True if changed, False otherwise.
        if self.map_state.grid[x][y].data.cost != low_cost:
            self.map_state.grid[x][y].data.cost = low_cost
            return True
        else:
            return False

    '''
    Pop a node of the open list and expand it by computing the costs of neighbors and
    pushing them on the open list.
    '''

    def expand(self, x, y):
        i = x - 1

        # Try to lower the costs of neighbors.
        while i <= x + 1:
            j = y - 1
            while j <= y + 1:
                if (i != x) or (j != y):
                    change = False

                    # If the neighbor is not an obstacle and it has not
                    # yet been expaned.
                    if ((self.map_state.grid[i][j].data.cost == self.BIG_COST) and
                            (self.map_state.grid[i][j].data.occupancy != self.FULL)):
                        change = self.cell_cost(i, j)

                    # If the new cost is lower, push the neighbor on the open list too.
                    if change:
                        self.insert_node(i, j, self.map_state.grid[i][j].data.cost)
                j += 1
            i += 1

    '''
    Compute the cost grid based on the map represented in the occupancy grid.
    '''

    def plan(self):
        result = self.EMPTY
        self.insert_node(self.map_state.goal_x, self.map_state.goal_y, 0.0)

        while self.open_head != self.EMPTY:
            result = self.pop_node()

            if result[2] != self.EMPTY:
                self.expand(result[0], result[1])
            else:
                break

        x = int(math.floor(self.robot.x + 0.5))
        y = int(math.floor(self.robot.y + 0.5))

        # When there has been a change to the plan rebuild the path.
        self.path = []

        try:
            self.build_path(x, y)
        except RuntimeError as err:
            if str(err) == "maximum recursion depth exceeded in comparison":
                raise NoPathException("No path to Goal!")


    '''
    Recursively build the path based on the robot's position x and y,
    until the final point is less than 0.5 away in both x and y.

    If the robot ends up surround by obstacles this could throw a
    recursive error.
    '''

    def build_path(self, x, y):
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
                    x_part += self.map_state.grid[i][j].data.cost
                elif i > x:
                    x_part -= self.map_state.grid[i][j].data.cost

                if j < y:
                    y_part += self.map_state.grid[i][j].data.cost
                elif j > y:
                    y_part -= self.map_state.grid[i][j].data.cost

                # If the cost at one of the cells is BIG_COST then there is
                # an obstacle there and we should use a different approach
                # for computing the heading.
                if self.map_state.grid[i][j].data.cost == self.BIG_COST:
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
            if y_part == 0.0 and self.map_state.grid[x][y].data.cost > self.map_state.grid[x][y - 1].data.cost:
                y_part = 2 * (self.map_state.grid[x][y - 1].data.cost - self.map_state.grid[x][y].data.cost)

            # x knife edge.
            if x_part == 0.0 and self.map_state.grid[x][y].data.cost > self.map_state.grid[x - 1][y].data.cost:
                x_part = 2 * (self.map_state.grid[x - 1][y].data.cost - self.map_state.grid[x][y].data.cost)
        else:
            # Use the lowest cost neighboring cell as the heading instead
            # of the gradient. This is in the case of a nearby obstacle.
            low_cost = self.map_state.grid[x][y].data.cost
            x_part = 0
            y_part = 1
            i = x - 1

            while i <= x + 1:
                j = y - 1

                while j <= y + 1:
                    if self.map_state.grid[i][j].data.cost < low_cost:
                        low_cost = self.map_state.grid[i][j].data.cost
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

        if x_difference <= 0.5 and y_difference <= 0.5:
            return
        else:
            self.build_path(int(math.floor(next_x)), int(math.floor(next_y)))


    '''
    Updates the occupancy grid based on the cells that were updated on
    the map.
    '''

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

            occpancy = self.map_state.grid[cell.x][cell.y].data.occupancy

            if cell.state == 0 and occpancy != self.EMPTY:
                self.map_state.grid[cell.x][cell.y].data.occupancy = self.EMPTY
            elif cell.state == 1 and occpancy != self.EMPTY:
                self.map_state.grid[cell.x][cell.y].data.occupancy = self.EMPTY
            elif cell.state == 2 and occpancy != self.FULL:
                self.map_state.grid[cell.x][cell.y].data.occupancy = self.FULL

    '''
    Prints the contents of the occupancy grid to the standard output.
    '''

    def print_occupancy_grid(self):
        y = self.map_state.cells_square - 1
        footer = ""
        rows = ""
        symbol = "     "
        start_spacing = ""
        end_spacing = ""
        cell = 0
        grid = self.map_state.grid

        while y >= 0:
            if y < 10:
                rows += str(y) + "  "
            elif y >= 10 and y < 100:
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
            elif cell > 10 and cell < 100:
                start_spacing = "   "
                end_spacing = "    "
            elif cell > 100 and cell < 1000:
                start_spacing = "  "
                end_spacing = "     "

            footer += start_spacing + str((self.map_state.cells_square - 1) - y) + end_spacing

            for x in range(self.map_state.cells_square):
                symbol = "     "

                if (x == int(math.floor(self.robot.x))
                    and y == int(math.floor(self.robot.y))):
                    symbol = "ROBOT"
                elif x == self.map_state.goal_x and y == self.map_state.goal_y:
                    symbol = "GOAL "
                elif grid[x][y].data.occupancy == self.FULL:
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

        print(rows)
        print(footer + '\n')

    '''
    Prints the contents of the cost grid to the standard output.
    '''

    def print_cost_grid(self):
        y = self.map_state.cells_square - 1
        footer = ""
        footer_padding = "         "
        rows = ""
        cost = ""
        start_spacing = ""
        end_spacing = ""
        cell = 0
        grid = self.map_state.grid

        while y >= 0:
            if y < 10:
                rows += str(y) + "  "
            elif y >= 10 and y < 100:
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
            elif cell > 10 and cell < 100:
                start_spacing = "   "
                end_spacing = "     "
            elif cell > 100 and cell < 1000:
                start_spacing = "  "
                end_spacing = "      "

            footer += start_spacing + str((self.map_state.cells_square - 1) - y) + end_spacing

            for x in range(self.map_state.cells_square):
                cost = grid[x][y].data.cost
                padding = ""

                if cost < 10:
                    padding = "  "
                elif cost < 100:
                    padding = " "

                rows += "[ %.2f" % cost + padding + " ]"

            y -= 1

            if y >= 0:
                rows += "\n\n"

        print(rows)
        print(footer + '\n')

    '''
    Prints the points in the robots path.
    '''

    def print_path(self):
        path = "path: "

        for i in range(len(self.path)):
            path += str(self.path[i])

            if i != len(self.path) - 1:
                path += "->"

        print(path + "\n")

# ----------------------------------------------------------------------#
# Inner Classes                                            		   #
# ----------------------------------------------------------------------#

'''
A linked list node object that contains the cells current x, y, its
key in the form of a cost value, and the next nodes index.
'''


class Node:
    def __init__(self, x, y, key, next_node):
        self.x = x
        self.y = y
        self.key = key
        self.next_node = next_node


'''
The data the GridNav algorithm uses for its calculations. It is 
stored in each map cells data attribute.
	
self.map.grid[x][y].data = some grid nav data.
'''


class GridNavData:
    def __init__(self, gridnav):
        self.occupancy = gridnav.EMPTY
        self.scheduled = gridnav.NOT_SCHEDULED
        self.cost = gridnav.BIG_COST

import math

from model.map import Map

'''

'''
class GridNav:
    '''

    '''
    def __init__(self, map):
        self.map = map

        self.BIG_COST = 500 # Highest cost for a cell.
        self.EMPTY = -1
        self.FULL = 1
        self.SCHEDULED = 0
        self.NOT_SCHEDULED = 1

        # Expressed in cells.
        self.MAX_VELOCITY = 0.5 

        # Keep track of computations.
        self.cell_count = 0

        # Linked list stuff.
        self.open_list = []
        self.free_head = 0
        self.open_head = self.EMPTY

        self.setup_open_list()
        self.setup_grid()

    '''
    Initialize the open list.
    '''
    def setup_open_list(self):
        i = 0
        while i < (self.map.cells_square * self.map.cells_square):
            self.open_list.append(Node(0, 0, self.BIG_COST, i + 1))
            i += 1

        self.open_list[(self.map.cells_square * self.map.cells_square) - 1].next_node = self.EMPTY
        self.free_head = 0
        self.open_head = self.EMPTY 

    '''
    
    '''
    def setup_grid(self):
        x = 0
        while x < self.map.cells_square:
            y = 0
            while y < self.map.cells_square:
                data = GridNavData(self)

                '''
                If it's a cell on the boundary, set it up to be an obstacle,
                and scheduled. This will prevent expanding nodes out of bounds.
                '''
                if ((x == 0) or (x == (self.map.cells_square - 1)) or
                    (y == 0) or (y == (self.map.cells_square - 1))):
                    data.occupancy = self.FULL
                    data.scheduled = self.SCHEDULED
                elif x == self.map.goal_x and y == self.map.goal_y:
                    data.cost = 0

                self.map.grid[x][y].data = data
                y += 1
            x += 1

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
        if self.map.grid[x][y].data.scheduled == self.NOT_SCHEDULED:
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
                    
            self.map.grid[x][y].data.scheduled = self.SCHEDULED

    '''
    Calculate the cost of travelling from a cell to the goal.

    Returns False if no change, True otherwise.
    '''
    def cell_cost(self, x, y):
        self.cell_count += 1

        # Check if the current cell is an obstacle.
        if self.map.grid[x][y].data.occupancy == self.FULL: 
            self.map.grid[x][y].data.cost = self.BIG_COST
            return False

        # Check if it's the goal.
        if (x == self.map.goal_x) and (y == self.map.goal_y):
            self.map.grid[x][y].data.cost = 0
            return False

        low_x = x
        low_y = y
        temp_cost = 0
        low_cost = self.map.grid[x][y].data.cost

        # Look at neighbors to find lowest potential cost - low_cost.
        i = x - 1
        while i <= x + 1:
            j = y - 1
            while j <= y + 1:
                if (i == x) or (j == y):
                    # If horizontal or vertical neighbor, cost is 1.0.
                    temp_cost = self.map.grid[i][j].data.cost + 1
                else:
                    # If diagonal neighbor, coist is SQRT2.
                    temp_cost = self.map.grid[i][j].data.cost + 1.4142136 # SQRT2

                if temp_cost < low_cost: # If lowest cost so far, reset tracking variables.
                    low_cost = temp_cost
                    low_x = i
                    low_y = j
                j += 1
            i += 1

        # Reset cost if appropriate, and return True if changed, False otherwise.
        if self.map.grid[x][y].data.cost != low_cost:
            self.map.grid[x][y].data.cost = low_cost
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
                    if ((self.map.grid[i][j].data.cost == self.BIG_COST) and
                        (self.map.grid[i][j].data.occupancy != 1)):
                        change = self.cell_cost(i, j)

                    # If the new cost is lower, push the neighbor on the open list too.
                    if change:
                        self.insert_node(i, j, self.map.grid[i][j].data.cost)
                j += 1
            i += 1

    '''
    Compute the cost grid based on the map represented in the occupancy grid.
    '''
    def replan(self):
        result = self.EMPTY
        self.insert_node(self.map.goal_x, self.map.goal_y, 0.0)

        while self.open_head != self.EMPTY: 
            result = self.pop_node()

            if result[2] != self.EMPTY:
                self.expand(result[0], result[1])
            else:
                break

    '''
    Look at the cost grid to decide which way to go.

    Returns the next x, y coordinate to travel too.

    East (+X) is 0, north (+Y) is PI / 2.
    '''
    def check_plan(self):
        x = int((self.map.robot.x / self.map.cell_size) + 0.5)
        y = int((self.map.robot.y / self.map.cell_size) + 0.5)
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
                    x_part += self.map.grid[i][j].data.cost
                elif i > x:
                    x_part -= self.map.grid[i][j].data.cost
                    
                if j < y:
                    y_part += self.map.grid[i][j].data.cost
                elif j > y:
                    y_part -= self.map.grid[i][j].data.cost

                # If the cost at one of the cells is BIG_COST then there is
                # an obstacle there and we should use a different approach
                # for computing the heading.
                if self.map.grid[x][y].data.cost == self.BIG_COST:
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
            if y_part == 0.0 and self.map.grid[x][y].data.cost > self.map.grid[x][y - 1].data.cost:
                y_part = 2 * (self.map.grid[x][y - 1].data.cost - self.map.grid[x][y].data.cost)

            # x knife edge.
            if x_part == 0.0 and self.map.grid[x][y].data.cost > self.map.grid[x - 1][y].data.cost:
                              x_part = 2 * (self.map.grid[x - 1][y].data.cost - self.map.grid[x][y].data.cost)
        else:
            # Use the lowest cost neighboring cell as the heading instead
            # of the gradient. This is in the case of a nearby obstacle.
            low_cost = self.map.grid[x][y].data.cost
            x_part = 0
            y_part = 1
            i = x - 1

            while i <= x + 1:
                j = y - 1

                while j <= y + 1:
                    if self.map.grid[i][j].data.cost < low_cost:
                        low_cost = self.map.grid[i][j].data.cost
                        low_x = i
                        low_y = j
                    j += 1
                i += 1

            # Use the direction towards the lowest cost cell as the heading.
            x_part = low_x - x
            y_part = low_y - y

        heading = math.atan2(y_part, x_part)

        new_x = (self.map.robot.x / self.map.cell_size) + (self.MAX_VELOCITY * math.cos(heading))
        new_y = (self.map.robot.y / self.map.cell_size) + (self.MAX_VELOCITY * math.sin(heading))
        new_point = [new_x, new_y]

        return new_point

    '''

    '''
    def update_occupancy(self, cells):
        for cell in cells:
            occpancy = self.map.grid[cell.x][cell.y].data.occupancy
            
            if cell.state == 0 and occpancy != self.EMPTY:
                self.map.grid[cell.x][cell.y].data.occupancy = self.EMPTY
            elif cell.state == 1 and occpancy != self.EMPTY:
                self.map.grid[cell.x][cell.y].data.occupancy = self.EMPTY
            elif cell.state == 2 and occpancy != self.FULL:
                self.map.grid[cell.x][cell.y].data.occupancy = self.FULL

    def print_occupancy_grid(self):
        y = 0
        header = ""
        rows = ""
        symbol = ""
        grid = self.map.grid

        while y < self.map.cells_square:
            x = 0
            header += "    " + str(y)
            rows += str(y) + " "
            
            while x < self.map.cells_square:
                if grid[x][y].data.occupancy == self.EMPTY:
                    symbol = "EMPTY"
                elif grid[x][y].data.occupancy == self.FULL:
                    symbol = "FULL "
                rows += "[ " + symbol + " ]"

                x += 1

            y += 1

            if y < self.map.cells_square:
                rows += "\n\n"

        print(header)
        print(rows)

    def print_cost_grid(self):
        y = 0
        header = ""
        rows = ""
        cost = ""
        grid = self.map.grid

        while y < self.map.cells_square:
            x = 0
            header += "    " + str(y)
            rows += str(y) + " "
            
            while x < self.map.cells_square:
                cost = grid[x][y].data.cost
                rows += "[ %f" % cost + " ]"

                x += 1

            y += 1

            if y < self.map.cells_square:
                rows += "\n\n"

        print(header)
        print(rows)

#----------------------------------------------------------------------#
#   Inner Classes                                            		   #
#----------------------------------------------------------------------#
        
'''

'''
class Node:
    def __init__(self, x, y, key, next_node):
        self.x = x
        self.y = y
        self.key = key
        self.next_node = next_node

'''

'''
class GridNavData:
    def __init__(self, gridnav):
        self.occupancy = gridnav.EMPTY 
        self.scheduled = gridnav.NOT_SCHEDULED 
        self.cost = gridnav.BIG_COST

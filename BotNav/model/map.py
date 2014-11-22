import math
import copy

from .cell import Cell

'''
A Map containing a 2D grid of cells of a specified size. The size of the
grid is in meters, the number of cells in the grid is calculated using
the grids size and the size of individual cells.

The Map also contains the robot operatin on it and the location of the
goal. Path planners can use the data in each cell of the grid for 
planning the motion of the robot.
'''


class Map:
    '''
    Initialises the Map based on the parameters provided.
    '''

    def __init__(self, robot, grid_size, cell_size):
        self.robot = robot
        self.grid_size = grid_size  # Size in meters down either side.
        self.cell_size = cell_size

        # Contains a list of columns, treated like a 2D array.
        self.grid = []

        # Number of cells down either side of the grid.
        self.cells_square = int(round(self.grid_size / self.cell_size, 0))

        self.goal_x = 0
        self.goal_y = 0

        self.populate_grid()

    '''
    Populates the grid with unknown cells.
    '''

    def populate_grid(self):
        '''
        O(n^2) algorithm to run over the grid. Consider using
        recursion instead.

        Same as running over a 2D array in C using two for loops.
        '''
        for x in range(self.cells_square):
            column = []

            for y in range(self.cells_square):
                column.append(Cell(x, y, "", 0))  # Add an unknown cell.

            self.grid.append(column)

    '''
    Returns the most significant cells involved in a ping, it is not
    100% accurate in cases where the ping cuts the corner of a cell but
    it does return the most important cells.

    Returns a copy of the cells in the pings area.

    Returns -1 if no cells are affected.

    Distance is specified in cells.

    TODO: Take into account that different range finders will have
    different fields of view!
    '''

    def ping_to_cells(self, distance):
        cells = []
        did_occupy = False

        last_x = -1
        last_y = -1

        while distance > 0:
            cell_x = math.floor(self.robot.x + (distance *
                                                math.cos(self.robot.heading)))

            cell_y = math.floor(self.robot.y + (distance *
                                                math.sin(self.robot.heading)))

            if cell_x >= self.cells_square or cell_y >= self.cells_square:
                distance -= self.cell_size
                continue

            cell = copy.deepcopy(self.grid[cell_x][cell_y])

            if cell != -1:
                if (cell.x == math.floor(self.robot.x) and
                            cell.y == math.floor(self.robot.y)):
                    break

                if cell.x != last_x or cell.y != last_y:
                    if not did_occupy:  # Occupy the first cell to be inbounds.
                        cell.state = 2  # Occupied.
                        did_occupy = True
                    else:
                        cell.state = 1  # Free.

                    cells.append(cell)
                    last_x = cell.x
                    last_y = cell.y

            distance -= self.cell_size

        return cells

    '''
    Returns the cell at the x, y coordinate in meters.

    Returns -1 if out of bounds.
    '''

    def point_to_cell(self, x, y):
        cell_x = int(round(max(0, min(x / self.cell_size,
                                      self.cells_square - 1)), 0))

        cell_y = int(round(max(0, min(y / self.cell_size,
                                      self.cells_square - 1)), 0))

        if cell_x > self.cells_square or cell_y > self.cells_square:
            return -1  # Cell out of bounds.

        return self.grid[cell_x][cell_y]

    '''
    Updates the map based on the new cell data provided.

    Returns the affected cells if any.
    '''

    def update_map(self, cells):
        updated_cells = []

        if len(cells) > 0:
            for i in range(len(cells)):
                cell = cells[i]

                if self.grid[cell.x][cell.y].state != cell.state:
                    self.grid[cell.x][cell.y].state = cell.state
                    updated_cells.append(self.grid[cell.x][cell.y])

        return updated_cells

    '''
    Prints a textual representation of the map.
    '''

    def print_map(self):
        '''
        O(n^2) algorithm to run over the grid. Consider using
        recursion instead.

        Same as running over a 2D array in C using two for loops.
        '''
        y = 0
        header = ""
        rows = ""
        symbol = ""
        foundRobot = False
        robot_position = self.point_to_cell(self.robot.x, self.robot.y)

        if robot_position == -1:
            foundRobot = True

        for y in self.cells_square:
            x = 0
            header += "    " + str(y)
            rows += str(y) + " "

            for x in range(self.cells_square):
                if not foundRobot and robot_position.x == x and robot_position.y == y:
                    rows += "[ R ]"
                    foundRobot = True
                else:
                    if self.grid[x][y].state == 0:
                        symbol = "#"
                    elif self.grid[x][y].state == 1:
                        symbol = " "
                    else:
                        symbol = "0"
                    rows += "[ " + symbol + " ]"

            if y < self.cells_square:
                rows += "\n\n"

        print(header)
        print(rows)


'''
my_robot = Robot(0)
my_robot.x = 0.60
my_robot.y = 0.90
my_robot.heading = 1.57

my_map = Map(my_robot, 3.0, 0.30)

cells = my_map.ping_to_cells(0.60)
my_map.update_map(cells)

my_map.print_map()
'''

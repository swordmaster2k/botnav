import math

from cell import Cell
from robot import Robot

'''

'''
class Map:
    
    '''

    '''
    def __init__(self, robot, width, height, cell_size):
        self.robot = robot
        self.width = width
        self.height = height
        self.cell_size = cell_size

        '''
        Contains a list of columns, which act like a 2D array
        '''
        self.grid = [] 

        self.cells_wide = int(round(self.width / self.cell_size, 0))
        self.cells_high = int(round(self.height / self.cell_size, 0))

        self.populate_grid()

    '''
    Populates the grid with blank cells.
    '''
    def populate_grid(self):
        '''
        O(n)^2 algorithm to run over the grid. Consider using
        recursion instead.

        Same as running over a 2D array in C using two for loops.
        '''
        x = 0
        while x < self.cells_wide:
            column = []
            y = 0
            while y < self.cells_high:
                column.append(Cell(x, y, "#"))
                y += 1

            self.grid.append(column)
            x += 1

    '''
    Returns the most significant cells involved in a ping, it is not 100% accurate in cases where the ping
    cuts the corner of a cell but it does return the most important cells.

    Returns a list of the cells in the pings area.

    Returns -1 if no cells are affected.

    TODO: Take into account that different range finders will have different fields of view!
    '''
    def ping_to_cells(self, distance):
        cells = []
        did_occupy = False

        last_x = -1
        last_y = -1

        while distance > 0:
            ping_x = round(self.robot.x + (distance * math.cos(self.robot.heading)), 2) # Our heading at that moment.
            ping_y = round(self.robot.y + (distance * math.sin(self.robot.heading)), 2)

            if ping_x > self.width or ping_y > self.height:
                distance -= self.cell_size
                continue

            cell = self.point_to_cell(ping_x, ping_y)

            print("cell.x: " + str(cell.x))
            print("cell.y: " + str(cell.y))
            
            if cell != -1:
                if cell.x != last_x or cell.y != last_y:
                    if not did_occupy: # Occupy the first cell to be inbounds.
                        cell.data = "0"
                        did_occupy = True
                    else:
                        cell.data = " "
                
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
        cell_x = int(round(max(0, min(x / self.cell_size, self.cells_wide - 1)), 0))
        cell_y = int(round(max(0, min(y / self.cell_size, self.cells_high - 1)), 0))
        
        if cell_x > self.cells_wide or cell_y > self.cells_high:
            return -1 # Cell out of bounds.

        return self.grid[cell_x][cell_y]

    '''
    Updates the map based on the new cell data provided. Currently very basic.

    Returns True if a changed occured, otherwise false.
    '''
    def update_map(self, cells):
        did_update = False
        
        if len(cells) > 0:
            i = 0
            while i < len(cells):
                cell = cells[i]
                if self.grid[cell.x][cell.y].data != cell.data:
                    self.grid[cell.x][cell.y].data = cell.data
                    
                    if not did_update:
                        did_update = True
                        
                self.grid[cell.x][cell.y].data = cell.data
                i += 1

        return did_update

    '''
    Prints a textual representation of the map. Shifts the grid so that 0,0 appears in the
    bottom left.
    '''
    def print_map(self):
        '''
        O(n)^2 algorithm to run over the grid. Consider using
        recursion instead.

        Same as running over a 2D array in C using two for loops.
        '''
        y = 0
        header = ""
        rows = ""
        foundRobot = False
        robot_position = self.point_to_cell(self.robot.x, self.robot.y)

        if robot_position == -1:
            foundRobot = True

        while y < self.cells_high:
            x = 0
            header += "    " + str(y)
            rows += str(y) + " "
            
            while x < self.cells_wide:
                if not foundRobot:
                    if robot_position.x == x and robot_position.y == y:
                        rows += "[ R ]"
                        foundRobot = True
                    else:
                        rows += "[ " + str(self.grid[x][y].data) + " ]"
                else:
                    rows += "[ " + str(self.grid[x][y].data) + " ]"

                x += 1

            y += 1

            if y < self.cells_high:
                rows += "\n\n"

        print(header)
        print(rows)

my_robot = Robot(0)
my_robot.x = 1.00
my_robot.y = 1.00
my_robot.heading = 3.14

my_map = Map(my_robot, 2.3, 2.3, 0.23)

cells = my_map.ping_to_cells(0.46)
my_map.update_map(cells)

my_map.print_map()

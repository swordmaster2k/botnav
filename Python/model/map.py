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
        self.grid = []

        self.populate_grid()

    def populate_grid(self):
        cells_wide = self.width // self.cell_size
        cells_high = self.height // self.cell_size

        '''
        O(n)^2 algorithm to run over the grid. Consider using
        recursion instead.

        Same as running over a 2D array in C using two for loops.
        '''
        i = 0
        while i < cells_wide:
            column = []
            j = 0
            while j < cells_high:
                column.append(j)
                j += 1

            self.grid.append(column)
            i += 1

my_map = Map(10, 10, 10, 0.2)
print(my_map.grid[0][9])




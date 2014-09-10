'''

'''
class Cell:
    '''

    '''
    def __init__(self, x, y, data, state):
        self.x = x
        self.y = y
        self.data = data
        self.state = 0 # 0 = Unknown, 1 = Empty, 2 = Occupied

    def to_string(self):
        print("x: " + str(self.x)  + ", y: " + str(self.y) + ", data: " + str(self.data) + ", state: " + str(self.state))
    

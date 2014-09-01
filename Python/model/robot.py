import math

'''

'''
class Robot:
    '''

    '''
    def __init__(self, connection):
        # Data connection to robot.
        self.connection = connection
        
        # Odometry.
        self.x = 0
        self.y = 0
        self.heading = 1.57

        # List of visited points.
        self.path = []
        self.path.append([self.x, self.y])

        # Physical dimensions in meters.
        self.width = 0.18
        self.lenght = 0.23

        # State string.
        self.state = "Halted"

    '''
    
    '''
    def go_forward(self):
        self.connection.send("w\n")

        self.state = "Moving Forward"

    '''

    '''
    def go_backward(self):
        self.connection.send("s\n")

        self.state = "Moving Backward"

    '''
    
    ''' 
    def rotate_left(self):
        self.connection.send("a\n")

        self.state = "Rotating Left"

    '''

    '''
    def rotate_right(self):
        self.connection.send("d\n")

        self.state = "Rotating Right"

    '''

    '''
    def halt(self):
        self.connection.send("q\n")

        self.state = "Halted"

    '''

    '''
    def scan(self):
        self.connection.send("e\n")

        self.state = "Scanning"

    '''

    '''
    def reset(self):
        self.connection.send("z\n")

        self.state = "Resetting"

    '''

    '''
    def rotate_to(self, heading):
        self.connection.send("r\n");

        self.state = "Rotating"
        
        '''
        angle = heading - self.heading

        if angle < -math.pi:
            angle += math.pi * 2
        elif angle > math.pi:
            angle -= math.pi * 2

        left_buffer = heading + 0.09
        right_buffer = heading - 0.09

        if left_buffer > math.pi * 2:
            left_buffer -= math.pi * 2

        if right_buffer < -math.pi * 2:
            right_buffer += math.pi * 2

        if angle < 0:
            self.rotate_left()
        elif angle > 0:
            self.rotate_right()

        print(right_buffer)
        print(left_buffer)

        while not (self.heading >= right_buffer and self.heading <= left_buffer):
            True # Busy wait.

        self.halt()
        '''

    '''
    Ideally this should be event or listener based, currently it just returns a boolean value
    to the caller indicating if there has been a change.
    '''
    def update_odometry(self, update):
        changed = False

        if self.x != update.x:
            self.x = update.x
            changed = True

        if self.y != update.y:
            self.y = update.y
            changed = True

        if self.heading != update.heading:
            self.heading = update.heading
            changed = True

        if changed:
            self.path.append([self.x, self.y])

        return changed

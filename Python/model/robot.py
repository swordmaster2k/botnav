import math
import time

'''

'''
class Robot:
    '''

    '''
    def __init__(self, connection):
        # Data connection to robot.
        self.connection = connection
        
        # Odometry, x and y are cell based.
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
        self.state = ""
        
        # Size of the cells we are operating in.
        self.cell_size = 0.3

    '''
    
    '''
    def go_forward(self):
        self.connection.send("w\n")

    '''

    '''
    def go_backward(self):
        self.connection.send("s\n")

    '''
    
    ''' 
    def rotate_left(self):
        self.connection.send("a\n")

    '''

    '''
    def rotate_right(self):
        self.connection.send("d\n")

    '''

    '''
    def halt(self):
        self.connection.send("q\n")

    '''

    '''
    def scan(self):
        self.connection.send("e\n")

    '''

    '''
    def ping(self):
        self.connection.send("p\n")

    '''

    '''
    def reset(self):
        self.connection.send("z\n")

    '''

    '''
    def rotate_to(self, heading):
        if (not (heading >= 0.0 and heading <= 6.28)):
            print("heading not within bounds: " + str(heading))
            return -1
        elif (self.heading == heading):
            print("already at heading: " + str(heading))
            return heading

        print("rotate_to: " + str(heading))

        self.connection.send("r" + str(round(heading, 2)) + "\n")

        return heading

    def travel_distance(self, distance):
        print("travel_distance: " + str(round(distance * self.cell_size, 2)))
        
        # Distance is cell based so send as meters.
        self.connection.send("t" + str(round(distance * self.cell_size, 2)) + "\n")

    '''
    
    '''
    def face(self, x, y):
        dx = x - self.x
        dy = y - self.y

        alpha = math.atan2(dy, dx)
        beta = alpha - self.heading

        if beta < 0:
            beta += 6.28
        elif beta >= 6.28:
            beta -= 6.28

        heading = self.heading + beta

        if heading < 0:
            heading += 6.28
        elif heading >= 6.28:
            heading -= 6.28

        heading = self.rotate_to(round(heading, 2))

        return heading

    def go_to(self, x, y):
        heading = self.face(x, y)

        while self.state != "Halted":
            continue

        #if did_break:
        #    return -1 # Something went wrong

        distance = math.sqrt((x - self.x) ** 2 + (y - self.y) ** 2)

        self.travel_distance(round(distance, 2))

    def change_odometry(self, x, y, heading):
        x = round(x * self.cell_size, 2)
        y = round(y * self.cell_size, 2)
        self.connection.send("c," + str(x) + "," + str(y) + "," + str(heading) + "\n")

    '''
    Ideally this should be event or listener based, currently it just returns a boolean value
    to the caller indicating if there has been a change.
    
    The update stores the x and y coordinates in meters so they must be converted.
    '''
    def update_odometry(self, update):
        changed = False

        if self.x != (update.x / self.cell_size):
            self.x = (update.x / self.cell_size)
            changed = True

        if self.y != (update.y / self.cell_size):
            self.y = (update.y / self.cell_size)
            changed = True

        if self.heading != update.heading:
            self.heading = update.heading
            changed = True

            if self.heading < 0:
                self.heading += 6.28
            elif self.heading >= 6.28:
                self.heading -= 6.28

        if changed:
            self.path.append([self.x, self.y])

        return changed

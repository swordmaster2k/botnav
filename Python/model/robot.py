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
        print("travel_distance: " + str(distance))

        self.connection.send("t" + str(distance) + "\n")

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

        if heading == -1:
            return heading # Error.

        left_buffer = heading + 0.01
        right_buffer = heading - 0.01

        if left_buffer > 6.28:
            left_buffer -= 6.28

        if right_buffer < -6.28:
            right_buffer += 6.28

        left_buffer = round(left_buffer, 2)
        right_buffer = round(right_buffer, 2)

        print("left_buffer: " + str(left_buffer))
        print("right_buffer: " + str(right_buffer))

        did_break = False
        start = time.time()
        last_heading = self.heading

        while not (self.heading >= right_buffer and self.heading <= left_buffer):
            '''if 5.0 >= time.time() - start: # Have 3 seconds elapsed?
                if last_heading == self.heading: # Have we rotated in that time?
                    print("appear to be stuck!")
                    did_break = True # If not then we are stuck get out of infinite loop!
                    break
                else:
                    lastheading = self.heading'''

        if did_break:
            return -1 # Something went wrong

        distance = math.sqrt((x - self.x) ** 2 + (y - self.y) ** 2)

        self.travel_distance(round(distance, 2)) 

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

            if self.heading < 0:
                self.heading += 6.28
            elif self.heading >= 6.28:
                self.heading -= 6.28

        if changed:
            self.path.append([self.x, self.y])

        return changed

import math
import random

from .robot import Robot
from events import ScanResult

'''
A generic Robot class which (may) represent(s) a hardware robot that 
implements the communications interface defined by the robonav tool.
It is possible to use this class for simulations where no hardware
robot exists.

It can communicate with a hardware robot using Bluetooth, WiFi, 
Ethernet, Serial, InfraRed, etc. using the abstracted connection
approach.

This class keeps track of the robots position, orientation, the path it
has traversed, physical dimensions, state, and the cell resolution
it is operating in.

It does not matter if the robot is a wheeled, tracked, bipod, etc. as 
long as the hardware conforms to the generic interface required by
the robonav tool. 
'''


class SimulatedRobot(Robot):
    '''
    Initialises the robot using the connection specified.
    '''

    def __init__(self, connection):
        Robot.__init__(self, connection)

    '''
    Instructs the robot to go forward.
    '''

    def go_forward(self):
        return

    '''
    Instructs the robot to go backward.
    '''

    def go_backward(self):
        return

    '''
    Instructs the robot to rotate left.
    '''

    def rotate_left(self):
        return

    '''
    Instructs the robot to rotate right.
    '''

    def rotate_right(self):
        return

    '''
    Instructs the robot to halt.
    '''

    def halt(self):
        return

    '''
    Instructs the robot to begin a scan.
    '''

    def scan(self):
        return

    '''
    Instructs the robot to ping.
    '''

    def ping(self):
        # Send a dummy scan result with a distance of 0.
        # Will have no effect.
        for listener in self.connection.listeners:
            listener.handle_event(ScanResult([0]))

    '''
    Instructs the robot to update its odometry with the new parameters.
    '''

    def change_odometry(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading
        self.trail.append([self.get_cell_x(), self.get_cell_y()])

    '''
    Instructs the robot to go a point.
    '''

    def go_to(self, x, y):
        self.x = (x + random.uniform(-0.2, 0.2)) * self.cell_size  # Introduce a little uncertainty.
        self.y = (y + random.uniform(-0.2, 0.2)) * self.cell_size

        self.trail.append([self.get_cell_x(), self.get_cell_y()])

        self.state = "Travelled"


'''

'''
import os
import math
import time
import threading
import _thread as thread
import RPi.GPIO as io

from threading import Thread
from connection import IPConnection
from proxy import Proxy

DISTANCE_PER_STEP = 0.16    # Average distance in metres that Robosapien travels per STEP command.

TIME_PER_STEP = 3.0         # Average time taken per STEP command in seconds.

MESSAGE_RATE = 0.5          # Rate at which messages are to be sent in seconds.

# LIRC commands.
IR_SEND = "irsend"
SEND_ONCE = "SEND_ONCE"
REMOTE_NAME = "robosapien"
STOP = "STOP"
WALK_FORWARD = "WALK_FORWARD"
WALK_BACKWARD = "WALK_BACKWARD"
TURN_RIGHT = "TURN_RIGHT"
TURN_LEFT = "TURN_LEFT"
FORWARD_STEP = "FORWARD_STEP"

# All possible commands for the robot.
COMMANDS = ['w', 's', 'a', 'd', 'q', 'r', 'e', 'p', 't', 'c']

io.setmode(io.BCM)

'''
Update the heading via a separate thread for the MPU6050.

For travel() either going to have to depend on average time Robosapien
takes per step or the accelerometer to detect no movement.

update_odometry() likely to be called after the travel() command has
completed rather than via a thread. Introduce update_heading stub.
'''


class Robosapien(Thread):
    '''

    '''

    def __init__(self):
        # Create the communications thread.
        self.proxy = Proxy(IPConnection("10.42.0.1", 50001))

        self.x = 0.0
        self.y = 0.0
        self.heading = 1.57
        self.offset = 0.0  # Offset to record when the odometry has been forced.

        Thread.__init__(self)

    '''
    Processes a command received down the communications channel, if
    it is valid an action is taken.
    '''

    def process_command(self, command):
        state = ""

        if command[0] == 'w':
            self.invoke_lirc(WALK_FORWARD)
            state = "Going Forward\n"
        elif command[0] == 's':
            self.invoke_lirc(WALK_BACKWARD)
            state = "Going Backward\n"
        elif command[0] == 'a':
            self.invoke_lirc(TURN_LEFT)
            state = "Turning Left\n"
        elif command[0] == 'd':
            self.invoke_lirc(TURN_RIGHT)
            state = "Turning Right\n"
        elif command[0] == 'q':
            self.invoke_lirc(STOP)
            state = "Halted\n"
        elif command[0] == 'r':
            self.face(command)
        elif command[0] == 'e':
            self.scan()
        elif command[0] == 't':
            self.travel(command)
        elif command[0] == 'p':
            self.ping()
        elif command[0] == 'c':
            self.change_odometry(command)

        if state != "":
            self.proxy.send(state)  # Inform host of state change.

    '''

    '''

    @staticmethod
    def invoke_lirc(command):
        os.popen('/usr/bin/' + IR_SEND + " " + SEND_ONCE + " " + REMOTE_NAME + " " + command, 'w')

    '''
    Updates just the robot's heading, called from a separate thread.
    '''

    def update_heading(self, heading):
        self.heading = heading

    '''
    Updates the robot's odometry (x, y).
    '''

    def update_odometry(self, steps):
        # Do this based on the number of steps we have taken, and
        # the robots current heading.
        self.x += (steps / DISTANCE_PER_STEP) * math.cos(self.heading)
        self.y += (steps / DISTANCE_PER_STEP) * math.sin(self.heading)

    '''
    Sends the robot's current odometry to the host.
    '''

    def send_odometry(self):
        self.proxy.send("o,%.2f" % self.x + ",%.2f" % self.y + ",%.2f" % self.heading + "\n")

    '''
    Changes the robot's odometry by force.
    '''

    def change_odometry(self, x, y, heading):
        self.x = x
        self.y = x
        self.offset = self.heading - heading
        self.heading = heading

    '''
    Rotates the robot to face a particular heading.
    '''

    def face(self, command):
        if len(command) == 5:
            command = command.strip("r\n")
            heading = float(command)

            if 0.0 <= heading <= 6.27:
                angle = heading - self.heading

                # Some wrap around.
                if angle < -3.14:
                    angle += 6.28
                elif angle > 3.14:
                    angle -= 6.28

                # Get within 0.5 degrees of target heading.
                left_buffer = angle + 0.01
                right_buffer = angle - 0.01

                if left_buffer > 6.28:
                    left_buffer -= 6.28

                if right_buffer < 6.28:
                    right_buffer += 6.28

                message_time = time.time()

                # Our context is based on a standard circle with the
                # angle increasing from right to left.
                if angle < 0:
                    self.invoke_lirc(TURN_RIGHT)
                if angle > 0:
                    self.invoke_lirc(TURN_LEFT)

                interrupted = False

                while self.heading < right_buffer or self.heading > left_buffer:
                    # Check for any commands that could cancel this.
                    with self.proxy.command_mutex:
                        if len(self.proxy.command_queue) > 0:
                            interrupted = True
                            break

                    if time.time() - message_time >= MESSAGE_RATE:
                        self.send_odometry()
                        message_time = time.time()

                self.invoke_lirc(STOP)
                self.proxy.send("Current Heading: %.2f" % self.heading + "\n")

                if interrupted:
                    print("interrupted during rotation!")
                    with self.proxy.command_mutex:
                        self.process_command(self.proxy.command_queue.pop())

    '''
    Will remain unimplemented.
    '''

    def scan(self):
        return

    '''
    Will remain unimplemented.
    '''

    def ping(self):
        return

    '''
    Drives the robot forward a certain distance.
    '''

    def travel(self, command):
        print(command)
        if len(command) >= 3:
            command = command.strip("t\n")
            distance = float(command)

            if distance > 0:
                message_time = time.time()

                interrupted = False

                if distance < DISTANCE_PER_STEP:
                    number_of_steps = 1
                else:
                    number_of_steps = distance / DISTANCE_PER_STEP

                steps_taken = 0

                self.invoke_lirc(FORWARD_STEP)
                step_time = time.time()

                # Will just have to wait for some predefined time
                # or check the accelerometer. Drift will happen!
                while True:
                    # Check for any commands that could cancel this.
                    with self.proxy.command_mutex:
                        if len(self.proxy.command_queue) > 0:
                            interrupted = True
                            break

                    # Either wait for a predefined time to elapse
                    # or probe the accelerometer for movement.
                    if time.time() - step_time >= TIME_PER_STEP:
                        steps_taken += 1

                        if steps_taken < number_of_steps:
                            self.invoke_lirc(FORWARD_STEP)
                            step_time = time.time()
                        else:
                            break

                    if time.time() - message_time >= MESSAGE_RATE:
                        self.update_odometry(steps_taken)
                        self.send_odometry()
                        message_time = time.time()

                # Compute new odometry based on the number steps that were taken.
                self.update_odometry(steps_taken)

                self.invoke_lirc(STOP)
                self.proxy.send("travelled: %.2f" % (steps_taken / DISTANCE_PER_STEP) + "\n")

                if interrupted:
                    print("interrupted during travel!")
                    with self.proxy.command_mutex:
                        self.process_command(self.proxy.command_queue.pop())

    '''

    '''

    def run(self):
        self.proxy.start()

        message_time = time.time()

        while True:
            try:
                command = ""

                # Check the command poll.
                with self.proxy.command_mutex:
                    if len(self.proxy.command_queue) > 0:
                        command = self.proxy.command_queue.pop()

                # If it exists then process it.
                if COMMANDS.count(command[0]) < 1:
                    self.process_command(command)

                if time.time() - message_time >= MESSAGE_RATE:
                    self.send_odometry()
                    message_time = time.time()
            except Exception as err:
                print(str(err))
                break

        self.proxy.socket.close()

        print("Cleaning up GPIO!")
        io.cleanup()

        print("Exiting Now!")

robosapien = Robosapien()
robosapien.start()

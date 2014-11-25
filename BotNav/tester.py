import sys
import select
import datetime
import threading

from pathlib import Path
from threading import Thread

from proxy import Proxy
from model.map import Map
from planner import Planner
from util import gnuplotter
from model.robot import Robot
from algorithm.gridnav import GridNav
from connection.bluetooth_connection import BluetoothConnection
from events import OdometryReport, ScanResult, StateEvent
from model.simulated_robot import SimulatedRobot



class Tester(threading.Thread):
    def __init__(self, map_file):
        #self.proxy = Proxy(BluetoothConnection("00:00:12:06:56:83", 0x1001))
        #self.robot = Robot(self.proxy)

        self.proxy = Proxy("DUMMY")  # Dummy connection.
        self.robot = SimulatedRobot(self.proxy)

        self.proxy.listeners.append(self)

        self.map_file = map_file

        # Will be assigned in open_map().
        self.cell_size = None
        self.grid_size = None
        self.map = None

        self.algorithm = None
        self.planner = None

        self.output_file = None
        self.gnuplot_file = None

        self.output_path = None

        Thread.__init__(self)

    def open_map(self):
        infile = Path(self.map_file).open()

        # Do not bother with any validation for now.
        self.grid_size = float(infile.readline())
        self.cell_size = float(infile.readline())

        # Addition of the boundary.
        self.grid_size += self.cell_size
        self.grid_size = int(self.grid_size / self.cell_size)

        self.robot.cell_size = self.cell_size
        self.map = Map(self.robot, self.grid_size * self.cell_size, self.cell_size)

        y = self.grid_size - 1

        while y >= 0:
            line = infile.readline()

            for x in range(self.grid_size):
                if line[x] == "#":
                    self.map.grid[x][y].state = 2
                elif line[x] == "R":
                    # Put the robot in the center of the cell.
                    self.robot.change_odometry(round(x + 0.5, 2),
                                               round(y + 0.5, 2), 1.57)

                    print("Waiting for odometry change...")

                    # Wait for odometry change to take affect.
                    while self.robot.x != round(x + 0.5, 2) and self.robot.y != round(y + 0.5, 2):
                        continue

                    print("Odometry change successful!")

                elif line[x] == "G":
                    self.map.goal_x = x
                    self.map.goal_y = y

            y -= 1

        infile.close()

    def setup_output(self):
        # Establish the path to the "output" directory.
        p = Path(str(Path(self.map_file).parents[0]) + "/output")

        # Check to see if the "output" directory actually exists.
        if not p.exists():
            # It does not exist so create it.
            p.mkdir()

        directory_path = str(Path(self.map_file).parents[0]) + "/output/" + str(Path(self.map_file).name) + \
            str(datetime.datetime.utcnow())

        # Create the directory for this run.
        p = Path(directory_path)

        if not p.exists():
            # It does not exist so create it.
            p.mkdir()

        self.output_path = directory_path

        # Create the general output file by tagging the ".output" extension to the existing file and a time stamp.
        file_string = directory_path + "/debug_info.output"
        p = Path(file_string)

        if p.exists():
            p.replace(file_string)
        else:
            p.touch()

        self.output_file = p.open(mode='w')

        # Do the same with the gnuplot output.
        file_string = directory_path + "/paths.gnuplot"
        p = Path(file_string)

        if p.exists():
            p.replace(file_string)
        else:
            p.touch()

        self.gnuplot_file = p.open(mode='w+')

    def handle_event(self, event):
        if isinstance(event, OdometryReport):
            did_change = self.robot.update_odometry(event)
        elif isinstance(event, StateEvent):
            self.robot.state = event.state

    def run(self):
        self.proxy.start()

        command = ""

        while command != "quit":
            if self.planner.finished:
                break

            # This method only work on Unix, try timers.
            i, o, e = select.select([sys.stdin], [], [], 1)

            if i:
                command = sys.stdin.readline().strip()

                if command == "begin":
                    if not self.planner.finished:
                        self.planner.start()
                elif command == "quit":
                    self.planner.finished = True

        # Close the debugging files.
        self.output_file.close()
        self.gnuplot_file.close()

        gnuplotter.generate_output(self.output_path, self.algorithm.total_plan_steps, self.grid_size, "png")

if __name__ == '__main__':
    if len(sys.argv) == 2:
        map_file = sys.argv[1]

        if not Path(map_file).exists():
            print(map_file + " does not exist.")
            exit(-1)

        tester = Tester(map_file)

        try:
            tester.open_map()
            tester.setup_output()
        except IOError as err:
            print(err)
            exit(-1)

        tester.algorithm = GridNav(tester.map)
        tester.planner = Planner(tester.map, tester.algorithm, tester.proxy, tester.output_file, tester.gnuplot_file)

        print("Type \"begin\" to start run...")

        tester.start()

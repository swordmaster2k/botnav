import sys
import time
import select
import _thread
import datetime
import threading

from pathlib import Path
from threading import Thread
from threading import Lock

from proxy import Proxy
from model.map import Map
from planner import Planner
from util import gnuplotter
from model.robot import Robot
from algorithm.gridnav import GridNav
from algorithm.dstarlite import DStarLite
from connection.bluetooth_connection import BluetoothConnection
from connection.ip_connection import IPConnection
from events import OdometryReport, ScanResult, StateEvent
from model.simulated_robot import SimulatedRobot

NUMBER_OF_RUNS = 10
GNU_PLOT_OUTPUT = "png"


class Tester(threading.Thread):
    def __init__(self, map_file):
        self.proxy = None
        self.robot = None

        self.map_file = map_file

        # Will be assigned in open_map().
        self.cell_size = None
        self.grid_size = None
        self.map = None

        self.algorithm = None
        self.planner = None

        self.mode = None

        self.output_file = None
        self.gnuplot_file = None
        self.occupancy_file = None

        self.output_path = None

        self.lock = None

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
                    self.robot.change_odometry(round(x, 2),
                                               round(y, 2), 1.57)

                    print("Waiting for odometry change...")

                    # Wait for odometry change to take affect.
                    while self.robot.x != round(x, 2) and self.robot.y != round(y, 2):
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

        # Do the same with the occupancy grid output.
        file_string = directory_path + "/occupancy.gnuplot"
        p = Path(file_string)

        if p.exists():
            p.replace(file_string)
        else:
            p.touch()

        self.occupancy_file = p.open(mode='w+')

    def handle_event(self, event):
        if isinstance(event, OdometryReport):
            did_change = self.robot.update_odometry(event)
        elif isinstance(event, StateEvent):
            self.robot.state = event.state

    def run(self):
        self.proxy.start()

        command = ""

        if NUMBER_OF_RUNS == 1:
            print("Type \"begin\" to start run...")

            while command != "quit":
                if self.planner.finished:
                    break

                # This method only works on POSIX.
                stdin, o, e = select.select([sys.stdin], [], [], 1)

                if stdin:
                    command = sys.stdin.readline().strip()

                    if command == "begin":
                        if not self.planner.finished:
                            self.planner.start()
                    elif command == "quit":
                        self.planner.finished = True
        elif self.mode == "simulated":
            self.planner.start()
        else:
            sys.stdout.write("Number of runs greater than 1 and not in simulated mode!")

        while not self.planner.finished:
            continue

        # Populate the occupancy file.
        for x in range(self.map.cells_square):
            for y in range(self.map.cells_square):
                if self.map.grid[x][y].state == 2:
                    # Its occupied write this point out.
                    self.occupancy_file.write(str(x) + "\t" + str(y) + "\n")

        # Close files.
        self.output_file.close()
        self.gnuplot_file.close()
        self.occupancy_file.close()

        gnuplotter.generate_output(self.output_path, self.algorithm.total_plan_steps, self.grid_size, GNU_PLOT_OUTPUT)

        self.lock.acquire()


def load_config(config_file):
    """

    :return:
    """

    p = Path(config_file)

    if not p.exists():
        raise RuntimeError("config.botnav file not found!")

    test = Tester("")
    mode = ""
    algorithm = ""
    connection = None
    parameter_1 = None  # First connection parameter.
    parameter_2 = None  # Second connection paramter.
    config_file = p.open()

    while True:
        result = read_config_line(config_file)

        if result is None:  # Comment.
            continue
        elif result == "eof":  # End of file.
            break

        token = result[0]
        value = result[1]

        if token == "map":
            test.map_file = value
        elif token == "planner":
            algorithm = value
        elif token == "mode":
            mode = value
            if mode == "physical":
                result = read_config_line(config_file)

                if result is None or result == "eof":
                    raise RuntimeError("invalid connection in config file")

                token = result[0]
                value = result[1]

                if token == "connection":
                    connection = value

                    result = read_config_line(config_file)

                    if result is None or result == "eof":
                        raise RuntimeError("invalid connection in config file")

                    parameter_1 = result[1]

                    result = read_config_line(config_file)

                    if result is None or result == "eof":
                        raise RuntimeError("invalid connection in config file")

                    parameter_2 = result[1]
                else:
                    raise RuntimeError("invalid connection in config file")

    if test.map_file == "" or mode == "" or algorithm == "":
        raise RuntimeError("config file is incomplete")

    # If everything was parsed successfully we can safely do this stuff.
    if mode == "simulated":
        test.proxy = Proxy("DUMMY")  # Dummy connection.
        test.robot = SimulatedRobot(test.proxy)
    elif mode == "physical":
        if connection == "bluetooth":
            test.proxy = Proxy(BluetoothConnection(parameter_1, int(parameter_2, base=16)))  # MAC, Port
        elif connection == "ip":
            test.proxy = Proxy(IPConnection(parameter_1, int(parameter_2, base=16)))  # IP, Port
        else:
            raise RuntimeError("unsupported connection in config file")

        test.robot = Robot(test.proxy)
    else:
        raise RuntimeError("unsupported mode in config file")

    test.mode = mode

    test.open_map()
    test.setup_output()

    if algorithm == "grid_nav":
        test.algorithm = GridNav(test.map)
    elif algorithm == "d_star_lite":
        test.algorithm = DStarLite(test.map)
        test.algorithm.setup(test.map_file)
    elif algorithm == "field_d_star":
        raise RuntimeError("unsupported planner in config file")  # Not ready yet.
    else:
        raise RuntimeError("unsupported planner in config file")

    test.proxy.listeners.append(test)
    test.planner = Planner(test.map, test.algorithm, test.proxy, test.output_file, test.gnuplot_file)

    return test


def read_config_line(config_file):
    line = config_file.readline()
    line = line.strip()

    if line.startswith('#') or line.startswith('\n'):  # It is a comment or blankline.
        return None
    elif line == '':  # EOF.
        return "eof"

    line = line.rstrip('\n')

    if not '=' in line:
        raise RuntimeError("missing '=' in config file")

    partition = line.partition('=')

    return partition[0].lower(), partition[2].lower()

if __name__ == '__main__':
    if len(sys.argv) == 2:
        try:
            for i in range(NUMBER_OF_RUNS):
                thread_lock = _thread.allocate_lock()
                tester = load_config(sys.argv[1])
                tester.lock = thread_lock
                tester.start()

                while not thread_lock.locked():
                    continue
        except RuntimeError as err:
            print(err)
            exit(-1)
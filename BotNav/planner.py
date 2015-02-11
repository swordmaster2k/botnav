import sys
import time
import threading

from threading import Thread

from events import ScanResult
from util import result_generator

import exceptions


class Planner(threading.Thread):
    """
    The Planner actually deals with moving the robot from A to B until it
    reaches the goal. It makes use of the map, robot, and a path planning
    algorithm to do this.

    It is implemented on its own thread.
    """

    def __init__(self, map, algorithm, proxy, output_file, gnuplot_file):
        """
        Initialises the planner with a map and algorithm. It gets the
        robot from the map it is passed.

        :param map: current state space of the environment
        :param algorithm: path planning algorithm to use during run()
        :param proxy: communications connection to the robot
        :param output_file: debugging information file
        :param gnuplot_file: plot file for paths
        :return: a new planner
        """

        self.map = map
        self.robot = self.map.robot
        self.algorithm = algorithm

        self.proxy = proxy
        self.proxy.listeners.append(self)

        # Make sure these are open before we write to them.
        self.output_file = output_file
        self.gnuplot_file = gnuplot_file

        self.finished = False
        self.last_scan = 0

        Thread.__init__(self)

    def handle_event(self, event):
        """
        Handles specific events received from the system.

        :param event: generated system event from robot
        :return: none
        """
        if isinstance(event, ScanResult):
            self.last_scan = event

    def write_state(self):
        """
        Writes the current state of the planning operation to the debug file.

        :return: none
        """

        # Print some information to stdout.
        self.algorithm.print_path(sys.stdout)
        self.algorithm.print_cost_grid(sys.stdout)
        self.algorithm.print_occupancy_grid(sys.stdout)
        sys.stdout.write(('-' * 120) + '\n\n')

        if self.output_file is not None:
            if not self.output_file.closed:
                self.algorithm.print_path(self.output_file)
                self.algorithm.print_cost_grid(self.output_file)
                self.algorithm.print_occupancy_grid(self.output_file)
                self.output_file.write(('-' * 120) + '\n\n')

    def write_debug_info(self):
        """
        Writes additional information to the debug file.

        :return: none
        """

        sys.stdout.write("cell x: %.2f" % self.robot.get_cell_x() + ", cell y: %.2f" % self.robot.get_cell_y() + "\n")
        sys.stdout.write(
            "x: %.2f" % (self.robot.x) +
            ", y: %.2f" % (self.robot.y) + "\n"
            "heading %.2f" % self.robot.heading)
        sys.stdout.write("\n\n" + ('-' * 120) + "\n\n")

        if self.output_file is not None:
            if not self.output_file.closed:
                self.output_file.write("cell x: %.2f" % self.robot.get_cell_x() + ", cell y: %.2f" % self.robot.get_cell_y() + "\n")
                self.output_file.write(
                    "x: %.2f" % (self.robot.x) +
                    ", y: %.2f" % (self.robot.y) + "\n" +
                    "heading %.2f" % self.robot.heading)
                self.output_file.write("\n\n" + ('-' * 120) + "\n\n")

    def run(self):
        """
        The actions of the Planner using any algorithm are:

        1 . Plan
        2 . Check sensors to find obstacles.
        3 . Check the map for discrepancies.
        3a. If the map has changed, recompute the plan.
        4 . Check plan and initiate movement along shortest path.
        5 . Go to 2.

        Until we reach the goal.

        :return: none
        """

        # Variable for holding all the paths that
        # are generated for later writing to gnuplot file.
        paths = []

        # Write the initial state.
        #self.write_state()

        start_time = time.process_time()

        try:
            '''
            Step 1: Plan.
            '''
            self.algorithm.plan()

            # Write the state after first planning step.
            self.write_state()

            # Append a copy of the path to our paths record.
            paths.append(self.algorithm.path[:])

            # Stick the starting position of the robot into
            # the first path.
            paths[0].insert(0, [self.robot.get_cell_x(), self.robot.get_cell_y()])

            # Calculate our initial distance from the goal.
            x_difference = self.map.goal_x - self.robot.get_cell_x()
            y_difference = self.map.goal_y - self.robot.get_cell_y()

            if x_difference < 0:
                x_difference = -x_difference

            if y_difference < 0:
                y_difference = -y_difference

            # Write some debugging info.
            self.write_debug_info()

            # While we are not within 0.7 cells of the goal in both x and y.
            while not (0.5 >= x_difference >= -0.5 and 0.5 >= y_difference >= -0.5):
                '''
                Step 2: Scan the immediate area for obstacles and free space.
                '''
                self.robot.ping()

                while self.last_scan == 0:
                    continue

                # Just take 1 reading for now.
                affected_cells = []#self.map.ping_to_cells(
                    #round(float(self.last_scan.readings[0]) / self.map.cell_size, 2))
                self.last_scan = 0

                '''
                Step 3: Update the map if necessary.
                '''
                if len(affected_cells) > 0:
                    updated_cells = self.map.update_map(affected_cells)
                    if len(updated_cells) > 0:
                        self.algorithm.update_occupancy_grid(updated_cells)

                        '''
                        Step 3a: Recompute the plan if necessary.
                        '''
                        self.algorithm.plan()

                        # Append a copy of the path to our paths record.
                        paths.append(self.algorithm.path[:])

                '''
                Step 4: Pop the next point from the current path.
                '''
                if len(self.algorithm.path) == 0:  # Make there is a point.
                    break

                next_point = self.algorithm.pop_next_point()
                self.robot.go_to(next_point[0], next_point[1])

                # Wait for the robot to finish travelling.
                while self.robot.state != "Travelled":
                    continue

                self.robot.state = ""  # Reset the state.

                # Write the state and debug info after the movement.
                self.write_state()
                self.write_debug_info()

                x_difference = self.map.goal_x - self.robot.get_cell_x()
                y_difference = self.map.goal_y - self.robot.get_cell_y()

                if x_difference < 0:
                    x_difference = +x_difference

                if y_difference < 0:
                    y_difference = +y_difference

            self.robot.halt()

        except exceptions.NoPathException as ex:
            print(ex)
        except Exception as ex:
            print(ex)
        finally:
            execution_time = round(time.process_time() - start_time, 3)

            # Write the final state.
            self.algorithm.print_cost_grid(sys.stdout)
            self.algorithm.print_occupancy_grid(sys.stdout)
            self.algorithm.print_debug(sys.stdout)
            sys.stdout.write("Total Execution Time (seconds): " + str(execution_time))

            if self.output_file is not None:
                if not self.output_file.closed:
                    self.algorithm.print_cost_grid(self.output_file)
                    self.algorithm.print_occupancy_grid(self.output_file)
                    self.algorithm.print_debug(self.output_file)
                    self.output_file.write("Total Execution Time (seconds): " + str(execution_time))

            # Write all of the paths that were used to the gnuplot file.
            if self.gnuplot_file is not None:
                if not self.gnuplot_file.closed:
                    result_generator.write_paths(self.gnuplot_file, paths)

            if not self.finished:
                self.finished = True

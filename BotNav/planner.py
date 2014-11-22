import threading

from threading import Thread

from events import ScanResult
import exceptions

'''
The Planner actually deals with moving the robot from A to B until it
reaches the goal. It makes use of the map, robot, and a path planning
algorithm to do this. The algorithm being used must implement: 
replan(), update_occupancy_grid(), and check_plan().

It is implemented on its own thread.
'''


class Planner(threading.Thread):
    '''
    Initialises the planner with a map and algorithm. It gets the
    robot from the map it is passed.
    '''

    def __init__(self, map, algorithm, proxy):
        self.map = map
        self.robot = self.map.robot
        self.algorithm = algorithm
        self.proxy = proxy
        self.proxy.listeners.append(self)
        self.finished = False
        self.last_scan = 0

        Thread.__init__(self)

    def handle_event(self, event):
        if isinstance(event, ScanResult):
            self.last_scan = event


    '''
    The actions of the Planner using any algorithm are:

        1. Plan
        2. Check sensors to find obstacles.
        3. Update map if sensors show a discrepancy.
        4. If the map has changed, recompute the plan.
        5. Check plan and initiate movement along shortest path.
        6. Go to 2.

    Until we reach the goal.
    '''

    def run(self):

        # Print the initial state.
        self.algorithm.print_path()
        self.algorithm.print_cost_grid()
        self.algorithm.print_occupancy_grid()

        try:
            '''
            Step 1: Plan.
            '''
            self.algorithm.plan()

            # Print state after first planning step.
            self.algorithm.print_path()
            self.algorithm.print_cost_grid()
            self.algorithm.print_occupancy_grid()

            # Calculate our initial distance from the goal.
            x_difference = self.map.goal_x - self.robot.x
            y_difference = self.map.goal_y - self.robot.y

            if x_difference < 0:
                x_difference = -x_difference

            if y_difference < 0:
                y_difference = -y_difference

            print("cell x: %.2f" % self.robot.x + ", cell y: %.2f" % self.robot.y)
            print(
            "x: %.2f" % (self.robot.x * self.map.cell_size) +
            ", y: %.2f" % (self.robot.y * self.map.cell_size) +
            ", heading %.2f" % self.robot.heading)
            print("\n" + ('-' * 73) + "\n")

            # While we are not with 0.5 cells of the goal in both x and y.
            while x_difference > 0.5 or y_difference > 0.5:
                '''
                Step 2: Scan the immediate area for obstacles and free space.
                '''
                self.robot.ping()

                while self.last_scan == 0:
                    continue

                # Just take 1 reading for now.
                affected_cells = self.map.ping_to_cells(
                    round(float(self.last_scan.readings[0]) / self.map.cell_size, 2))
                self.last_scan = 0

                '''
                Step 3: Update the map if necessary.
                '''
                if len(affected_cells) > 0:
                    updated_cells = self.map.update_map(affected_cells)
                    if len(updated_cells) > 0:
                        self.algorithm.update_occupancy_grid(updated_cells)

                        '''
                        Step 4: Recompute the plan if necessary.
                        '''
                        self.algorithm.plan()

                '''
                Step 5: Pop the next point from the current path.
                '''
                new_point = self.algorithm.pop_next_point()
                self.robot.go_to(new_point[0], new_point[1])

                # Wait for the robot to finish travelling.
                while self.robot.state != "Travelled":
                    continue

                self.robot.state = ""  # Reset the state.

                # Print some information.
                self.algorithm.print_path()
                self.algorithm.print_cost_grid()
                self.algorithm.print_occupancy_grid()
                print("cell x: %.2f" % self.robot.x + ", cell y: %.2f" % self.robot.y)
                print(
                "x: %.2f" % (self.robot.x * self.map.cell_size) +
                ", y: %.2f" % (self.robot.y * self.map.cell_size) +
                ", heading %.2f" % self.robot.heading
                )
                print("\n" + ('-' * 73) + "\n")

                # Recalculate the x and y differences from our position
                # to the goal.
                x_difference = self.map.goal_x - self.robot.x
                y_difference = self.map.goal_y - self.robot.y

                if x_difference < 0:
                    x_difference = +x_difference

                if y_difference < 0:
                    y_difference = +y_difference

            self.robot.halt()

            if not self.finished:
                self.finished = True
        except exceptions.NoPathException as ex:
            print(ex)

            if not self.finished:
                self.finished = True
        finally:
            # Print the final state.
            self.algorithm.print_path()
            self.algorithm.print_cost_grid()
            self.algorithm.print_occupancy_grid()

'''
An object used to store odometry data.
'''


class OdometryReport:
    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading

    def __str__(self):
        return ("x: " + str(self.x) + ", y: " + str(self.y)
                + ", heading: " + str(self.heading))


'''
An object used to store the results of a scan.
'''


class ScanResult:
    def __init__(self, readings):
        self.readings = readings

    def __str__(self):
        string = "scan: "

        for reading in self.readings:
            string += str(reading) + ", "

        return string


'''
An object used to store a state change.
'''


class StateEvent:
    def __init__(self, state):
        self.state = state

    def __str__(self):
        return "state: " + str(self.state)

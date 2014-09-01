'''
Used to store a robots odometry at a given time. 
'''
class OdometryReport:
    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading

	'''
	Returns a string representation of the odometry.
	'''
    def to_string(self):
        return "x: " + str(self.x) + ", y: " + str(self.y) + ", heading: " + str(self.heading)

'''
Used to store a list of readings generated from a scan.
'''
class ScanResult:
    def __init__(self, readings):
        self.readings = readings

	'''
	Returns a string representation of the scan.
	'''
    def to_string(self):
        string = ""
        
        for reading in self.readings:
            string += str(reading) + ", "

        return string

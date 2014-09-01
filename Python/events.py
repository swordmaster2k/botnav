class OdometryReport:
    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading

    def to_string(self):
        return "x: " + str(self.x) + ", y: " + str(self.y) + ", heading: " + str(self.heading)

class ScanResult:
    def __init__(self, readings):
        self.readings = readings

    def to_string(self):
        string = ""
        
        for reading in self.readings:
            string += str(reading) + ", "

        return string

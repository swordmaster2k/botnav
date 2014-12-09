"""
Super class that all connection types should inherit from defines and implements some of the
methods that a connection contains.
"""


class Connection():
    def __init__(self, infile, outfile, connection):
        self.infile = infile  # File opened with 'r' option.
        self.outfile = outfile  # File opened with 'w' option.

        self.connection = connection

        self.closed = self.infile.closed and self.outfile.closed

    '''
    Write data to the connection, assumes that the stream can be wrote to.
    '''

    def write(self, data):
        if not self.outfile.closed:
            self.outfile.write(data)
            self.outfile.flush()

    '''
    Read a line from the connection, assumes that the stream can be read from.
    '''

    def readline(self):
        if not self.infile.closed:
            return self.infile.readline()

    '''
    Base class will handle close because closing physical connection types vary.
    '''

    def close(self):
        raise NotImplementedError
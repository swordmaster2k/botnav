import socket

from .connection import Connection

'''
Wrapper class for the Python3.3 bluetooth socket implementation, it 
communicates with the bluetooth device using input and output files.
'''


class BluetoothConnection(Connection):
    '''
    Creates a new BluetoothConnection to the bluetooth address on the
    specified port.
    '''

    def __init__(self, addr, port):
        self.addr = addr
        self.port = port

        self.sock = socket.socket(socket.AF_BLUETOOTH,
                                  socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.sock.connect((self.addr, self.port))

        Connection.__init__(self, self.sock.makefile('r'), self.sock.makefile('w'))

    '''
    Closes the socket connection and any open files.
    '''

    def close(self):
        self.infile.close()
        self.outfile.close()
        self.sock.close()
        self.closed = True

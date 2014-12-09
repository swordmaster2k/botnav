import socket

from .connection import Connection

'''
Wrapper class for the Python3.3 IP socket implementation, it 
communicates with the IP device using input and output files.
'''


class IPConnection(Connection):
    def __init__(self, ip, port):
        socket.setdefaulttimeout(10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((ip, port))

        try:
            self.sock.listen(1)
            client, address = self.sock.accept()
        except:
            print("No client connected across IP link. Aborting")
            exit()

        Connection.__init__(self, client.makefile('r'), client.makefile('w'), client)

    '''
    Closes the socket connection and any open files.
    '''

    def close(self):
        self.infile.close()
        self.outfile.close()
        self.sock.close()
        self.closed = True

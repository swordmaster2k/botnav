import sys
import socket

'''
Wrapper class for the Python3.3 bluetooth socket implementation, it 
communicates with the bluetooth device using input and output files.
'''
class BluetoothConnection():

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
		
		self.infile = self.sock.makefile('r')
		self.outfile = self.sock.makefile('w')
		
		self.closed = self.infile.closed and self.outfile.closed
	
	'''
	Writes data to the output file.
	'''	
	def write(self, data):
		self.outfile.write(data)
		self.outfile.flush()
	
	'''
	Reads a line from the input file.
	'''	
	def readline(self):
		return self.infile.readline()
		
	'''
	Closes the socket connection and any open files.
	'''
	def close(self):
		self.infile.close()
		self.outfile.close()
		self.sock.close()
		self.closed = True


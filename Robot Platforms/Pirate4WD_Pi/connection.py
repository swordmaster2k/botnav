import sys
import socket

'''

'''
class IPConnection:
	'''
	
	'''
	def __init__(self, ip, port):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sock.connect((ip, port))
		
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
		try:
			self.sock.close()
			self.infile.close()
			self.outfile.close()
			self.closed = True
		except:
			print("Broken Pipe.")
		finally:
			self.sock.close()
			self.closed = True

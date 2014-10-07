import sys
import socket

'''
Wrapper class for the Python3.3 IP socket implementation, it 
communicates with the IP device using input and output files.
'''
class IPConnection():
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
		
		self.infile =  client.makefile('r')
		self.outfile = client.makefile('w')
		
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

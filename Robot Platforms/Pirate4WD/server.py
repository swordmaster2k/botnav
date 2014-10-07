import time
import socket

backlog = 1
size = 1024

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(("10.42.0.1", 50001))

server.listen(backlog)

try:
	client, address = server.accept()
	
	print("client connected")

	while 1:
		#data = client.recv(size)

		#if data:
			#print(data)
			#client.send(data)
			
		time.sleep(3)
		server.write("h\n")
except:	
	print("Closing socket")	
	client.close()
	server.close()

import sys
import bluetooth

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

bt_addr="00:00:12:06:56:83"
port = 0x1001

print("trying to connect to %s on PSM 0x%X" % (bt_addr, port))

sock.connect((bt_addr, port))
file = sock.makefile('w')
filein = sock.makefile('r')

print("connected.")

while True:
    #data = input()
    #if(len(data) == 0): break
    #sock.send("a")
    #data = sock.recv(1024)
    file.write("e\n")
    print(filein.readline())

sock.close()

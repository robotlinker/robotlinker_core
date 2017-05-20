# Echo client program
import socket
import time

home_joint = [0.78539815, -2.0943950667, 1.7453292222, -1.2217304556, -1.5707963, 1.5707963]

HOST = "192.168.0.50    "    # The remote host
PORT = 30002              # The same port as used by the server
print "Starting Program"
count = 0
while (count < 1):
 s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
 s.connect((HOST, PORT))
 time.sleep(0.5)


 s.send ("movej([0.78539815, -2.0943950667, 1.7453292222, -1.2217304556, -1.5707963, 1.5707963], a=0.3962634015954636, v=0.2471975511965976)" + "\n")
 time.sleep(2)
 count = count + 1
 print "The count is:", count
 time.sleep(1)
 data = s.recv(1024)
 
s.close()
print ("Received", repr(data))
print "Program finish"

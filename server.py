import socket
import sys

s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

port = 8400
#10.29.106.95
s.bind(("", port))

s.listen(5)

while True:
    c, addr = s.accept()
    print("Got connection from", addr)
    c.send("f100".encode())
    c.close()
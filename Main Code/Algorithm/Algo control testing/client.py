import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("172.21.145.192", 1234))

msg = s.recv(1024)
print(msg.decode("utf-8"))
import pickle
import socket


class RPiClient:
    """
    Used for connecting to...
    """
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = socket.socket()

    def connect(self):
        self.socket.connect((self.host, self.port))

    def send_message(self, obj):
        # self.socket.sendall(pickle.dumps(obj))
        for command in obj:
            self.socket.send(command.encode('utf-8'))

    def receive_message(self):
        data = self.socket.recv(1024)
        print(data.__len__())
        if not data:
            return False
        return data

    def close(self):
        self.socket.close()

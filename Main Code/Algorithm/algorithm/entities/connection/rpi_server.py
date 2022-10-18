import socket
import pickle


class RPiServer:
    """
    Used as the server in the RPi.
    """
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = socket.socket()

        self.__data = []
        self.conn, self.address = None, None

    def start(self):
        print(f"Creating server at {self.host}:{self.port}")
        self.socket.bind((self.host, self.port))
        self.socket.listen()
        print("Listening for connection...")

        self.conn, self.address = self.socket.accept()
        print(f"Connection from {self.address}")

    def receive_data(self):
        assert self.conn is not None and self.address is not None
        with self.conn:
            print(f"Connection from {self.address}")
            while True:
                d = self.conn.recv(1024)
                if not d:
                    break
                self.__data.append(d)

        print(self.__data)
        # This may allow arbitrary code execution. Only connect to trusted connections!!!
        return pickle.loads(b''.join(self.__data))

    def close(self):
        print("Closing socket.")
        self.socket.close()

from mdp_comm.core import BaseServer, ConnectionState
from mdp_comm.utils.socket_utils import send_msg, recv_msg

import io
import picamera
import socket

class ImageServer(BaseServer):

    def __init__(self, recv_callback):
        super().__init__(recv_callback)

        self._state = ConnectionState.STARTUP

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind(('0.0.0.0', 50000))
        self.socket.listen()

        self.client_sock: socket.socket = None

    def _send(self, msg):

        if self._state == ConnectionState.CONNECTED:
            try:
                send_msg(self.client_sock, msg)

            except IOError:
                print("disconnected")
                self._state = ConnectionState.DISCONNECTED

    def is_connected(self):
        return self._state == ConnectionState.CONNECTED

    def capture_and_send(self):
        stream = io.BytesIO()
        with picamera.PiCamera() as camera:
            camera.capture(stream, format='jpeg', use_video_port=True)
            jpeg = stream.getvalue()
            self.send(jpeg)

    def _server_loop(self):

        while True:
            if self._state == ConnectionState.STARTUP or self._state == ConnectionState.DISCONNECTED:
                print("Waiting for conn...")
                self.client_sock, addr = self.socket.accept()
                self._state = ConnectionState.CONNECTED
                print(f"Connected to {addr}")

            elif self._state == ConnectionState.CONNECTED:
                try:
                    data = recv_msg(self.client_sock)
                    self.recv_callback(data)
                except IOError:
                    print("disconnected")
                    self._state = ConnectionState.DISCONNECTED
                except Exception as e:
                    print(e)

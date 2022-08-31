import socket
from connection_state import ConnectionState
from socket_utils import send_msg, recv_msg
import threading
import picamera
import io
import time

class ImageServer:

    def __init__(self, recv_callback=lambda x: None):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind(('0.0.0.0', 50000))
        self.socket.listen()
        self._state = ConnectionState.STARTUP
        self.recv_callback = recv_callback

    def send(self, msg):
        
        if self._state == ConnectionState.CONNECTED:
            try:
                send_msg(self.client_sock, msg)

            except IOError:
                print("disconnected")
                self._state = ConnectionState.DISCONNECTED

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
                    
    def run(self):
        t = threading.Thread(target=self._server_loop)
        t.start()
        print("Started...")

    
            
if __name__ == "__main__":
    i = ImageServer(print)
    i.run()
    time.sleep(15)
    i.capture_and_send()

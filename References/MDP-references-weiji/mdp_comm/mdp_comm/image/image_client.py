from mdp_comm.utils.socket_utils import send_msg, recv_msg

import socket
import cv2
import numpy as np

class ImageClient:

    def __init__(self, addr, port, recv_callback=lambda x:None):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((addr, port))
        self.recv_callback = recv_callback

    def loop(self):
        while True:
            data = recv_msg(self.socket)
            img = cv2.imdecode(np.fromstring(data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            self.recv_callback(img)
            

    def send(self, msg):
        send_msg(self.socket, msg)

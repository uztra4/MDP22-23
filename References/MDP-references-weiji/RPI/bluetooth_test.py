"""
A simple test server that returns a random number when sent the text "temp" via Bluetooth serial.
"""

import os
import glob
import time
import random
import socket

from bluetooth import *

server_sock = BluetoothSocket(RFCOMM)
server_sock.bind(("", PORT_ANY))
server_sock.listen(1)

port = server_sock.getsockname()[1]

uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

advertise_service(
    server_sock,
    "TestServer",
    service_id=uuid,
    service_classes=[uuid, SERIAL_PORT_CLASS],
    profiles=[SERIAL_PORT_PROFILE],
)

print("Waiting for connection on RFCOMM channel %d" % port)
client_sock, client_info = server_sock.accept()
print("Accepted connection from ", client_info)

while True:

    try:
        req = client_sock.recv(1024)
        if len(req) == 0:
            break
        print("received [%s]" % req)

        data = None
        if req in (b"temp\r\n", "*temp"):
            data = str(random.random()) + "!"
        else:
            pass

        if data:
            print("sending [%s]" % data)
            client_sock.send(data)

    except IOError:
        print("disconnected")
        client_sock.close()
        server_sock.shutdown(socket.SHUT_RDWR)
        server_sock.close()

    except KeyboardInterrupt:

        print("disconnected")

        client_sock.close()
        server_sock.shutdown(socket.SHUT_RDWR)
        server_sock.close()
        print("all done")

        break

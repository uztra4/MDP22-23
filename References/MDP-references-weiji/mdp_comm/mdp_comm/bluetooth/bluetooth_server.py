from mdp_comm.core import BaseServer, ConnectionState

import bluetooth

class BluetoothServer(BaseServer):

    def __init__(self, recv_callback):
        super().__init__(recv_callback)

        self._state = ConnectionState.STARTUP

        self._socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self._socket.bind(("", bluetooth.PORT_ANY))
        self._socket.listen(1)

        self.client_sock: bluetooth.BluetoothSocket = None

        uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

        bluetooth.advertise_service(
            self._socket,
            "TestServer",
            service_id=uuid,
            service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
            profiles=[bluetooth.SERIAL_PORT_PROFILE],
        )

    def is_connected(self):
        return self._state == ConnectionState.CONNECTED

    def _send(self, msg):
        if self._state == ConnectionState.CONNECTED:
            try:
                self.client_sock.send(msg)

            except IOError:
                print("disconnected")
                self._state = ConnectionState.DISCONNECTED

    def _server_loop(self):
        while True:
            if self._state == ConnectionState.STARTUP or self._state == ConnectionState.DISCONNECTED:
                # wait for connenction
                print("Waiting for connection on RFCOMM channel %d" % self._socket.getsockname()[1])
                self.client_sock, client_info = self._socket.accept()
                self._state = ConnectionState.CONNECTED
                print("Accepted connection from ", client_info)

            elif self._state == ConnectionState.CONNECTED:
                # handle connection
                try:
                    req = self.client_sock.recv(1024)
                    if len(req) == 0:
                        break

                    self.recv_callback(req)

                except IOError:
                    print("disconnected")
                    self._state = ConnectionState.DISCONNECTED

            else:
                raise RuntimeError("Unknown state")

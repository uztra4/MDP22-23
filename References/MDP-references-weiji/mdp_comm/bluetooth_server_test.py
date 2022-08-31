import time
from mdp_comm.bluetooth.bluetooth_server import BluetoothServer

b = BluetoothServer(print)
b.run()

while not b.is_connected():
    time.sleep(1)



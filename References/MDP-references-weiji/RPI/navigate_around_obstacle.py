import time
from RPI.obstacle_storage import ObstacleStorage
from RPI.bluetooth_server import BluetoothServer
from RPI.image_server import ImageServer
from RPI.stm32 import STM32
from RPI.rpimessageparserV4 import androidToRpi

if __name__ == "__main__":
    found = False

    image_server = ImageServer()
    image_server.run()
    def image_server_callback(msg):
        global found
        if msg.decode() != "Target":
            found = True
            print("Found", msg)
    image_server.recv_callback = image_server_callback

    robot = STM32()
    robot.connect()

    path = [("r", ), ("f", 50), ("l", ), ("f", 50), ("l", )]

    def nav_path(path):
        for move in path:
            robot.direction_wrapper(*move)
    

    time.sleep(10)

    for x in range(3):
        image_server.capture_and_send()
        time.sleep(2)
        if found:
            print("Found!")
        else:
            nav_path(path)
    
    image_server.capture_and_send()
    time.sleep(2)
    if found:
        print("Found!")

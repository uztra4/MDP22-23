from RPI.obstacle_storage import ObstacleStorage
from RPI.bluetooth_server import BluetoothServer
from RPI.image_server import ImageServer
from RPI.stm32 import STM32
from RPI.rpimessageparserV4 import androidToRpi

if __name__ == "__main__":
    obstacle_storage = ObstacleStorage()
    bluetooth_server = BluetoothServer()
    image_server = ImageServer()
    robot = STM32()
    robot.connect()

    def bluetooth_handler(msg):
        parsed = androidToRpi(msg)
        if parsed[0] == "moving":
            robot.direction_wrapper(parsed[1])
        elif parsed[0] == "Obstacle":
            _, x, y, obstacle_id, obstacle_dir = parsed
            obstacle_storage.update_obstacle(obstacle_id, obstacle_dir, x, y)
        elif parsed[0] == "starting":
            pass


    temp = bluetooth_handler
    # def temp(msg):
    #     if b'capture' in msg:
    #         image_server.capture_and_send()
            
    bluetooth_server.recv_callback  = temp
    bluetooth_server.run()
    # image_server.run()

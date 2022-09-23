import sys
import time
from typing import List
import pickle

import settings
from app import AlgoSimulator, AlgoMinimal
from entities.effects.direction import Direction
from entities.connection.rpi_client import RPiClient
from entities.connection.rpi_server import RPiServer
from entities.grid.obstacle import Obstacle


class Main:
    def __init__(self):
        self.client = None

    def parse_obstacle_data(self, data) -> List[Obstacle]:
        obs = []
        for obstacle_params in data:
            obs.append(Obstacle(obstacle_params[0],
                                obstacle_params[1],
                                Direction(obstacle_params[2]),
                                obstacle_params[3]))
        # [[x, y, orient, index], [x, y, orient, index]]
        return obs

    def run_simulator(self):
        # Fill in obstacle positions with respect to lower bottom left corner.
        # (x-coordinate, y-coordinate, Direction)
        # obstacles = [[15, 75, 0, 0]]
        # obs = parse_obstacle_data(obstacles)
        obs = self.parse_obstacle_data([])
        app = AlgoSimulator(obs)
        app.init()
        app.execute()

    def run_minimal(self, also_run_simulator):
        # Create a client to connect to the RPi.
        if self.client is None:
            print(f"Attempting to connect to {settings.RPI_HOST}:{settings.RPI_PORT}")
            self.client = RPiClient(settings.RPI_HOST, settings.RPI_PORT)
            # Wait to connect to RPi.
            while True:
                try:
                    self.client.connect()
                    break
                except OSError:
                    pass
                except KeyboardInterrupt:
                    self.client.close()
                    sys.exit(1)
            print("Connected to RPi!\n")
        # client.send_message(["STM:w\n", "STM:a\n", "STM:d\n"])
        # Create a server to receive information from the RPi.
        # server = RPiServer(settings.PC_HOST, settings.PC_PORT)
        # # Wait for the RPi to connect to the PC.
        # try:
        #     server.start()
        # except OSError or KeyboardInterrupt as e:
        #     print(e)
        #     server.close()
        #     client.close()
        #     sys.exit(1)
        #
        # # At this point, both the RPi and the PC are connected to each other.
        # # Create a synchronous call to wait for RPi data.
        # data: list = server.receive_data()
        # server.close()

        print("Waiting to receive obstacle data from RPi...")
        d = self.client.receive_message()
        print("Got data from RPi:")
        d = d.decode('utf-8')
        d = d.split(',')
        print(d)
        if len(d) != 1:
            d = [eval(i) for i in d]
            print(d)
            data = []
            data.append(d)
        else:
            data = d
        #
        # while True:
        #     d = client.receive_message()
        #     print(d)
        #     if not d:
        #         break
        #     print("Got data from RPi:")
        #     d = d.decode('utf-8')
        #     d = d.split(',')
        #     d = [eval(i) for i in d]
        #     print(d)
        #     data.append(d)

        self.decision(self.client, data, also_run_simulator)

    def decision(self, client, data, also_run_simulator):
        if isinstance(data[0], list):
            obstacles = self.parse_obstacle_data(data)
            if also_run_simulator:
                app = AlgoSimulator(obstacles)
                app.init()
                app.execute()
            app = AlgoMinimal(obstacles)
            app.init()
            app.execute()
            # Send the list of commands over.
            print("Sending list of commands to RPi...")
            commands = app.robot.convert_all_commands()
            print(commands)
            client.send_message(commands)
            # client.close()

        elif isinstance(data[0], str):
            if data[0] == "bullseye":
                print("Sending list of commands to RPi...")
                commands = ["STM:b030\n", "STM:r\n", "STM:f010\n", "STM:l\n", "STM:l\n", "STM:f010\n", "STM:p\n"]
                print(commands)
                client.send_message(commands)

    def run_rpi(self):
        while True:
            self.run_minimal(False)
            time.sleep(5)


def init():
    algo = Main()
    algo.run_rpi()


def sim():
    algo = Main()
    algo.run_simulator()


if __name__ == '__main__':
    # sim()
    init()

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
        self.commands = None

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

        # Wait for message from RPI
        print("Waiting to receive data from RPi...")
        d = self.client.receive_message()
        print("Got data from RPi:")
        d = d.decode('utf-8')
        d = d.split(',')
        print(d)
        if len(d) != 1:
            print(d)
            data = []
            for i in range(0, len(d), 4):
                data.append(d[i:i + 4])
        else:
            data = d

        print(data)

        for i in range(len(data)):
            data[i][0] = 10 * int(data[i][0]) + 5
            # 200 - flip obstacle plot according to android
            data[i][1] = 200 - (10 * int(data[i][1]) + 5)
            match data[i][2]:
                case 'N':
                    data[i][2] = 90
                case 'S':
                    data[i][2] = -90
                case 'E':
                    data[i][2] = 0
                case 'W':
                    data[i][2] = 180
            data[i][3] = int(data[i][3])

        # data = [[145, 35, 180, 0], [115, 85, 0, 1], [25, 155, -90, 2], [175, 175, 180, 3], [105, 115, 180, 4]]
        self.decision(self.client, data, also_run_simulator)

    def decision(self, client, data, also_run_simulator):
        def isvalid(img):
            checklist = [str(i) for i in range(32)]
            if img in checklist:
                return True
            return False

        # Obstacle list
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
            self.commands = app.robot.convert_all_commands()
            self.commands.append("RPI:STOPPED\n")
            if len(self.commands) != 0:
                sent_commands = self.commands[:self.commands.index("STM:pn\n") + 1]
                self.commands = self.commands[self.commands.index("STM:pn\n") + 1:]
                print(sent_commands)
                print(self.commands)
                client.send_message(sent_commands)
                # client.close()

        # String commands from Rpi
        elif isinstance(data[0], str):
            # Check valid image taken
            if isvalid(data[0]):
                if len(self.commands) != 0:
                    sent_commands = self.commands[:self.commands.index("STM:pn\n") + 1]
                    self.commands = self.commands[self.commands.index("STM:pn\n") + 1:]
                    print(sent_commands)
                    print(self.commands)
                    client.send_message(sent_commands)
            elif data[0] == "bullseye":
                print("Sending list of commands to RPi...")
                fixed_commands = ["STM:Ln\n", "STM:w060n\n", "STM:ln\n", "STM:w025n\n", "STM:ln\n", "STM:pn\n"]
                print(fixed_commands)
                client.send_message(fixed_commands)
            # If no image taken
            elif data[0] == "-1":
                correction_commands = ["STM:s010n\n", "STM:pn\n"]
                print(correction_commands)
                client.send_message(correction_commands)

    def run_rpi(self):
        while True:
            self.run_minimal(True)
            time.sleep(5)


def init():
    algo = Main()
    algo.run_rpi()


def sim():
    algo = Main()
    algo.run_simulator()


if __name__ == '__main__':
    sim()
    # init()

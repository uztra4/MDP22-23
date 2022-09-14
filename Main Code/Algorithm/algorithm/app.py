from typing import List
from abc import ABC, abstractmethod

import pygame

import settings
from entities.assets import colors
from entities.assets.direction import Direction
from entities.grid.grid import Grid
from entities.grid.obstacle import Obstacle
from entities.robot.robot import Robot


class AlgoApp(ABC):
    def __init__(self, obstacles: List[Obstacle]):
        self.grid = Grid(obstacles)
        self.robot = Robot(self.grid)
        self.direction = Direction.TOP
        self.obstacles = obstacles
        self.index = 0

    @abstractmethod
    def init(self):
        pass

    @abstractmethod
    def execute(self):
        pass


class AlgoSimulator(AlgoApp):
    """
    Run the algorithm using a GUI simulator.
    """
    def __init__(self, obstacles: List[Obstacle]):
        super().__init__(obstacles)

        self.running = False
        self.size = self.width, self.height = settings.WINDOW_SIZE
        self.screen = self.clock = None

    def init(self):
        """
        Set initial values for the app.
        """
        pygame.init()
        self.running = True

        self.screen = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.clock = pygame.time.Clock()

        # Inform user that it is finding path...
        pygame.display.set_caption("Calculating path...")
        font = pygame.font.SysFont("arial", 35)
        text = font.render("Calculating path...", True, colors.WHITE)
        text_rect = text.get_rect()
        text_rect.center = settings.WINDOW_SIZE[0] / 2, settings.WINDOW_SIZE[1] / 2
        self.screen.blit(text, text_rect)
        pygame.display.flip()

        # Calculate the path.
        self.robot.brain.plan_path()
        pygame.display.set_caption("Simulating path!")  # Update the caption once done.

    def settle_events(self):
        """
        Process Pygame events.
        """
        for event in pygame.event.get():
            # On quit, stop the game loop. This will stop the app.
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                x = pos[0] // settings.GRID_CELL_LENGTH * 10 + 5
                y = abs(pos[1] // settings.GRID_CELL_LENGTH - 19) * 10 + 5
                print("Click ", pos, "Grid coordinates: ", x, y)
                obstacle = [x, y, self.direction.value, self.index]
                self.index += 1
                self.obstacles.append(obstacle)
                obs = self.parse_obstacle_data()
                self.grid = Grid(obs)

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_DOWN:
                    self.direction = Direction.BOTTOM
                    print(self.direction)
                elif event.key == pygame.K_UP:
                    self.direction = Direction.TOP
                    print(self.direction)
                elif event.key == pygame.K_LEFT:
                    self.direction = Direction.LEFT
                    print(self.direction)
                elif event.key == pygame.K_RIGHT:
                    self.direction = Direction.RIGHT
                    print(self.direction)
                elif event.key == pygame.K_SPACE:
                    # Inform user that it is finding path...
                    pygame.display.set_caption("Calculating path...")
                    font = pygame.font.SysFont("arial", 35)
                    text = font.render("Calculating path...", True, colors.WHITE)
                    text_rect = text.get_rect()
                    text_rect.center = settings.WINDOW_SIZE[0] / 2, settings.WINDOW_SIZE[1] / 2
                    self.screen.blit(text, text_rect)
                    pygame.display.flip()
                    self.robot = Robot(self.grid)
                    # Calculate the path.
                    self.robot.brain.plan_path()
                    pygame.display.set_caption("Simulating path!")

    def parse_obstacle_data(self) -> List[Obstacle]:
        obs = []
        for obstacle_params in self.obstacles:
            obs.append(Obstacle(obstacle_params[0],
                                obstacle_params[1],
                                Direction(obstacle_params[2]),
                                obstacle_params[3]))
        # [[x, y, orient, index], [x, y, orient, index]]
        return obs

    def do_updates(self):
        self.robot.update()

    def render(self):
        """
        Render the screen.
        """
        self.screen.fill(colors.WHITE, None)

        self.grid.draw(self.screen)
        self.robot.draw(self.screen)

        # Really render now.
        pygame.display.flip()

    def execute(self):
        """
        Initialise the app and start the game loop.
        """
        while self.running:
            # Check for Pygame events.
            self.settle_events()
            # Do required updates.
            self.do_updates()

            # Render the new frame.
            self.render()

            self.clock.tick(settings.FRAMES)


class AlgoMinimal(AlgoApp):
    """
    Minimal app to just calculate a path and then send the commands over.
    """
    def __init__(self, obstacles):
        # We run it as a server.
        super().__init__(obstacles)

    def init(self):
        pass

    def execute(self):
        # Calculate path
        print("Calculating path...")
        self.robot.brain.plan_path()
        print("Done!")

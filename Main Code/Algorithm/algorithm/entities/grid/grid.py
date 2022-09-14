import math
from collections import deque
from typing import List

import pygame

import settings
from entities.assets import colors
from entities.grid.node import Node
from entities.grid.obstacle import Obstacle
from entities.grid.position import Position


class Grid:
    def __init__(self, obstacles: List[Obstacle]):
        self.obstacles = obstacles
        self.nodes = self.generate_nodes()

    def generate_nodes(self):
        """
        Generate the nodes for this grid.
        """
        nodes = deque()
        for i in range(settings.GRID_NUM_GRIDS):
            row = deque()
            for j in range(settings.GRID_NUM_GRIDS):
                x, y = (settings.GRID_CELL_LENGTH // 2 + settings.GRID_CELL_LENGTH * j), \
                       (settings.GRID_CELL_LENGTH // 2 + settings.GRID_CELL_LENGTH * i)
                new_node = Node(x, y, not self.check_valid_position(Position(x, y)))
                row.append(new_node)
            nodes.appendleft(row)
        return nodes

    def get_coordinate_node(self, x, y):
        """
        Get the corresponding Node object that contains specified x, y coordinates.

        Note that the x-y coordinates are in terms of the grid, and must be scaled properly.
        """
        col_num = math.floor(x / settings.GRID_CELL_LENGTH)
        row_num = settings.GRID_NUM_GRIDS - math.floor(y / settings.GRID_CELL_LENGTH) - 1
        try:
            return self.nodes[row_num][col_num]
        except IndexError:
            return None

    def copy(self):
        """
        Return a copy of the grid.
        """
        nodes = []
        for row in self.nodes:
            new_row = []
            for col in row:
                new_row.append(col.copy())
            nodes.append(new_row)
        new_grid = Grid(self.obstacles)
        new_grid.nodes = nodes
        return new_grid

    def check_valid_position(self, pos: Position):
        """
        Check if a current position can be here.
        """
        # Check if position is inside any obstacle.
        if any(obstacle.check_within_boundary(*pos.xy()) for obstacle in self.obstacles):
            return False

        # Check if position too close to the border.
        # NOTE: We allow the robot to overextend the border a little!
        # We do this by setting the limit to be GRID_CELL_LENGTH rather than ROBOT_SAFETY_DISTANCE
        if (pos.y < settings.GRID_CELL_LENGTH or
            pos.y > settings.GRID_LENGTH - settings.GRID_CELL_LENGTH) or \
                (pos.x < settings.GRID_CELL_LENGTH or
                 pos.x > settings.GRID_LENGTH - settings.GRID_CELL_LENGTH):
            return False
        return True

    @classmethod
    def draw_arena_borders(cls, screen):
        """
        Draw the arena borders.
        """
        # Draw upper border
        pygame.draw.line(screen, colors.RED, (0, 0), (settings.GRID_LENGTH, 0))
        # Draw lower border
        pygame.draw.line(screen, colors.RED, (0, settings.GRID_LENGTH), (settings.GRID_LENGTH, settings.GRID_LENGTH))
        # Draw left border
        pygame.draw.line(screen, colors.RED, (0, 0), (0, settings.GRID_LENGTH))
        # Draw right border
        pygame.draw.line(screen, colors.RED, (settings.GRID_LENGTH, 0), (settings.GRID_LENGTH, settings.GRID_LENGTH))

    def draw_obstacles(self, screen):
        for ob in self.obstacles:
            ob.draw(screen)

    def draw_nodes(self, screen):
        for row in self.nodes:
            for col in row:
                col.draw(screen)

    def draw(self, screen):
        # Draw nodes
        self.draw_nodes(screen)
        # Draw arena borders
        self.draw_arena_borders(screen)
        # Draw obstacles
        self.draw_obstacles(screen)

import settings
from entities.effects.direction import Direction


class Position:
    def __init__(self, x, y, direction: Direction = None):
        """
        x and y coordinates are in terms of the grid.
        Note that they should already be scaled properly.

        Most of the time, we do not need to set angle. Should only be used for the robot.
        Note that the angle should be in DEGREES.
        """
        self.x = x
        self.y = y
        self.direction = direction

    def __str__(self):
        return f"Position({(self.x // settings.SCALING_FACTOR)}, {self.y // settings.SCALING_FACTOR}, " \
               f"angle={self.direction})"

    __repr__ = __str__

    def xy(self):
        """
        Return the true x, y coordinates of the current Position.
        """
        return self.x, self.y

    def xy_dir(self):
        return *self.xy(), self.direction

    def get_dir(self):
        return self.direction

    def xy_pygame(self):
        """
        Return the x, y coordinates in terms of Pygame coordinates. Useful for drawing on screen.
        """
        return self.x, settings.GRID_LENGTH - self.y

    def copy(self):
        """
        Create a new copy of this Position.
        """
        return Position(self.x, self.y, self.direction)


class RobotPosition(Position):
    def __init__(self, x, y, direction: Direction = None, angle=None):
        super().__init__(x, y, direction)
        self.angle = angle
        if direction is not None:
            self.angle = direction.value

    def __str__(self):
        return f"RobotPosition({super().__str__()}, angle={self.angle})"

    __repr__ = __str__

    def copy(self):
        return RobotPosition(self.x, self.y, self.direction, self.angle)

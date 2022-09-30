import math

import settings
from entities.effects.direction import Direction
from entities.commands.command import Command
from entities.grid.position import Position, RobotPosition


class SpotTurnCommand(Command):
    def __init__(self, angle):

        time = abs((math.radians(angle) * settings.ROBOT_LENGTH) /
                   (settings.ROBOT_SPEED_PER_SECOND * settings.ROBOT_S_FACTOR))
        super().__init__(time)

        self.angle = angle

    def __str__(self):
        return f"SpotTurnCommand({self.angle:.2f}degrees, {self.total_ticks} ticks)"

    __repr__ = __str__

    def process_one_tick(self, robot):
        if self.total_ticks == 0:
            return

        self.tick()
        angle = self.angle / self.total_ticks
        robot.spot(angle)

    def apply_on_pos(self, curr_pos: Position):
        curr_pos.angle += self.angle

        if curr_pos.angle < -180:
            curr_pos.angle += 2 * 180
        elif curr_pos.angle >= 180:
            curr_pos.angle -= 2 * 180

        # Update the Position's direction.
        if 45 <= curr_pos.angle <= 3 * 45:
            curr_pos.direction = Direction.TOP
        elif -45 < curr_pos.angle < 45:
            curr_pos.direction = Direction.RIGHT
        elif -45 * 3 <= curr_pos.angle <= -45:
            curr_pos.direction = Direction.BOTTOM
        else:
            curr_pos.direction = Direction.LEFT
        return self

    def convert_to_message(self):
        if self.angle > 0:
            # This is spot turn left
            return "STM:gn\n"
        elif self.angle < 0:
            # This is spot turn right
            return "STM:hn\n"

import settings
from entities.assets.direction import Direction
from entities.commands.command import Command
from entities.grid.position import Position


class StraightCommand(Command):
    def __init__(self, dist):
        """
        Specified distance is scaled. Do not divide the provided distance by the scaling factor!
        """
        # Calculate the time needed to travel the required distance.
        time = abs(dist / settings.ROBOT_SPEED_PER_SECOND)
        super().__init__(time)

        self.dist = dist

    def __str__(self):
        return f"StraightCommand(dist={self.dist / settings.SCALING_FACTOR}, {self.total_ticks} ticks)"

    __repr__ = __str__

    def process_one_tick(self, robot):
        if self.total_ticks == 0:
            return

        self.tick()
        distance = self.dist / self.total_ticks
        robot.straight(distance)

    def apply_on_pos(self, curr_pos: Position):
        """
        Apply this command onto a current Position object.
        """
        if curr_pos.direction == Direction.RIGHT:
            curr_pos.x += self.dist
        elif curr_pos.direction == Direction.TOP:
            curr_pos.y += self.dist
        elif curr_pos.direction == Direction.BOTTOM:
            curr_pos.y -= self.dist
        else:
            curr_pos.x -= self.dist

        return self

    def convert_to_message(self):
        # MESSAGE: fXXXX for forward, bXXXX for backward.
        # XXXX is the distance in decimal in centimeters.

        # Note that the distance is now scaled.
        # Therefore, we need to de-scale it.
        descaled_distance = int(self.dist // settings.SCALING_FACTOR)
        # Check if forward or backward.
        if descaled_distance < 0:
            # It is a backward command.
            return f"b{abs(descaled_distance):04}"
        # Else, it is a forward command.
        return f"f{descaled_distance:04}"

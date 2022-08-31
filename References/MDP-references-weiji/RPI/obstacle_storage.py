from dataclasses import dataclass

@dataclass
class Obstacle:
    x: int
    y: int
    obstacle_id: int
    obstacle_direction: str

class ObstacleStorage:

    def __init__(self) -> None:
        self._obstacles = {}

    def update_obstacle(self, obstacle_id, obstacle_direction, x, y):
        self._obstacles[obstacle_id] = Obstacle(obstacle_id=obstacle_id, obstacle_direction=obstacle_direction, x=x, y=y)

    def extract_required_format(self):
        return [obs.obstacle_direction for obs in self._obstacles.values()], [(obs.x, obs.y) for obs in self._obstacles.values()]

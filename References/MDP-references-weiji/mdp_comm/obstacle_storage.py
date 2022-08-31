from dataclasses import dataclass

@dataclass
class Obstacle:
    x: int
    y: int
    obstacle_id: int
    direction: str

class ObstacleStorage:

    def __init__(self) -> None:
        self._obstacles = {}

    def update_obstacle(self, obstacle_id, obstacle_direction, x, y):
        dir_map = {
            "up":"N",
            "down": "S",
            "left":"W",
            "right":"E",
            "null":"null"
        }
        self._obstacles[obstacle_id] = Obstacle(obstacle_id=obstacle_id, direction=dir_map[obstacle_direction], x=x, y=y)

    def extract_required_format(self):
        # return [obs.direction for obs in self._obstacles.values() if obs.direction!="null"], [(obs.x * 10, obs.y * 10) for obs in self._obstacles.values() if obs.direction != "null"], [(obs.x * 10, obs.y * 10) for obs in self._obstacles.values() if obs.direction == "null"]
        return [obs.direction for obs in self._obstacles.values() if obs.direction!="null"], [(obs.x, obs.y) for obs in self._obstacles.values() if obs.direction != "null"], [(obs.x, obs.y) for obs in self._obstacles.values() if obs.direction == "null"]

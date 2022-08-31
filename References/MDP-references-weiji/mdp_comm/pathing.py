import heapq
import copy

NORTH = "N"
SOUTH = "S"
EAST = "E"
WEST = "W"

face_to_dir = {
    NORTH: (0, 1),
    SOUTH: (0 , -1),
    EAST: (1, 0),
    WEST: (-1, 0)
} 


class Pathing:

    def __init__(self, obstacle_coords, faces, other_obstacles) -> None:
        self.obstacle_coords = obstacle_coords
        self.faces = faces
        self.other_obstacles = other_obstacles
        self._goals = self._calculate_goals()
        self.pos = (1,1)

    def _calculate_goals(self):
        goals = []
        for obs, face in zip(self.obstacle_coords, self.faces):
            x, y = obs
            d_x, d_y = face_to_dir[face]
            goals.append((x+3*d_x, y+3*d_y))
    
        return goals

    def get_path(self):
        pos = self.pos
        prev_dir = (0, 1)

        paths = []
        heading_at_targets = []
        heading_at_starts = []

        for goal, face in zip(self._goals, self.faces):
            a = AStar(pos, self.obstacle_coords + self.other_obstacles)
            path = a.a_star(goal, prev_dir)
            paths.append(path)

            heading_at_starts.append(prev_dir)

            pos = path[-1]
            neg_x, neg_y = face_to_dir[face]
            prev_dir = (-neg_x, -neg_y)

            heading_at_targets.append(prev_dir)

        return heading_at_starts, paths, heading_at_targets

class AStar:

    def __init__(self, pos, obstacle_coords) -> None:
        self.obstacle_coords = obstacle_coords
        # self.faces = faces
        self.pos = pos
        self.queue = []
        self.visited = set()

        self._create_semi_obstacles()
    
    def _create_semi_obstacles(self):
        self.semi_obstacles = set()
        for obstacle in self.obstacle_coords:
            for neighbour in self.get_neighbours(obstacle, cell=True):
                self.semi_obstacles.add(neighbour)

    def _back_if_possible(self, prev_dir):
        x, y = self.pos
        d_x, d_y = prev_dir
        new_loc = (x- d_x, y-d_y)

        if self.check_valid_loc(new_loc):
            self.pos = new_loc
            return [(x,y)]

        return []

    def a_star(self, goal, prev_dir):
        path = self._back_if_possible(prev_dir)
        f = self.manhattan_distance(self.pos, goal)
        g = 0
        h = f + g
        self.queue = [self.create_queue_item(h, f, g, self.pos, path)]
        self.visited = {self.pos}

        while self.queue:
            _, _, g, coord, path = heapq.heappop(self.queue)

            if path:
                x_0, y_0 = path[-1]
                x, y = coord
                prev_dir = (x-x_0, y-y_0)


            path = copy.deepcopy(path)
            path.append(coord)

            if coord == goal:
                print(f"{path=}")
                return path

            self.expand_children(coord, goal, g, path, prev_dir=prev_dir)
            
        print("Failed to find a path")

    def expand_children(self, pos, goal, g, path, prev_dir=None):

        g = g + 1

        for ind, child in enumerate(self.get_neighbours(pos, prev_dir=prev_dir)):

            if self.check_valid_loc(child):

                # if child in self.visited:
                #     continue

                # self.visited.add(child)

                # # ensure ordering by the order get neighbours returns
                # f = self.manhattan_distance(child, goal) + 0.00000001*ind
                # # prefer more moves made 
                # h = f + g - (g * 0.00001)
                
                # heapq.heappush(
                #     self.queue,
                #     self.create_queue_item(h, f, g, child, path)
                # )

                if child in self.visited:
                    continue

                self.visited.add(child)

                _g = g

                for n in self.get_neighbours(child, cell=True):
                    if n in self.semi_obstacles:
                        _g += 9
                        break

                # ensure ordering by the order get neighbours returns
                f = self.manhattan_distance(child, goal) + 0.00000001*ind
                # prefer more moves made 
                h = f + _g - (_g * 0.00001)
                
                heapq.heappush(
                    self.queue,
                    self.create_queue_item(h, f, _g, child, path)
                )

    @staticmethod
    def create_queue_item(h, f, g, coord, path):
        return (h, f, g, coord, path)
    
    def check_valid_loc(self, point):
        for pt in (self.get_neighbours(point, cell=True) + [point]):
            if not self.check_bounds(pt) or pt in self.obstacle_coords:
                return False
        return True

    @staticmethod
    def get_neighbours(point, prev_dir=None, cell=False):
        x, y = point
        if cell:
            return [(x+1, y), (x-1, y), (x, y+1), (x, y-1), (x+1, y-1), (x-1, y+1), (x+1, y+1), (x-1, y-1)]

        if prev_dir is None:
            prev_dir = (1, 0)

        p_x, p_y = prev_dir

        assert abs(p_x + p_y) == 1, f"{prev_dir}, weird"
        return [(x+p_x, y+p_y), (x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        
    @staticmethod
    def check_bounds(point):
        x, y = point
        return x >= 0 and y >= 0 and x < 20 and y < 20

    @staticmethod
    def manhattan_distance(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

def create_instructions(heading_at_start, path, heading_at_target):
    path_headings = []

    for ind in range(len(path) - 1):
        p_x, p_y = path[ind]
        x, y = path[ind + 1]
        path_headings.append((x-p_x, y-p_y))

    headings = [heading_at_start] + path_headings + [heading_at_target]
    instructions = []

    for ind in range(len(headings) - 1):
        p_x, p_y = prev_heading = headings[ind]
        x, y = curr_heading = headings[ind + 1]

        neg_prev_heading = (-p_x, -p_y)

        if not instructions:
            prev_move = "f"
        else:
            prev_move = instructions[-1]

        if prev_heading == curr_heading:
            instructions.append(prev_move)

        elif neg_prev_heading == curr_heading:
            if prev_move == "f":
                instructions.append("b")
            elif prev_move == "b":
                instructions.append("f")
                
        else:
            if y*p_x- x*p_y == 1:

                if prev_move == "f":
                    instructions.append("l")
                elif prev_move == "b":
                    instructions.append("r")
            else:
                assert y*p_x- x*p_y == -1

                if prev_move == "f":
                    instructions.append("r")
                elif prev_move == "b":
                    instructions.append("l")

            instructions.append("f")

    if instructions[-1] == "b":
        return instructions[:-1] + ["l", "l"]

    return instructions[:-1]

def compress(string):
    result = []
    counter = 1
    for i in range(len(string)-1):
        if string[i+1] == string[i]:
            counter += 1
            if i == len(string)-2:
                result.append((string[i+1], counter))
        else:
            result.append((string[i], counter))
            counter = 1
            if i == len(string)-2:
                result.append((string[i+1], 1))
    output = []
    for ltr, count in result:
        if ltr not in ['f','b']:
            for i in range(count):
                output.append(ltr)
        else:
            output.append((ltr, count* 10))
    print(string)
    return output

def pathfind(faces, obstacle_coords, other_obstacles):
    p = Pathing(obstacle_coords, faces, other_obstacles)
    results = p.get_path()
    instructions = list(map(lambda x: compress(create_instructions(*x)), zip(*results)))
    print(f"{instructions=}")
    return instructions

if __name__ == "__main__":
    # p = AStar((1,1), [(3, 3), (3, 7), (4, 1)])
    # p.a_star((6, 3), (0, 1))

    pathfind(["S", "N", "N", "E"], [(1, 15), (3, 3), (3, 9), (4, 1)], [])




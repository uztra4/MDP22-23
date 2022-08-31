from math import sqrt
from dataclasses import dataclass
from enum import Enum
import itertools
import typing


@dataclass(eq=True, frozen=True)
class Node:
    x: int
    y: int

    def get_pos(self):
        return (self.x, self.y)

@dataclass(eq=True, frozen=True)
class Robot:
    pos: Node
    _facing: str

    @property
    def facing(self):
        return self._facing

    @facing.setter
    def facing(self, value):
        accepted_vals = ['N','S','E','W']
        if value not in accepted_vals:
            raise ValueError(f'Value must be in {accepted_vals}')
        self._facing = value


class Pathfinder2:
    obstacle_size = '2x2'
    robot_size = '3x3'
    ARENA_SIZE = (19,19)
    # for tuples, standardize that 
    
    @classmethod
    def get_path_between_points(cls, start: Node, target: Node, obstacles: list):
        memo = {}

        def _get_path(start: Node, target: Node):
            sx, sy = cls.extract_pos(start)
            ex, ey = cls.extract_pos(target)

            memo_key = (sx,sy,ex,ey)

            if sx == ex and sy == ey:
                return 

            if memo_key in memo:
                return memo[memo_key]


    @classmethod
    def check_if_robot_is_in_obstacle(self, robot_location:Node, obstacle:list):
        # robot_locations_to_check = [robot_location[0], robot_location[1] for x,y in [(0,1), (1,0), (1,1), (-1,0),(0,-1),(-1,-1)]]
        pass

    @staticmethod
    def extract_pos(node: typing.Union[Node, typing.Tuple]):
        """ Gets the x and y coordinates of an object whose type could be either Node or a simple Tuple """
        if isinstance(node, Node):
            x, y = node.get_pos()
        elif isinstance(node, tuple):
            x, y = node[0], node[1]
        else:
            raise ValueError('Object must be either a Node instance or a tuple of two values (x,y)')

        return x, y

    @staticmethod
    def _change_val(current: int, diff: int, increment: bool):
        """
        Helper function:
        eg. If going right, x-value needs to increment, otherwise it needs to decrement
        This function abstracts that to reduce the amount of code in the main function
        """
        return current + diff if increment else current - diff

class Pathfinder:
    @staticmethod
    def get_path_between_points(node_list: typing.List[Node], obstacles: list):
        '''	
        Returns a path using the internal Pathfinder get_path function that will travel between the given nodes, in the order they were given.
        
        Parameters
        ----------
        node_list: List(Nodes)
            The nodes to travel to, in the order of travel
        obstacles: List(Nodes or Tuples) - mixed Tuple and Nodes are allowed
            A list of the location of obstacles, of which travel on is prohibited

        Returns
        -------
        {
            'path': List(Tuples)
            'distance': int
        }
        '''

        total_path, total_distance = [], 0

        for i in range(len(node_list)-1):
            start_node, end_node = node_list[i], node_list[i+1]
            res = Pathfinder.get_path(start_node, end_node, obstacles)
            total_path.append(res['path'])
            total_distance += res['distance']

        return {'path': total_path, 'distance': total_distance}

    @staticmethod
    def shortest_path_to_n_points(start_node: Node, node_list: typing.List[Node], obstacles: list):
        '''	
        Given a unordered list of nodes, and a starting location, try to find the shortest path that will travel to all of the given nodes
        
        Parameters
        ----------
        start_node: Node
            The starting node from which the path will be started on the given
        node_list: List(Node)
            An unordered list of target nodes which will be visited. The order of visting is not guaranteed to be the same as the order in the list
        obstacles: List(Tuple or Node)
            A list of the location of obstacles, of which travel on is prohibited

        Returns
        -------
        {
            'path': List(Tuples)
            'distance': int
        }
        '''
        result = {}
        if len(node_list) == 0:
            raise ValueError('Cannot find path if there are no target nodes')

        if len(node_list) <= 5:
            temp_result = []
            all_paths = list(itertools.permutations(node_list))
            all_paths = [(start_node, *path) for path in all_paths]

            for path in all_paths:
                _ = Pathfinder.get_path_between_points(path, obstacles)
                temp_result.append((_['path'], _['distance']))

            path, distance = sorted(temp_result, key=lambda x: x[1])[0]
            return {'path': path, 'distance': distance}

        else:
            # todo implement approximate TSP
            return NotImplementedError
            # remaining_nodes = node_list[:]
            # closest_node, shortest_path, shortest_distance = None, [], float('inf')

            # for node in remaining_nodes:
            #     res = Pathfinder.get_path(start_node, node)
            #     if res['distance'] < shortest_distance:
            #         shortest_path = res['path']
            #         closest_node = node
            # remaining_nodes.remove(closest_node)

        return result

    @staticmethod
    def get_path(start_node: Node, end_node: Node, obstacles: typing.List[typing.Union[typing.Tuple, Node]] = []) -> typing.Dict[str, typing.Union[typing.List, int]]:
        """
        Calculates a path from a starting position to end position using the following heuristics:
            1. Try to find the longest 45 degree diagonal (eg. move in steps of (1,1)) 
            2. Once the diagonal is exhausted (in path from (1,3)->(5,6), and (4,6) has been reached), move in remaining x or y direction to the target
            
            3. If there is an obstacle in the calculated path, check the points adjacent and see if there is a path from those
            3.1: If there are no paths, try the points adjacent to those

        Parameters
        ----------
        start_node: Tuple(x,y) or Node 
            Starting location 
        end_node: Tuple(x,y) or Node 
            End location
        Obstacles: List(Nodes or Tuples) - mixed Tuple and Nodes are allowed
            Locations of all the obstacles in the space
        
        Returns
        -------
        {
            'path': List(Tuples)
            'distance': int
        }
        """

        memo = {}

        def _get_path(x_start: int, y_start: int, x_end: int, y_end: int, step_size: int = 1) -> typing.Dict[str, typing.Union[typing.List, int]]:
            """
            Internal get path function that does all the calculation between two points mentioned in the enclosing function
            """
            # memoise return results (ignore this if unsure, this doesn't affect the logic of the function)
            nonlocal memo
            memo_key = (x_start, y_start, x_end, y_end)
            if memo_key in memo:
                return memo[memo_key]

            # output of this function
            path, total_distance = [(x_start, y_start)], 0
            result = {'path': None, 'distance': None}

            # begin algorithm

            # determine the x and y length between the source and target
            dist_x, dist_y = x_end - x_start, y_end - y_start
            going_right, going_up = dist_x > 0, dist_y > 0

            dist_x, dist_y = abs(dist_x), abs(dist_y)

            # base case
            if dist_x == 0 and dist_y == 0:
                return result

            # determine diagonal length and path
            longest_diagonal_1d_length = min(dist_x, dist_y)

            # determine total absolute distance from source to target
            diagonal_distance = sqrt(2*(longest_diagonal_1d_length**2))
            straight_line_dist_remaining = max(dist_x, dist_y) - longest_diagonal_1d_length
            total_distance = diagonal_distance + straight_line_dist_remaining

            # find the (x,y) coordinates of each step along the diagonal
            diagonal_path = [(Pathfinder._change_val(x_start, i, going_right), Pathfinder._change_val(y_start, i, going_up))
                             for i in range(1, longest_diagonal_1d_length+1)]

            path.extend(diagonal_path)

            # starting from where the diagonal left off, traverse in one dimension to the target
            # if there is no possible diagonal, then start from starting position
            if longest_diagonal_1d_length != 0:
                current_x, current_y = diagonal_path[-1]
            else:
                current_x, current_y = x_start, y_start

            if dist_x < dist_y:  # diagonal limited by x
                path.extend([(current_x, Pathfinder._change_val(current_y, i, going_up))
                             for i in range(1, straight_line_dist_remaining+1)])
            else:
                path.extend([(Pathfinder._change_val(current_x, i, going_right), current_y)
                             for i in range(1, straight_line_dist_remaining+1)])

            # check if obstacls are in diagonal path
            for obstacle in obstacles:
                obstacle_x, obstacle_y = Pathfinder.extract_pos(obstacle)

                if (obstacle_x, obstacle_y) in path:
                    return None

            result['path'] = path
            result['distance'] = total_distance

            memo[memo_key] = result
            return result


        x_start, y_start = Pathfinder.extract_pos(start_node)
        x_end, y_end = Pathfinder.extract_pos(end_node)

        # try find uninterrupted path from start to target
        result = _get_path(x_start, y_start, x_end, y_end)
        if result: 
            return result
        else:
            # try travelling outward in any of the four directions, and see if there are valid paths there
            queue = []

            # save the coordinates of the immediate right, top, bottom and left nodes in a queue
            for (change_x, change_y) in [(1, 0), (0, 1), (0, -1), (-1, 0)]:
                new_starting_location = (x_start+change_x, y_start+change_y, x_end, y_end)
                queue.append(new_starting_location)

            # try running get_path from those locations, and if there is no valid path, try going more in that same direction
            while queue:
                x1, y1, x2, y2 = queue.pop(0)
                going_right, going_left = x1 > x_start, x1 < x_start
                going_up,    going_down = y1 > y_start, y1 < y_start

                if (x1, y1) in obstacles:
                    going_right, going_left, going_up, going_down = False, False, False, False
                    continue
                else:
                    result = _get_path(x1, y1, x2, y2)

                    if result: 
                        length_of_travel = max(abs(x1-x_start), abs(y1-y_start))
                        path_to_intermediate = [
                            (x_start+i, y_start) if going_right else
                            (x_start-i, y_start) if going_left else
                            (x_start, y_start+i) if going_up else
                            (x_start, y_start-i)
                            for i in range(1, length_of_travel+1)
                        ]
                        result['path'] = [*path_to_intermediate, *result['path']]
                        result['distance'] = result['distance'] + length_of_travel
                        return result
                    else:
                        new_starting_location = (
                            (x1+1, y1, x2, y2) if going_right else
                            (x1-1, y1, x2, y2) if going_left else
                            (x1, y1+1, x2, y2) if going_up else
                            (x1, y1-1, x2, y2)
                        )
                        queue.append(new_starting_location)

        return None

    @staticmethod
    def extract_pos(node: typing.Union[Node, typing.Tuple]):
        """ Gets the x and y coordinates of an object whose type could be either Node or a simple Tuple """
        if isinstance(node, Node):
            x_end, y_end = node.get_pos()
        elif isinstance(node, tuple):
            x_end, y_end = node[0], node[1]
        else:
            raise ValueError(
                'Object must be either a Node instance or a tuple of two values (x,y)')

        return x_end, y_end

    @staticmethod
    def _change_val(current: int, diff: int, increment: bool):
        """
        Helper function:
        eg. If going right, x-value needs to increment, otherwise it needs to decrement
        This function abstracts that to reduce the amount of code in the main function
        """
        return current + diff if increment else current - diff




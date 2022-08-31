from math import sqrt
from dataclasses import dataclass
import itertools
import math
import typing
import heapq

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


global_LOADER_VAL = 0
def progress_display(log_every:int=10000, total_count:int=None, logger_type=True):
    """Helper function to display a message every {log_every} iterations"""
    index = 0

    def display_updater():
        if logger_type:
            nonlocal index
            index+=1
            if index % log_every == 0:
                print(f'{index}           \r', end=' ') 
        else:
            global global_LOADER_VAL
            global_LOADER_VAL += 1
            if global_LOADER_VAL % 4001 == 0:
                print('/             \r', end=' ')
            elif global_LOADER_VAL % 7001 == 0:
                print('|             \r', end=' ')
            elif global_LOADER_VAL % 9001 == 0:
                print('\             \r', end=' ')
            elif global_LOADER_VAL % 10001 == 0:
                print('&             \r', end=' ')
            
    return display_updater


class Pathfinder:
    OBSTACLE_SIZE = (1,1)
    ROBOT_SIZE = (3,3)
    ARENA_SIZE = (19,19)

    @classmethod
    def get_path_between_points(cls, node_list: typing.List[Node], obstacles: list, update_callback=None, facing_direction_for_node_list: typing.List[str]=[]):
        '''	
        Returns a path using the internal Pathfinder get_path function that will travel between the given nodes, in the order they were given.
        
        Parameters
        ----------
        node_list: List(Nodes)
            The nodes to travel to, in the order of travel
        facing_direction_for_node_list: List(str)
            List of direction that the agent should be facing at each target node
        obstacles: List(Nodes or Tuples) - mixed Tuple and Nodes are allowed
            A list of the location of obstacles, of which travel on is prohibited

        Returns
        -------
        {
            'path': List of n List(Tuples), for n targets
            'distance': int
        }
        '''

        total_path, total_distance = [], 0
        if facing_direction_for_node_list:
            if len(facing_direction_for_node_list) == len(node_list):
                raise ValueError('List of facing directions should be one less than the list of target nodes, because there is no target direction at the first node')
        
        facing_direction_at_end = None

        for i in range(len(node_list)-1):
            start_node, end_node = node_list[i], node_list[i+1]
            if facing_direction_for_node_list:
                facing_direction_at_end = facing_direction_for_node_list[i]
            res = Pathfinder.get_path_between_two_points(start=start_node, target=end_node, obstacles=obstacles, update_callback=update_callback, direction_at_target=facing_direction_at_end)
            total_path.append(res['path'])
            total_distance += res['distance']

        return {'path': total_path, 'distance': total_distance}

    @classmethod
    def shortest_path_to_n_points(cls, start_node: Node, node_list: typing.List[Node], obstacles: list, approximate=True, update_callback=None, facing_direction_for_node_list: typing.List[str]=[]):
        '''	
        Given a unordered list of nodes, and a starting location, try to find the shortest path that will travel to all of the given nodes
        
        Parameters
        ----------
        start_node: Node
            The starting node from which the path will be started on the given
        node_list: List(Node)
            An unordered list of target nodes which will be visited. The order of visting is not guaranteed to be the same as the order in the list
        facing_direction_for_node_list: List(str)
            List of direction that the agent should be facing at each target node
        obstacles: List(Tuple or Node)
            A list of the location of obstacles, of which travel on is prohibited

        Returns
        -------
        {
            'path': List of n List(Tuples), for n targets
            'distance': int
        }
        '''
        if len(node_list) == 0:
            raise ValueError('Cannot find path if there are no target nodes')

        if len(node_list) <= 3 or not approximate:
            temp_result = []
            all_paths = list(itertools.permutations(node_list))
            all_paths = [(start_node, *path) for path in all_paths]
            for path in all_paths:
                _ = cls.get_path_between_points(node_list=path, obstacles=obstacles, update_callback=update_callback, facing_direction_for_node_list=facing_direction_for_node_list)
                temp_result.append((_['path'], _['distance']))

            path, distance = sorted(temp_result, key=lambda x: x[1])[0]
            return {'path': path, 'distance': distance}

        else:
            traversal_order = [start_node]
            facing_direction_in_traversal_order = []
            
            current_node = start_node
            node_list = node_list[:]
            while node_list:
                closest_node = sorted(node_list, key=lambda node:Pathfinder.h_function(*current_node, *node))[0]
                traversal_order.append(closest_node)
                if facing_direction_for_node_list:
                    facing_direction_in_traversal_order.append(facing_direction_for_node_list[node_list.index(closest_node)])
                node_list.remove(closest_node)
                current_node = closest_node

            print(f'{traversal_order=}')
            return cls.get_path_between_points(node_list=traversal_order, obstacles=obstacles, update_callback=update_callback, facing_direction_for_node_list=facing_direction_for_node_list)

            # remaining_nodes = node_list[:]
            # closest_node, shortest_path, shortest_distance = None, [], float('inf')

            # for node in remaining_nodes:
            #     res = Pathfinder.get_path(start_node, node)
            #     if res['distance'] < shortest_distance:
            #         shortest_path = res['path']
            #         closest_node = node
            # remaining_nodes.remove(closest_node)


    @classmethod
    def get_path_between_two_points(cls, start: Node, target: Node, obstacles: list,
        direction_at_target=None, update_callback=None, standoff_distance=0):
        '''	
        Use A* to calculate a path between two points
        
        Parameters
        ----------
        start: node or tuple (x,y)
        target: node or tuple(x,y)
        obstacles: List(node) or List(tuple x,y)
        direction_at_target: str Left, Right, Up, Down
            direction the agent should be facing at the target point
            Using this argument breaks the speed of A* since the costing function can no longer accurately determine the cost to the target
            because the optimal path might involve moving away from the target to orient itself to the right facing direction
        update_callback: function
            logging function to be called when path has been found
        standoff_distance: int
            one-dimensional distance from target where multiple paths to a node can be found
            (used to find paths with correction facing direction at target)
        
        Returns
        -------
        {'path': List(tuples)
            coords of each point on the path from start to target, 
        'distance': float
            total distance covered by the path taken
        }
        '''

        sx, sy = cls.extract_pos(start)
        ex, ey = cls.extract_pos(target)

        # heapqueue structure - total cost f, current location, current_direction, path to location
        queue = [(0,(sx,sy),'UP',[])]

        visited = {}

        # estimator cost function from location to destination
        h = Pathfinder.h_function

        # evaluator function for cost of path travelled so far
        # extension point for if turning logic changes
        # g = lambda path: sum(1 if i == moveset['UP'] or i == moveset['DOWN'] else 1.7 for i in path)
        # g = lambda path: len(path)
        g = lambda path: sum(Pathfinder.h_function(x,y,ex,ey) for (x,y) in path)

        moveset = {
            'UP':              (0,1), 
            'DOWN':            (0,-1), 
            'RIGHT_UP':        (1,1), 
            'RIGHT_DOWN':      (1,-1),
            'LEFT_UP':         (-1,1),
            'LEFT_DOWN':       (-1,-1),
        }

        logger = progress_display(logger_type=False)
        assert cls.ROBOT_SIZE == (3,3)
        assert cls.OBSTACLE_SIZE == (1,1)

        while queue:
            cost, (current_x, current_y), current_direction, path_to_current = heapq.heappop(queue)
            memo_key = (current_x, current_y)

            if memo_key in visited:
                continue
            
            next_moves =  [(current_x+dx,current_y+dy) for dx,dy in moveset.values()]
            for index,(x,y) in enumerate(next_moves):
                occupied_by_robot = [(x+dx,y+dy) for dx,dy in [(0,0), (1,0), (0,1), (-1,0), (0,-1),(1,1),(-1,-1), (-1,1),(1,-1)]]
                if any([i in obstacles for i in occupied_by_robot]) or cls.points_are_out_of_bounds(x, y):
                    continue

                next_cost = h(x, y, ex, ey) + cost
                path_to_next = [*path_to_current, (x,y)]
                next_direction = list(moveset.keys())[index]

                if 'RIGHT' in next_direction:
                    next_direction = 'RIGHT'
                elif 'LEFT' in next_direction:
                    next_direction = 'LEFT'

                if x == ex and y == ey:
                    if next_direction == direction_at_target or not direction_at_target:
                        if update_callback:
                            update_callback(f'{next_direction}')
                        return {'path': path_to_next, 'distance': next_cost}

                heapq.heappush(queue, (next_cost, (x, y), next_direction, path_to_next))

            if standoff_distance:
                if abs(current_x - ex) <= standoff_distance or abs(current_y - ey) <= standoff_distance:
                    pass
                else:
                    visited[memo_key] = True
            else:
                visited[memo_key] = True

            logger()
        return {'path': [], 'distance': float('inf')}


    @classmethod
    def points_are_out_of_bounds(cls, x,y):
        return x < 0 or y < 0 or x > cls.ARENA_SIZE[0] or y > cls.ARENA_SIZE[1]

    @classmethod 
    def find_facing_directions(cls, node_targets: list, node_obstacles: list):
        '''	
        Find the direction the agent should be facing for each node target
        
        
        Parameters
        ----------
        node_targets: List(nodes or tuples)
            list of target nodes to be visited, unordered
        node_obstacles: List(nodes or tuples)
            list of obstacles corresponding to the targets in node_targets
        Returns
        -------
            facing_direction_list: List(FacingDirections)
        '''
        # TODO facing directions should be a class variable
        FACING_DIRECTIONS ={
            (0, 1):  'UP',
            (0, -1): 'DOWN',
            (1, 0):  'RIGHT',
            (-1, 0): 'LEFT',
        }

        compare_return_at_most_abs_1 = lambda x1,x2: 0 if x1==x2 else 1 if x1-x2 > 0 else -1
        try:
            return [FACING_DIRECTIONS[(compare_return_at_most_abs_1(ox, tx), compare_return_at_most_abs_1(oy,ty))]
                for (tx, ty), (ox, oy) in zip(node_targets, node_obstacles)]
        except KeyError as e:
            raise ValueError('List of obstacles must be only one dimension (up,down,left or right) away from the target')

    @classmethod
    def h_function(cls,src_x,src_y,tgt_x,tgt_y):
        """estimator cost function from location to destination"""
        return math.sqrt((src_x-tgt_x)**2 + (src_y-tgt_y)**2)

    @classmethod
    def convert_path_to_instructions(cls, path):
        """Path consists of atomic left and right turns, but robot can only move in (1,1) moves 
        Function takes a path and converts to instructions usable by STM"""

        if not isinstance(path, list) or len(path) == 1:
            raise ValueError('path should in list format, and length of list should be greater than 1')
        
        if any(isinstance(i, list) for i in path):
            path = cls.flatten_output(path)
        
        instructions = [(x-path[index][0], y-path[index][1]) for index,(x,y) in enumerate(path[1:])] # index is the previous value, since we enumerate from path[1:]

        return instructions


    @classmethod
    def flatten_output(cls, listoflists):
        return [item for sublist in listoflists for item in sublist]
    
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

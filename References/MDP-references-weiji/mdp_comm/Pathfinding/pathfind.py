from math import sqrt
from dataclasses import dataclass
from enum import Enum
import math
import heapq

import vectors 

PRODUCTION_CONTINUE_ON_ERROR = False

# """ COMPLETED
# TODO
# change resolution to 1cm                                                                                                                               ✔️
#     moveset + atomic + all ranges that loop over points                                                                                                ✔️
#     all cost functions must increase by 10x                                                                           
#     - not sure if there are other functions to change                                                                                                  
# add 3pt turn + atomic moveset                                                                                                                          ✔️
#     displacement 10x10 and overall space 30x30                                                                                                         ✔️
#     move w facing too                                                                                                                                  ✔️
# extract occupied by robot function to own method                                                                                                       ✔️
# obstacle collision change to mathematic instead of coords in array                                                                                     ✔️
# refactor and fix find target axis                                                                                                                      ✔️
                                                                                                                                                       
# settle on how to determine pathfind to axis distance                                                                                                   ✔️
#     prev TODO - distance is not module-consistent right now -- A* functions add internal cost to right/left fwd/rvr turns to encourage straight lines  
#     final distance is not this internal distance                                                                                                       
#     other functions like pathfind to axis and reorient do not evaluate distance                                                                        
#     if distance is required then standardizing what distance to output                                                                                 
#     and providing a classmethod to evaluate distance based on moves might be required                                                                  
                                                                                                                                                       
#     just use cost?                                                                                                                                     
# restrict moveset when moving from axis to target                                                                                                       ✔️
#     prev TODO allow pathfinding to be limited to a moveset arg                                                                                         
                                                                                                                                                       
# add greedy pathfinding for directed pathfinding                                                                                                        
# """



"""
TODO
directed pathfind should be to rectangle in front of target instead of to point ✔️
refactor pathfinding to image to 
    find area in front of target where there is at least NxN space
    pathfind to target
    reorient


"""

# TODO calculate expected viewing radius along x for any y using tan(81.4) * Y * 2

@dataclass(eq=True, frozen=True)
class Node:
    x: int
    y: int

    def get_pos(self):
        return (self.x, self.y)


@dataclass(eq=True, frozen=True)
class Robot:
    CAMERA_FOV = 81.4
    valid_facings = ['N', 'S', 'E', 'W']

    moveset = {
        'FORWARD':         (0, 10),
        'REVERSE':         (0, -10),
        # 'RIGHT_FWD':       (32, 32),
        # 'RIGHT_RVR':       (32, -32),
        # 'LEFT_FWD':        (-32, 32),
        # 'LEFT_RVR':        (-32, -32),
        '3PT_RIGHT':       (0, 0),
        '3PT_LEFT':        (0, 0),
        '3PT_TURN_AROUND': (0, 0),
    }         
    
    moveset_i = {v:k for k,v in moveset.items()}    

    # define a set of smaller movements that the robot must move through to perform a single movement
    # all points should not collide with obstacles
    moveset_atomic = (lambda moveset: {
        'FORWARD':   [(0,10)],
        'REVERSE':   [(0,-10)],
        # 'RIGHT_FWD': [
        #     *[(0, i) for i in range(10, abs(moveset['RIGHT_FWD'][1])+1, 10)],
        #     *[(i, moveset['RIGHT_FWD'][1])
        #       for i in range(10, abs(moveset['RIGHT_FWD'][0])+1, 10)],
        #         ] + [moveset['RIGHT_FWD']],
        # 'RIGHT_RVR': [
        #     *[(0, -i) for i in range(10, abs(moveset['RIGHT_RVR'][1])+1, 10)],
        #     *[(i, moveset['RIGHT_RVR'][1])
        #       for i in range(10, abs(moveset['RIGHT_RVR'][0])+1, 10)],
        #         ] + [moveset['RIGHT_RVR']],
        # 'LEFT_FWD':  [
        #     *[(0, i) for i in range(10, abs(moveset['LEFT_FWD'][1])+1, 10)],
        #     *[(-i, moveset['LEFT_FWD'][1])
        #       for i in range(10, abs(moveset['LEFT_FWD'][0])+1, 10)],
        #         ] + [moveset['LEFT_FWD']],
        # 'LEFT_RVR':  [
        #     *[(0, -i) for i in range(10, abs(moveset['LEFT_RVR'][1])+1, 10)],
        #     *[(-i, moveset['LEFT_RVR'][1])
        #       for i in range(10, abs(moveset['LEFT_RVR'][0])+1, 10)],
        #         ] + [moveset['LEFT_RVR']],

        "3PT_RIGHT":  [(0, 0), (10, 0), (0, 10), (0, -4), (10, 10),
                        (10, -4), (14, 10), (14, 0), (14, -4)] + [moveset['3PT_RIGHT']],
        "3PT_LEFT":   [(0, 0), (0, 10), (-10, 0), (0, -4), (-10, -4),
                        (-10, 10), (-21, 0), (-14, -4), (-14, 10)] + [moveset['3PT_LEFT']],
        "3PT_TURN_AROUND": [(0, 0), (10, 0), (0, 10), (-10, 0), (0, -10), (10, 10), (-10, -10),
                        (-10, 10), (10, -10)] + [moveset['3PT_TURN_AROUND']],
    })(moveset)

    @classmethod
    def move_w_facing(cls, facing, command):
        if not (command in cls.moveset or command in cls.moveset.values()):
            raise ValueError(f'Command must be in the moveset of the robot')
        
        if not facing in cls.valid_facings:
            raise ValueError(f'{facing=} not in valid facings of agent')

        move_coords = None
        if isinstance(command, str):
            move_coords = cls.moveset[command]
        elif isinstance(command, tuple):
            move_coords = command
            command = cls.moveset_i[move_coords]
        
        points_to_keep_clear = []
        final_facing = None

        if facing == 'N':
            translate_coords = lambda x: x
            move_coords = translate_coords(move_coords)
            points_to_keep_clear = [translate_coords(point) for point in cls.moveset_atomic[command]]

            if command == 'FORWARD':
                final_facing = facing
            elif command == 'REVERSE':
                final_facing = facing
            elif command == 'RIGHT_FWD':
                final_facing = 'E'
            elif command == 'RIGHT_RVR':
                final_facing = 'W'
            elif command == 'LEFT_FWD':
                final_facing = 'W'
            elif command == 'LEFT_RVR':
                final_facing = 'E'
            elif command == '3PT_RIGHT':
                final_facing = 'E'
            elif command == '3PT_LEFT':
                final_facing = 'W'
            elif command == '3PT_TURN_AROUND':
                final_facing = 'S'

        if facing == 'S':
            translate_coords = lambda x: tuple_multiply(x, (-1,-1))
            move_coords = translate_coords(move_coords)
            points_to_keep_clear = [translate_coords(point) for point in cls.moveset_atomic[command]]

            if command == 'FORWARD':
                final_facing = facing
            elif command == 'REVERSE':
                final_facing = facing
            elif command == 'RIGHT_FWD':
                final_facing = 'W'
            elif command == 'RIGHT_RVR':
                final_facing = 'E'
            elif command == 'LEFT_FWD':
                final_facing = 'E'
            elif command == 'LEFT_RVR':
                final_facing = 'W'
            elif command == '3PT_RIGHT':
                final_facing = 'W'
            elif command == '3PT_LEFT':
                final_facing = 'E'
            elif command == '3PT_TURN_AROUND':
                final_facing = 'N'

        elif facing == 'E':
            translate_coords = lambda x: tuple_multiply(tuple_swap(x), (1,-1))
            move_coords = translate_coords(move_coords)
            points_to_keep_clear = [translate_coords(point) for point in cls.moveset_atomic[command]]

            if command == 'FORWARD':
                final_facing = facing
            elif command == 'REVERSE':
                final_facing = facing
            elif command == 'RIGHT_FWD':
                final_facing = 'S'
            elif command == 'RIGHT_RVR':
                final_facing = 'N'
            elif command == 'LEFT_FWD':
                final_facing = 'N'
            elif command == 'LEFT_RVR':
                final_facing = 'S'
            elif command == '3PT_RIGHT':
                final_facing = 'S'
            elif command == '3PT_LEFT':
                final_facing = 'N'
            elif command == '3PT_TURN_AROUND':
                final_facing = 'W'

        elif facing == 'W':
            translate_coords = lambda x: tuple_multiply(tuple_swap(x), (-1, 1))
            move_coords = translate_coords(move_coords)
            points_to_keep_clear = [translate_coords(point) for point in cls.moveset_atomic[command]]

            if command == 'FORWARD':
                final_facing = facing
            elif command == 'REVERSE':
                final_facing = facing
            elif command == 'RIGHT_FWD':
                final_facing = 'N'
            elif command == 'RIGHT_RVR':
                final_facing = 'S'
            elif command == 'LEFT_FWD':
                final_facing = 'S'
            elif command == 'LEFT_RVR':
                final_facing = 'N'
            elif command == '3PT_RIGHT':
                final_facing = 'N'
            elif command == '3PT_LEFT':
                final_facing = 'S'
            elif command == '3PT_TURN_AROUND':
                final_facing = 'E'
                
        return move_coords, final_facing, points_to_keep_clear


class Pathfinder:
    agent = Robot
    moveset = agent.moveset

    OBSTACLE_SIZE = (10, 10)
    ROBOT_SIZE = (30, 30)
    ARENA_SIZE = [(5,5), (195, 195)]
    ROBOT_TGT_DIST_FROM_IMG = 30
    ROBOT_DISTANCE_FROM_OBS = 10
    MIN_AXIS_LENGTH = 10

    @classmethod
    def shortest_path_between_points_strategy(cls, start, targets, obstacle_faces, obstacles, starting_face='N',
        algorithm=None):

        algorithm = algorithm or cls.get_path_directed

        result = {'pathfinding':None, 'traversal_order': None}
        # shortest path between n points

        if not len(targets) == len(obstacle_faces):
            raise ValueError("Number of obstacle faces and targets must match")

        current_node = start
        target_list, obstacle_faces_list = targets[:], obstacle_faces[:]

        coord_traversal_order = []
        traversal_order = []
        obstacle_face_order = []

        while target_list:
            closest_node = sorted(target_list, key=lambda node:Pathfinder.h_function(*current_node, *node))[0]
            node_index = target_list.index(closest_node)
            corresponding_face = obstacle_faces_list[node_index]

            coord_traversal_order.append(closest_node)
            original_index_of_target = targets.index(closest_node)
            traversal_order.append(original_index_of_target)

            obstacle_face_order.append(corresponding_face)
            current_node = closest_node
    
            target_list.pop(node_index)
            obstacle_faces_list.pop(node_index)
        
        result['traversal_order'] = traversal_order
        result['pathfinding'] = algorithm(start=start, targets=coord_traversal_order, obstacle_faces=obstacle_face_order, 
                                            obstacles=obstacles, starting_face=starting_face)

        return result

    @classmethod
    def get_path_directed(cls, start, targets, obstacle_faces, obstacles, starting_face='N'):
        """"""
        output = {'path': [], 'distance': 0, 'final_facing': None, 'moves':[]}

        LEEWAY_AROUND_TARGET = 5

        if not len(targets) == len(obstacle_faces):
            raise ValueError("Number of obstacle faces and targets must match")

        cx, cy = start
        facing = starting_face

        for i in range(len(targets)):
            path_to_target = []
            moves_to_target = []
            distance_to_target = 0

            target, obstacle_face = targets[i], obstacle_faces[i]

            # get axis in front of target
            possible_target_axes = Pathfinder.generate_possible_target_axes(target, obstacle_face, obstacles)
            if not possible_target_axes:
                print(f'No target axes found for target {target=}')
                if PRODUCTION_CONTINUE_ON_ERROR:
                    continue
                else:
                    raise RuntimeError(f'No target axes found for target {target=}')
            
            # take only axis containing target 
            if isinstance(possible_target_axes, list) and len(possible_target_axes) > 1:
                for axis in possible_target_axes:
                    if point_within_straight_line(target, *axis):
                        possible_target_axes.remove(axis)
                        possible_target_axes.insert(0, axis)
                        break
                axis_with_target = possible_target_axes[0]
            
            else:   
                axis_with_target = possible_target_axes[0]
            facing_at_axis = Pathfinder._opposite_direction(obstacle_face)
            
            points_on_axis_by_closest_to_target = sorted(axis_with_target, key=lambda pt: cls.h_function(*pt, *target))
            reorient_result = None
            
            closest_point, furthest_point = points_on_axis_by_closest_to_target
            point_to_pathfind_to = closest_point
            # try all points to find one that has enough space for reorient
            while point_to_pathfind_to != furthest_point:
                to_point_result = cls.find_path_to_point((cx,cy), point_to_pathfind_to, facing, obstacles, final_point_leweway=LEEWAY_AROUND_TARGET)
                if not to_point_result:
                    point_to_pathfind_to = cls.move_one(point_to_pathfind_to, obstacle_face, step=10)
                    continue
                facing_at_point = to_point_result['final_facing']
                coords_at_point = to_point_result['path'][-1]
                
                # execute reorient
                reorient_result = cls.reorient(coords_at_point, facing_at_point, facing_at_axis, obstacles)
                if reorient_result:
                    # print(f'{to_point_result=}')
                    # print(f'{reorient_result=}')

                    reorient_facing = reorient_result['final_facing']
                    reorient_path = reorient_result['path']
                    reorient_moves = reorient_result['moves']
                    reorient_distance = reorient_result['distance']
                    
                    path_to_target     = [*to_point_result['path'], *reorient_path]
                    moves_to_target    = [*to_point_result['moves'], *reorient_moves]
                    distance_to_target = to_point_result['distance'] + reorient_distance

                    output['path'].append(path_to_target)
                    output['moves'].append(moves_to_target)

                    output['distance'] = output['distance'] + distance_to_target
                    output['final_facing'] = reorient_facing
                    facing = reorient_facing
                    cx, cy = path_to_target[-1]

                    break

                else:
                    point_to_pathfind_to = cls.move_one(point_to_pathfind_to, obstacle_face, step=10)
            
            if not reorient_result:
                print(f'Could not find point on target axis {axis_with_target} to reorient in')
                if PRODUCTION_CONTINUE_ON_ERROR:
                    continue
                else:
                    raise RuntimeError
        return output

    # @classmethod
    # def get_path_betweeen_points_directed(cls, start, targets, obstacle_faces, obstacles, starting_face='N'):
    #     output = {'path': [], 'distance': 0, 'final_facing': None, 'moves':[]}

    #     if not len(targets) == len(obstacle_faces):
    #         raise ValueError("Number of obstacle faces and targets must match")

    #     for i in range(len(targets)):
    #         total_path = []
    #         total_moves = []
    #         total_distance = 0

    #         target, obstacle_face = targets[i], obstacle_faces[i]
    #         possible_target_axes = Pathfinder.generate_possible_target_axes(target, obstacle_face, obstacles)

    #         facing_at_axis = Pathfinder._opposite_direction(obstacle_face)
    #         result = Pathfinder.pathfind_to_axis_and_reorient(start, possible_target_axes, starting_face, facing_at_axis, obstacles, target)
    #         total_path, total_moves, facing, total_distance = result['path'], result['moves'], result['final_facing'], result['distance']

    #         print(f'{possible_target_axes=}')
    #         print(f'intermediate {total_path=}')        
    #         print(f'intermediate {total_moves=}')

    #         point_on_axis = total_path[-1]
    #         allowed_moves_along_axis = [
    #             'FORWARD',
    #             'REVERSE',
    #             'RIGHT_FWD',
    #             'LEFT_FWD',
    #             '3PT_RIGHT',
    #             '3PT_LEFT',
    #             '3PT_TURN_AROUND',
    #         ]

    #         px,py = point_on_axis
    #         tx,ty = target
    #         if not point_within_rect(tx, ty, 5, 5, px, py):
    #             along_axis_result = Pathfinder.find_path_to_point(point_on_axis, target, facing, obstacles, final_point_leweway=5)
    #             path, moves, facing, distance = along_axis_result['path'], along_axis_result['moves'], along_axis_result['final_facing'], along_axis_result['distance']

    #             total_path = [*total_path, *path]
    #             total_moves = [*total_moves, *moves]
    #             total_distance += distance

    #         output['path'].append(total_path)
    #         output['moves'].append(total_moves)
    #         output['final_facing'] = facing
    #         output['distance'] += total_distance


    #         path_faces = Pathfinder.determine_all_faces_on_path(starting_face, moves)
            
    #         starting_face = facing
    #         start = total_path[-1]
    #         print(f'{total_path=}')
    #         print(f'next {start=}')


    #     return output

    # @classmethod
    # def pathfind_to_axis_and_reorient(cls, start, possible_target_axes: list[tuple[tuple[int, int], tuple[int, int]]],
    #                                   starting_face, final_facing, obstacles: list[tuple], target,  min_axis_length=4):
    #     '''	
    #     Returns
    #     -------
    #         result = {'path':list, 'moves':list, 'final_facing': str}
    #     '''
    #     if not isinstance(possible_target_axes, list):
    #         raise ValueError("Arg must be a list of possible target axes")
    #     if not len(possible_target_axes[0]) == 2:
    #         raise ValueError(
    #             "Target axis must have one start and one end point")
    #     if not len(possible_target_axes[0][0]) == 2:
    #         raise ValueError('Coordinate of a point must contain only x,y value')

    #     result = {'path': [], 'moves': [], 'final_facing': None, 'distance': 0}

    #     # TODO base case if point is already in target axis
    #     fixed_x = final_facing in ['N', 'S']

    #     # if not any(point_within_straight_line(start, *axis) for axis in possible_target_axes):
    #     # perform pathfinding to linear_target
    #     # pass

    #     # problem with refactoring reorient outside pathfinding to linear target -
    #     # if reorient on current point is not possible due to obs, then function fails
    #     # original logic is to try pathfinding to other points
    #     # maybe fixed by making pathfind to linear target return current pos and no moves

    #     # push axis with target inside as first try
    #     for axis in possible_target_axes:
    #         if point_within_straight_line(target, *axis):
    #             possible_target_axes.remove(axis)
    #             possible_target_axes.insert(0, axis)
    #             break

    #     # try all the possible target axes, and find one where the reorientation can be done
    #     for axis_start, axis_end in possible_target_axes:
    #         result_to_axis = cls.find_path_to_linear_target(start, axis_start, axis_end, starting_face, obstacles)

    #         path = result_to_axis['path']
    #         facing_at_axis = result_to_axis['final_facing']
    #         end_point = path[-1]
    #         result_at_reorient = cls.reorient(end_point, facing_at_axis, final_facing, obstacles)

    #         if not result_at_reorient:
    #             sx, sy = axis_start
    #             ex, ey = axis_end

    #             #todo have to refactor possible other points to take into account new 1cm resolution
    #             if fixed_x:
    #                 possible_other_points = [(sx, y) for y in range(sy, ey+1, 10) if (sx, y) != end_point]
    #             else:
    #                 possible_other_points = [(x, sy) for x in range(sx, ex+1, 10) if (x, sy) != end_point]

    #             for point in possible_other_points:
    #                 result_to_axis = cls.find_path_to_point(start, point, starting_face, obstacles, final_point_leweway=5)

    #                 path = result_to_axis['path']
    #                 facing_at_axis = result_to_axis['final_facing']
    #                 end_point = path[-1]
    #                 result_at_reorient = cls.reorient(end_point, facing_at_axis, final_facing, obstacles)

    #                 if result_at_reorient:
    #                     break

    #         if result_at_reorient:
    #             result['path'] = [*result_to_axis['path'], *result_at_reorient['path']]
    #             result['moves'] = [*result_to_axis['moves'], *result_at_reorient['moves']]
    #             result['final_facing'] = result_at_reorient['final_facing']
    #             result['distance'] = result_at_reorient['distance'] + result_to_axis['distance']
    #             return result

    @classmethod
    def find_path_to_point(cls, start, target, starting_face='N', obstacles=None,
                            update_callback=None, moveset=None, final_point_leweway=5):
        '''	
        Find a reasonable path from starting location to destination location
        Pathfinding algorothm is A*

        Takes into account the direction the agent is facing, and the possible moves it can make
        Final facing direction can be any direction

        Returns
        -------
        path: {list(tuples)}
        distance: float
        moves: list(str)
        final_facing: str
        '''
        result = {'path':[], 'distance':0, 'moves':[], 'final_facing':None}

        assert cls.ROBOT_SIZE == (30, 30)
        assert cls.OBSTACLE_SIZE == (10, 10)
        assert cls.moveset
        
        if not obstacles:
            obstacles = []

        moveset = moveset or cls.moveset
        if not all(move in cls.moveset for move in moveset):
            raise ValueError("Moves passed as arg must be in agent's moveset")

        tx, ty = cls.extract_pos(target)

        if start == target:
            return result

        # heapqueue structure - total cost f, current location, path to target, move instructions to target
        queue = [(0, start, starting_face, [], [])]
        visited_nodes = set()

        # estimator cost function from location to destination
        def h(argx,argy):
            return Pathfinder.h_function(argx,argy,tx,ty)

        while queue:
            cost, current_node, facing_direction, path_to_current, movetypes_to_current = heapq.heappop(queue)
            (current_x, current_y) = current_node

            visited_key = (current_node, facing_direction)
            visited_nodes.add(visited_key)

            for movetype in cls.moveset:
                (dx, dy), final_facing, atomic_moves = cls.agent.move_w_facing(facing_direction, movetype) # delta-x and y from a possible movement by the robot
                final_x, final_y = (current_x+dx, current_y+dy) # final position after the move

                neighbour_visited_key = ((final_x, final_y), final_facing)
                if neighbour_visited_key in visited_nodes:
                    continue

                path_to_keep_clear = cls._generate_points_to_keep_clear_on_turn(current_x, current_y, atomic_moves)
                if not cls.check_all_points_on_path_valid(path_to_keep_clear, obstacles):
                    continue
                
                path_to_next = [*path_to_current, (final_x, final_y)]
                movetypes_to_next = [*movetypes_to_current, movetype]

                next_cost = h(final_x, final_y) + cost

                if movetype in ['RIGHT_FWD', 'LEFT_FWD']:
                    next_cost += 1.8
                elif movetype in ['RIGHT_RVR', 'LEFT_RVR']:
                    next_cost += 2.2
                elif movetype in ['REVERSE']:
                    next_cost += 0.5

                elif movetype in ['3PT_RIGHT', '3PT_LEFT']:
                    next_cost += 2
                elif movetype == '3PT_TURN_AROUND':
                    next_cost += 2

                if point_within_rect(tx, ty, final_point_leweway, final_point_leweway, final_x, final_y):
                # if (final_x == tx and final_y == ty):
                    if update_callback:
                        update_callback(f'target point {tx,ty}')
                    result['path'] = path_to_next
                    result['distance'] = next_cost
                    result['moves'] = movetypes_to_next
                    result['final_facing'] = final_facing
                    return result

                heapq.heappush(queue, (next_cost, (final_x, final_y), final_facing, path_to_next, movetypes_to_next))

    @classmethod
    def find_path_to_linear_target(cls, start, axis_start, axis_end, starting_face,
        obstacles):
        '''	
        Find a reasonable path from a starting position to a destination which is a line, which is defined as the points inclusive between axis_start and axis_end
        Pathfinding algorithm is A* with heuristic = distance from point to line 

        Takes into account the direction the agent is facing, and the possible moves it can make
        Final facing direction can be any direction

        result = {'path':None, 'distance':None, 'moves':None, 'final_facing':None}
        '''
        if not obstacles:
            obstacles = []

        result = {'path':[], 'distance':0, 'moves':[], 'final_facing':starting_face}

        tx1, ty1 = cls.extract_pos(axis_start)
        tx2, ty2 = cls.extract_pos(axis_end)

        if not ((tx1 == tx2) or (ty1 == ty2)):
            raise ValueError('Axis must be one dimensional')
        
        if point_within_straight_line(start, axis_start, axis_end):
            return result
        
        distance_heuristic = lambda x, y: cls._path_to_line((x,y), (tx1,ty1), (tx2,ty2))[0] # returns distance as first arg and closest point as second arg

        queue = [(0, start, starting_face, [], [])]
        visited_nodes = set()

        while queue:
            cost, current_node, facing_direction, path_to_current, movetypes_to_current = heapq.heappop(queue)
            (current_x, current_y) = current_node
            visited_key = (current_node, facing_direction)
            visited_nodes.add(visited_key)

            for movetype in cls.moveset:   
                (dx,dy), final_facing, atomic_moves = cls.agent.move_w_facing(facing_direction, movetype)  # delta-x and y from a possible movement by the robot
                x, y = (current_x+dx, current_y+dy) # final position after the move
                
                neighbour_visited_key = ((x, y), final_facing)
                if neighbour_visited_key in visited_nodes:
                    continue

                path_to_keep_clear = cls._generate_points_to_keep_clear_on_turn(current_x, current_y, atomic_moves)
                if not cls.check_all_points_on_path_valid(path_to_keep_clear, obstacles):
                    continue
                    
                path_to_next = [*path_to_current, (x, y)]
                movetypes_to_next = [*movetypes_to_current, movetype]

                next_cost = distance_heuristic(x,y) + cost
                
                if movetype in ['RIGHT_FWD', 'LEFT_FWD']:
                    next_cost += 1.5
                elif movetype in ['RIGHT_RVR','LEFT_RVR']:
                    next_cost += 2
                elif movetype in ['REVERSE']:
                    next_cost += 0.5

                elif movetype in ['3PT_RIGHT', '3PT_LEFT']:
                    next_cost += 2
                elif movetype == '3PT_TURN_AROUND':
                    next_cost += 2

                is_complete = False
                same_x = tx1 == tx2
                if same_x:
                    d = abs(ty1 - ty2) / 2
                    ty_mid = max(ty1, ty2) - d
                    if point_within_rect(tx1, ty_mid, 5, d, x, y):
                        is_complete = True
                else:
                    d = abs(tx1 - tx2) / 2
                    tx_mid = max(tx1, tx2) - d
                    if point_within_rect(tx_mid, ty1, d, 5, x, y):
                        is_complete = True
                
                # if (tx1 == tx2 and x == tx1 and y in range(ty1, ty2+1)) or \
                #     (ty1 == ty2 and y == ty1 and x in range(tx1, tx2+1)):
                if is_complete:
                    result['path'] = path_to_next
                    result['distance'] = next_cost
                    result['moves'] = movetypes_to_next
                    result['final_facing'] = final_facing
                    return result

                heapq.heappush(
                    queue, (next_cost, (x, y), final_facing, path_to_next, movetypes_to_next))
    
    @classmethod
    def generate_photo_taking_points(cls, obstacles, faces):
        '''	
        Generate points that are X distance away from the obstacles/image face, at which we will take a photo of the target
        
        Parameters
        ----------
        obstacles: tuple(x,y)
            Obstacle locations, where the obstacles contain a face which the images are on
        faces: str N/S/E/W
            The face of the obstacle on which the image is on
        
        Returns
        -------
        list[tuple(x,y)]
            The list of target points the agent should navigate to    
        '''

        targets = []
        
        if not all(i in ['N','S','E','W'] for i in faces):
            raise ValueError('All faces must be in N/S/E/W format')
        if not len(obstacles) == len(faces):
            raise ValueError('Number of obstacles must be equal to number of faces')
        
        for index, obstacle_coords in enumerate(obstacles):
            img_direction = faces[index]
            targets.append(cls.move_one(obstacle_coords, img_direction, cls.ROBOT_TGT_DIST_FROM_IMG))
        return targets
        
    @classmethod
    def generate_possible_target_axes(cls, target, obstacle_face:str, obstacles,
                             min_axis_length=None):
        '''	
        Generate a line X distance away from the image face of the obstacle
        The agent will route to this line, before aligning to the target
        
        Parameters
        ----------
        target: tuple(x,y)
            The point which the agent will stop at to attempt image recognition
        obstacle_face: str N/S/E/W
            The face of the obstacle on which the image is on
        obstacles: list ( tuple(x,y) )
            The coordinates of all other obstacles which the agent cannot intrude onto
        
        Returns
        -------
        ( tuple(tuple(x,y), tuple(x,y)) )
            list of possible axes, with each item in the list being a tuple of start and end points
        '''
        if obstacle_face not in ['N', 'S', 'E', 'W']:
            raise ValueError('obstacle_face must be N/S/E/W')

        min_axis_length = min_axis_length or cls.MIN_AXIS_LENGTH

        boundary, obstacles_in_axis = cls._get_boundary_and_obstacles_in_line(target, obstacle_face, obstacles)
        valid_points_in_axis = [target, boundary]
        
        for i in obstacles_in_axis:
            valid_points_in_axis.append(cls.move_one(i, obstacle_face, max(cls.OBSTACLE_SIZE)))
            valid_points_in_axis.append(cls.move_one(i, cls._opposite_direction(obstacle_face), max(cls.OBSTACLE_SIZE)))

        fixed_x = obstacle_face in ['N', 'S']
        points_in_axis = sorted(valid_points_in_axis, key=lambda x: x[1] if fixed_x else x[0])

        possible_axes = [
            (points_in_axis[i], points_in_axis[i+1]) for i in range(len(points_in_axis)-1)
            ]

        # filter away possible axes that have length less than min 
        # assert min_axis_length > max(cls.OBSTACLE_SIZE), "add new filter to remove obstacles between points, otherwise possible axis will intersect obstacle"
        possible_axes = [line for line in possible_axes if not any(point_within_straight_line(obs, *line) for obs in obstacles)]

        if fixed_x:
            possible_axes = [line for line in possible_axes if abs(line[0][1]-line[1][1]) >= min_axis_length ]
        else:
            possible_axes = [line for line in possible_axes if abs(line[0][0]-line[1][0]) >= min_axis_length ]

        return possible_axes

    @classmethod
    def _get_boundary_and_obstacles_in_line(cls, target, obstacle_face, obstacles):
        """ internal function used by generate_target_axis"""
        tx, ty = target
        min_boundary_x, min_boundary_y = cls.ARENA_SIZE[0]
        max_boundary_x, max_boundary_y = cls.ARENA_SIZE[1]
        
        if obstacle_face == 'N':
            boundary = (tx, max_boundary_y-10)
            obstacles_in_axis = [(ox,oy) for (ox,oy) in obstacles if (ox == tx and ty < oy <= max_boundary_y)]
        elif obstacle_face == 'S':
            boundary = (tx, min_boundary_y)
            obstacles_in_axis = [(ox,oy) for (ox,oy) in obstacles if (ox == tx and 0 <= oy < ty)]
        elif obstacle_face == 'E':
            boundary = (max_boundary_x-10, ty)
            obstacles_in_axis = [(ox,oy) for (ox,oy) in obstacles if (tx < ox <= max_boundary_x and oy == ty)]
        elif obstacle_face == 'W':
            boundary = (min_boundary_x, ty)
            obstacles_in_axis = [(ox,oy) for (ox,oy) in obstacles if (0 <= ox < tx and oy == ty)]
        else:
            raise ValueError("Unknown direction")

        return boundary, obstacles_in_axis

    @classmethod
    def reorient(cls, current_coords, current_facing, final_facing, obstacles):
        """Get a path that reorients the agent in a new direction on the same point
        ie. find a path with the same starting and ending location that has a final facing same as the given arg
        result = {'final_facing': str, 'path': [], 'moves': [], distance: int}
        """
        result = {'final_facing': None, 'path': [], 'moves':[], 'distance': 0}

        agent = cls.agent
        valid_facings = agent.valid_facings

        if current_facing not in valid_facings:
            raise ValueError(f'{current_facing=} is not valid in valid facings of agent')
        if final_facing not in valid_facings:
            raise ValueError(f'{final_facing=} is not valid in valid facings of agent')

        def _general_reorient(moves):
            nonlocal result
            facing = current_facing
            coords = current_coords
            path = []

            cx,cy = coords
            for move in moves:
                (dx,dy), new_facing, atomic_moves = agent.move_w_facing(facing, move)

                path_to_keep_clear = cls._generate_points_to_keep_clear_on_turn(cx, cy, atomic_moves)
                if not cls.check_all_points_on_path_valid(path_to_keep_clear, obstacles):
                    return None

                cx,cy = cx+dx, cy+dy
                facing = new_facing
                path.append((cx,cy))

            result['path'] = path
            result['moves'] = moves
            result['final_facing'] = final_facing
            result['distance'] = 0
            return result

        def first_truthy(func, *args):
            for arg in args:
                result = func(arg)
                if result:
                    return result
            return None

            
        if current_facing == final_facing:
            result['final_facing'] = current_facing
            return result

        # assert agent.turn_radius == 2, 'All hardcoded turns are only applicable for this turning radius'
        # TURN_RADIUS = agent.turn_radius

        AXIS_REVERSE_0 = ['3PT_TURN_AROUND']
        # AXIS_REVERSE_1 = ['RIGHT_RVR', 'LEFT_FWD'] 
        # AXIS_REVERSE_2 = ['LEFT_RVR', 'RIGHT_FWD']

        AXIS_RIGHT_TURN_0 = ['3PT_RIGHT']
        # AXIS_RIGHT_TURN_1 = ['REVERSE'] * TURN_RADIUS + ['RIGHT_FWD']
        # AXIS_RIGHT_TURN_2 = ['FORWARD'] * TURN_RADIUS + ['LEFT_RVR']

        AXIS_LEFT_TURN_0 = ['3PT_LEFT']
        # AXIS_LEFT_TURN_1 = ['REVERSE'] * TURN_RADIUS + ['LEFT_FWD']
        # AXIS_LEFT_TURN_2 = ['FORWARD'] * TURN_RADIUS + ['RIGHT_RVR']

        
        if current_facing == 'N':
            if final_facing == 'S':
                return first_truthy(_general_reorient, 
                        AXIS_REVERSE_0,
                        # AXIS_REVERSE_1,
                        # AXIS_REVERSE_2,
                )
            elif final_facing == 'E':
                return first_truthy(_general_reorient, 
                        AXIS_RIGHT_TURN_0,
                        # AXIS_RIGHT_TURN_1,
                        # AXIS_RIGHT_TURN_2,
                )
            elif final_facing == 'W':
                return first_truthy(_general_reorient, 
                        AXIS_LEFT_TURN_0,
                        # AXIS_LEFT_TURN_1,
                        # AXIS_LEFT_TURN_2,
                )

        elif current_facing == 'S':
            if final_facing == 'N':
                return first_truthy(_general_reorient, 
                        AXIS_REVERSE_0,
                        # AXIS_REVERSE_1,
                        # AXIS_REVERSE_2,
                )
            elif final_facing == 'E':
                return first_truthy(_general_reorient, 
                        AXIS_LEFT_TURN_0,
                        # AXIS_LEFT_TURN_1,
                        # AXIS_LEFT_TURN_2,
                )
            elif final_facing == 'W':
                return first_truthy(_general_reorient, 
                        AXIS_RIGHT_TURN_0,
                        # AXIS_RIGHT_TURN_1,
                        # AXIS_RIGHT_TURN_2,
                )
        
        elif current_facing == 'E':
            if final_facing == 'N':
                return first_truthy(_general_reorient, 
                        AXIS_LEFT_TURN_0,
                        # AXIS_LEFT_TURN_1,
                        # AXIS_LEFT_TURN_2,
                )
            elif final_facing == 'W':
                return first_truthy(_general_reorient, 
                        AXIS_REVERSE_0,
                        # AXIS_REVERSE_1,
                        # AXIS_REVERSE_2,
                )
            elif final_facing == 'S':
                return first_truthy(_general_reorient, 
                        AXIS_RIGHT_TURN_0,
                        # AXIS_RIGHT_TURN_1,
                        # AXIS_RIGHT_TURN_2,
                )

        elif current_facing == 'W':
            if final_facing == 'N':
                return first_truthy(_general_reorient, 
                        AXIS_RIGHT_TURN_0,
                        # AXIS_RIGHT_TURN_1,
                        # AXIS_RIGHT_TURN_2,
                )
            elif final_facing == 'S':
                return first_truthy(_general_reorient, 
                        AXIS_LEFT_TURN_0,
                        # AXIS_LEFT_TURN_1,
                        # AXIS_LEFT_TURN_2,
                )
            elif final_facing == 'E':
                return first_truthy(_general_reorient, 
                        AXIS_REVERSE_0,
                        # AXIS_REVERSE_1,
                        # AXIS_REVERSE_2,
                )
        
        assert False, 'Function should return a value before this point'

    @classmethod
    def check_all_points_on_path_valid(cls, coord_list: list, obstacles):
        if isinstance(coord_list, tuple):
            coord_list = [coord_list]
        if isinstance(obstacles, tuple):
            obstacles = [obstacles]

        if not all(isinstance(obs_coord, tuple) for obs_coord in obstacles):
            raise ValueError("Obstacle list should be list of coordinates")
        if not all(isinstance(coords, tuple) for coords in coord_list):
            raise ValueError("Path coord list should be list of coordinates")

        for x,y in coord_list:
            if cls.points_are_out_of_bounds(x,y):
                return False
            
            for (ox, oy) in obstacles:
                if point_within_rect(x, y, cls.ROBOT_DISTANCE_FROM_OBS+5, cls.ROBOT_DISTANCE_FROM_OBS+5, ox, oy):
                    return False

        return True

    @classmethod
    def _generate_points_to_keep_clear_on_turn(cls, start_x, start_y, atomic_moves:list):
        if not isinstance(atomic_moves, list):
            raise ValueError('atomic moves must be list')

        result = []
        x,y = start_x, start_y
        for (dx,dy) in atomic_moves:
            fx,fy = x+dx, y+dy
            result.append((fx,fy))
        return result

    @classmethod
    def find_images_in_view_arc(cls, current_coords, current_facing, obstaces_with_images, obstacle_image_face, other_obstacles):
        # filter out images that have face != opp current facing
        # of these images apply view at distance formula to determine if they are in viewing arc
            # viewing arc = tan(fov) * forward 1-d distance * 2

        # filter out images that are not in viewing arc
        # generate occlusion 

        pass
    

    @classmethod
    def points_are_out_of_bounds(cls, x, y):
        return x < cls.ARENA_SIZE[0][0] or y < cls.ARENA_SIZE[0][1] or x > cls.ARENA_SIZE[1][0] or y > cls.ARENA_SIZE[1][1]

    @classmethod
    def h_function(cls, src_x, src_y, tgt_x, tgt_y):
        """estimator cost function from location to destination"""
        return math.sqrt((src_x-tgt_x)**2 + (src_y-tgt_y)**2)

    @classmethod
    def _path_to_line(cls, pnt, start, end):
        line_vec = vectors.vector(start, end)
        pnt_vec = vectors.vector(start, pnt)
        line_len = vectors.length(line_vec)
        line_unitvec = vectors.unit(line_vec)
        pnt_vec_scaled = vectors.scale(pnt_vec, 1.0/line_len)
        t = vectors.dot(line_unitvec, pnt_vec_scaled)
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
        nearest = vectors.scale(line_vec, t)
        dist = vectors.distance(nearest, pnt_vec)
        nearest = vectors.add(nearest, start)
        return (dist, nearest)

    @classmethod
    def determine_all_faces_on_path(cls, initial_facing, move_instructions):
        """Determine the facing direction of the agent for every given move"""
        try:
            if not (isinstance(move_instructions, list) and all(isinstance(i, str) for i in move_instructions)):
                raise ValueError('move instructions must be a list of string instructions')
        except IndexError as E:
            raise ValueError('move instructions must be a list of string instructions')
        
        if any(i not in cls.moveset for i in move_instructions):
            raise ValueError('move instructions do not correspond to available moves by agent')

        result = []        
        facing = initial_facing
        for i in move_instructions:
            facing = cls.agent.move_w_facing(facing, i)[1]
            result.append(facing)
        return result

    @classmethod
    def determine_final_facing(cls, initial_facing, move_instructions):
        """Determine the final facing of the agent given an initial facing and a list of move instructions"""
        return cls.determine_all_faces_on_path(initial_facing, move_instructions)[-1]

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
    def move_one(cls, coords, direction, step=10):
        """Utility function to get the future coords based on some current coordinates and the direction to move in"""
        cx,cy = coords
        if direction == 'N':
            return (cx,cy+step)
        elif direction == 'S':
            return (cx,cy-step)
        elif direction == 'E':
            return (cx+step,cy)
        elif direction == 'W':
            return (cx-step,cy)
        else:
            raise ValueError("Unknown direction")

    @classmethod
    def _opposite_direction(cls, direction):
        if direction == 'N':
            return 'S'
        elif direction == 'S':
            return 'N'
        elif direction == 'E':
            return 'W'
        elif direction == 'W':
            return 'E'
        else:
            raise ValueError("Unknown direction")

    @classmethod
    def flatten_output(cls, listoflists):
        output = []
        for sublist in listoflists:
            if isinstance(sublist, list):
                output = [*output, *sublist]
            else:
                output.append(sublist)
        return output
        # return [item for sublist in listoflists for item in sublist]
    
    @staticmethod
    def extract_pos(node):
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


def tuple_multiply(t1, t2):
    return tuple(i * j for i, j in zip(t1, t2))

def tuple_swap(t: tuple):
    if len(t) != 2:
        raise ValueError
    
    return (t[1], t[0])

def point_within_straight_line(pt1, lpt1, lpt2):
    px, py = pt1
    lx1, ly1 = lpt1
    lx2, ly2 = lpt2

    if not (lx1 == lx2 or ly1 == ly2):
        # function only looking for if points are within a straight (1-d) line
        return False

    same_y = ly1 == ly2

    if same_y:
        return (py == ly1) and (lx1 <= px <= lx2 or lx2 <= px <= lx1)
    else:
        return (px == lx1) and ly1 <= py <= ly2 or ly2 <= py <= ly1

def point_within_rect(rect_x, rect_y, width, height, ox, oy):
    ex1, ey1 = rect_x + width, rect_y + height
    ex2, ey2 = rect_x - width, rect_y - height

    if (((ex1 <= ox <= ex2) or (ex2 <= ox <= ex1)) and ((ey1 <= oy <= ey2) or (ey2 <= oy <= ey1))):
        return True
    
    return False

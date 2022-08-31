# from pathfind import Pathfinder, Node
from pathfind import Pathfinder
from GUI_temp import TempGUI
import pytest

PLOT_VISUALIZATION = True


class TestPathfinder:
    def test_calculate_path_distance(self):
        pass

    def test_get_all_path_distance(self):
        pass

    @staticmethod
    def test_calculate_shortest_path():
        # nodes = [Node(3, 5), Node(6, 7), Node(2, 9), Node(18, 6)]
        # res = Pathfinder.get_path(nodes)
        pass


# if __name__ == "__main__":
#     start, end = (0,0), (10,10)

#     # target 1
#     obstacles = [(5, 14), (10, 5), (15, 3), (2, 10), (18, 16)]
    
#     ROBOT_DIST_FROM_IMG = 2
#     import random

#     targets = []
#     for image in obstacles:
#         img_direction = random.choice(['N','S','E','W'])
#         x, y = image
#         if img_direction == 'N':
#             targets.append((0+x, ROBOT_DIST_FROM_IMG+y))
#             # obstacles = [*obstacles, *[(target[0]+x, target[1]+y) for x in [1,0,-1] for y in [1,2]]]
#         elif img_direction == 'S':
#             targets.append((0+x, -ROBOT_DIST_FROM_IMG+y))
#             # obstacles = [*obstacles, *[(target[0]+x, target[1]+y) for x in [1,0,-1] for y in [-1,-2]]]
#         elif img_direction == 'W':
#             targets.append((-ROBOT_DIST_FROM_IMG+x, 0+y))
#             # obstacles = [*obstacles, *[(target[0]+x, target[1]+y) for x in [-1,-2] for y in [1,0,-1]]]
#         else:
#             targets.append((ROBOT_DIST_FROM_IMG+x, 0+y))
#             # obstacles = [*obstacles, *[(target[0]+x, target[1]+y) for x in [1,2] for y in [1,0,-1]]]

#     def update_callback(message=None):
#         times_called = 0
#         def wrapped(call_message):
#             nonlocal times_called
#             times_called+=1
#             print(f'{message} {times_called}')
#             if call_message:
#                 print(call_message)
#         return wrapped

#     points_completed_updater = update_callback(f'Pathfinding completed for point')

#     # user_input = input('1. Plot obstacles, images, and start location\n2. Plot shortest path to 5 images\n3. Plot non-optimal path to 5 images\n')
#     # if user_input == '1':
#     #     TempGUI.plot_targets_and_path([start, *targets], [], obstacles=obstacles, real_time=True)
#     # elif user_input == '2':
#     #     print('finding shortest path now...')
#     #     res = Pathfinder.shortest_path_to_n_points(start, targets, obstacles=obstacles)
#     #     print('\r              ')
#     #     from pathfind import global_LOADER_VAL
#     #     print('pathfinding completed successfully')
#     #     path = [item for sublist in res['path'] for item in sublist]
#     #     print(f'Number of iterations: {global_LOADER_VAL}')
#     #     print(f'Path taken: {len(path)} steps')
#     #     TempGUI.plot_targets_and_path([start, *targets], path, obstacles=obstacles, real_time=True)
#     # elif user_input == '3':
#     #     print('finding a non-optimal path now...')
#     #     res = Pathfinder.get_poor_path_between_n_points(start, targets, obstacles=obstacles)
#     #     print('\r              ')
#     #     from pathfind import global_LOADER_VAL
#     #     print('pathfinding completed successfully')
#     #     path = [item for sublist in res['path'] for item in sublist]
#     #     print(f'Number of iterations: {global_LOADER_VAL}')
#     #     print(f'Path taken: {len(path)} steps')
#     #     TempGUI.plot_targets_and_path([start, *targets], path, obstacles=obstacles, real_time=True)
#     # else:
#     #     print('input not recognised')

#     targets = [(3, 14), (12, 5), (15, 5), (2, 8), (20, 16)]
#     obstacles = [(5, 14), (10, 5), (15, 3), (2, 10), (18, 16)]
    
#     TempGUI.plot_targets_and_path([start, *targets], [], obstacles=obstacles, real_time=True)

#     print('================================================================')
#     print(f'{targets=}')
#     print(f'{obstacles=}')

#     facing_directions = Pathfinder.find_facing_directions(targets, obstacles)
#     # res = Pathfinder.shortest_path_to_n_points(
#         # start, targets, facing_direction_for_node_list=facing_directions, obstacles=obstacles, update_callback=points_completed_updater)
#     res = Pathfinder.shortest_path_to_n_points(start, targets, obstacles=obstacles, update_callback=points_completed_updater, facing_direction_for_node_list=[])
#     path = Pathfinder.flatten_output(res['path'])

#     print(f'\n{path=}\n')
#     print(instructions := Pathfinder.convert_path_to_instructions(path))

#     TempGUI.plot_targets_and_path([start, *targets], path, obstacles=obstacles, real_time=True, delay=0.2)

#     # facing_directions = Pathfinder.find_facing_directions(targets, obstacles)
#     # res = Pathfinder.get_path_between_two_points(start, targets[0], obstacles, facing_directions[0])

#     # TempGUI.plot_targets_and_path([start, *targets], res['path'], obstacles=obstacles, real_time=True, delay=0.4)


#     exit(1)



from pathfind import Pathfinder, Robot
from GUI_temp import TempGUI

def pathfind(obstacle_faces, obstacles_with_images, other_obstacles, 
            starting_face='N', start=(15,15)):  
    
    center_coords = lambda pt: (pt[0]+5, pt[1]+5)
    obstacles_with_images = [center_coords(pt) for pt in obstacles_with_images]
    other_obstacles = [center_coords(pt) for pt in other_obstacles]
    print(f'{obstacle_faces=}, {obstacles_with_images=}, {other_obstacles=}, {starting_face=}, {start=}' )

    obstacles = [*obstacles_with_images, *other_obstacles]
    targets = Pathfinder.generate_photo_taking_points(obstacles_with_images, obstacle_faces)
    result = Pathfinder.shortest_path_between_points_strategy(start, targets, obstacle_faces, obstacles, starting_face)
    
    return result

if __name__ == "__main__":
    print('running')
    starting_face = 'N'

    # obstacle_faces = ['N', 'W', 'S', 'N', 'W']
    # obstacles = [(40, 20), (190, 150), (50, 100), (160, 50), (120, 40)]
    # obstacle_ids = ['0', '1', '2', '3', '4']

    obstacles = [(10, 180), (60, 130), (130, 150), (180, 100)]
    obstacle_faces = ['S', 'S', 'S', 'W']
    targets = Pathfinder.generate_photo_taking_points(obstacles, obstacle_faces)

    TempGUI.plot_targets_and_path(targets=targets, obstacles=obstacles)

    result = pathfind(obstacles_with_images=obstacles, obstacle_faces=obstacle_faces, other_obstacles=[], starting_face=starting_face)
    pf_results = result['pathfinding']
    path, moves = pf_results['path'], pf_results['moves']
    path = Pathfinder.flatten_output(path)
    moves = Pathfinder.flatten_output(moves)
    path_faces = Pathfinder.determine_all_faces_on_path(starting_face, moves)

    print(moves)
    print(path)
    TempGUI.plot_targets_and_path(targets=targets, path=path, path_faces=path_faces, obstacles=obstacles, real_time=True, delay=0.8)
    

# def navigate_around_obstacle():
#     # Robot.turn_radius

#     one_side = ['REVERSE','REVERSE','REVERSE','REVERSE','RIGHT_FWD','FORWARD','LEFT_FWD','FORWARD','LEFT_FWD']
#     result = one_side * 3
    
#     mapping = {'FORWARD':'F','REVERSE': 'R','LEFT_FWD': 'L','RIGHT_FWD': 'R'}
#     return [mapping[i] for i in result]


# def display_path_around_obstacle():
#     facing = 'N'
#     path = []
#     faces = []
#     start = (10,10)
#     coords = start
#     for cmd in navigate_around_obstacle():
#         (dx,dy),facing, _ = Robot.move_w_facing(facing, cmd)
#         cx,cy = coords
#         coords = (cx+dx,cy+dy)
#         path.append(coords)

#     print(path)
#     TempGUI.plot_targets_and_path(start=start,targets=[(10, 13)], path=path, path_faces=faces, obstacles=[], real_time=True, delay=0.5)

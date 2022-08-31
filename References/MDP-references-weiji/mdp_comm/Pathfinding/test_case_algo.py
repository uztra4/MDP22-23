import random

# def main():
#     #TEST CASE LIST
#     test_case_list = []

#     # no_of_test_cases = int(input("Enter number of test cases: "))

#     for i in range(no_of_test_cases):
#         test_case_list.append(generate_one_test_case())

#     return test_case_list
    

def generate_one_test_case():
     #VARIABLES FOR OBSTACLES WITH IMAGES
    no_of_obstacle_images = 0
    obstacle_with_images = []
    obstacle_with_images_x = []
    obstacle_with_images_y = []

    #VARIABLES FOR OBSTACLE WITH NO IMAGES
    max_no_of_obstacle_no_images = random.randint(0,3)
    no_of_obstacle_no_images = 0
    obstacle_with_no_images_x = []
    obstacle_with_no_images_y = []

    #VARIABLES FOR FACES
    obstacle_image_faces = []
    IMAGE_FACES = ["N","S","E","W"]

    while(no_of_obstacle_images < 5):
        x = random.randint(0,19)
        y = random.randint(0,19)
        face = random.choice(IMAGE_FACES)

        if check_collision_edges(x, y, face) == True:
            continue

        if check_collision_with_images(x, y, face, obstacle_with_images_x, obstacle_with_images_y, obstacle_image_faces) == True:
            continue
        else:
            obstacle_with_images_x.append(x)
            obstacle_with_images_y.append(y)
            obstacle_image_faces.append(face)
            no_of_obstacle_images += 1
    
    while(no_of_obstacle_no_images < max_no_of_obstacle_no_images):
        x = random.randint(0,19)
        y = random.randint(0,19)
        if check_collision_without_images(x, y, obstacle_with_images_x, obstacle_with_images_y, obstacle_image_faces) == True:
            continue
        else:
            obstacle_with_no_images_x.append(x)
            obstacle_with_no_images_y.append(y)
            no_of_obstacle_no_images += 1
    
    obstacle_with_images = []
    for i in range(len(obstacle_with_images_x)):
        tuple_xy_image = (obstacle_with_images_x[i] * 10, obstacle_with_images_y[i] * 10)
        obstacle_with_images.append(tuple_xy_image)
    
    obstacle_with_no_images = []
    for i in range(len(obstacle_with_no_images_x)):
        tuple_xy_no_image = (obstacle_with_no_images_x[i] * 10, obstacle_with_no_images_y[i] * 10)
        obstacle_with_no_images.append(tuple_xy_no_image)
    
    one_test_case = [obstacle_with_images, obstacle_image_faces, obstacle_with_no_images]
    return one_test_case

def check_collision_edges(x , y, face):
    if( (x <= 4 and face == "W") or (x >= 16 and face == "E") ):
        return True
    elif( (y <= 4 and face == "S") or (y >= 16 and face == "N") ):
        return True
    else:
        return False


def check_collision_with_images(x, y, face, obstacle_with_images_x, obstacle_with_images_y, obstacle_image_faces):
    for count in range(len(obstacle_with_images_x)):
            if(x == obstacle_with_images_x[count]):
                distance = obstacle_with_images_y[count] - y
                if( (distance >= 0 and distance <=4) and (face == "N" or obstacle_image_faces[count] == "S") ):
                    return True
                elif( (distance <= 0 and distance >=-4) and (face == "S" or obstacle_image_faces[count] == "N") ):
                    return True
            if(y == obstacle_with_images_y[count]):
                distance = obstacle_with_images_x[count] - x
                if( (distance >= 0 and distance <=4) and (face == "E" or obstacle_image_faces[count] == "W") ):
                    return True
                elif( (distance <= 0 and distance >=-4) and (face == "W" or obstacle_image_faces[count] == "E") ):
                    return True
    return False

def check_collision_without_images(x, y, obstacle_with_images_x, obstacle_with_images_y, obstacle_image_faces):
    for count in range(len(obstacle_with_images_x)):
            if(x == obstacle_with_images_x[count]):
                distance = obstacle_with_images_y[count] - y
                if( (distance >= 0 and distance <=4) and (obstacle_image_faces[count] == "S") ):
                    return True
                elif( (distance <= 0 and distance >=-4) and (obstacle_image_faces[count] == "N") ):
                    return True
            if(y == obstacle_with_images_y[count]):
                distance = obstacle_with_images_x[count] - x
                if( (distance >= 0 and distance <=4) and (obstacle_image_faces[count] == "W") ):
                    return True
                elif( (distance <= 0 and distance >=-4) and (obstacle_image_faces[count] == "E") ):
                    return True
    return False

if __name__ == "__main__":
    from GUI_temp import TempGUI
    from pathfind import Pathfinder

    # for i in range(NUM_TEST_CASES):
    #     test_case = test_cases[i]
    #     obstacles_with_images, obstacle_faces, other_obstacles = test_case
    #     print(f'{obstacles_with_images=}')
    #     print(f'{obstacle_faces=}')
    #     print('-'*50)
    #     targets = Pathfinder.generate_photo_taking_points(obstacles_with_images, obstacle_faces)
    #     all_obstacles = [*obstacles_with_images, *other_obstacles]

    #     TempGUI.plot_targets_and_path(start=[], targets=targets, path=obstacles_with_images, obstacles=other_obstacles)

    test_case = generate_one_test_case()
        
    test_case = [[(80, 60), (180, 90), (50, 10), (150, 60), (140, 120)], ['N', 'N', 'N', 'N', 'E'], []]
    print(f'{test_case=}')
    starting_face = 'N'
    
    import main
    obstacles_with_images, obstacle_faces, other_obstacles = test_case

    print(f'{obstacles_with_images=}')
    print(f'{obstacle_faces=}')
    print('-'*50)

    targets = Pathfinder.generate_photo_taking_points(obstacles_with_images, obstacle_faces)
    TempGUI.plot_targets_and_path(targets=targets, obstacles_with_images=obstacles_with_images,obstacles=other_obstacles)

    obstacles_with_images, obstacle_faces, other_obstacles = test_case

    result = main.pathfind(obstacle_faces=obstacle_faces,
    obstacles_with_images=obstacles_with_images,
    other_obstacles=other_obstacles)
    
    pf_results = result['pathfinding']
    path, moves = pf_results['path'], pf_results['moves']
    path = Pathfinder.flatten_output(path)
    moves = Pathfinder.flatten_output(moves)
    path_faces = Pathfinder.determine_all_faces_on_path(starting_face, moves)

    print(path)
    print(moves)

    TempGUI.plot_targets_and_path(targets=targets, path=path, path_faces=path_faces, 
        obstacles_with_images=obstacles_with_images, obstacles=other_obstacles, real_time=True, delay=0.8)


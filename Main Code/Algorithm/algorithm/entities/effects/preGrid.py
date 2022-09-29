def getObsWorld(choice):
    obs_world_1 = [[145, 35, 180, 0], [115, 85, 0, 1], [25, 155, -90, 2], [175, 175, 180, 3], [105, 115, 180, 4]]
    obs_world_2 = [[145, 35, 0, 0], [115, 85, 0, 1], [25, 155, -90, 2], [175, 175, 180, 3], [105, 115, 180, 4]]
    obs_world_3 = [[145, 35, 0, 0], [85, 65, -90, 1], [25, 155, -90, 2], [175, 175, 180, 3], [105, 115, 180, 4]]
    obs_world_4 = [[145, 35, 0, 0], [45, 65, -90, 1], [25, 155, -90, 2], [175, 175, 180, 3], [105, 115, 180, 4]]
    obs_world_5 = [[55, 115, -90, 0], [145,115, 180, 1], [85, 95, 90, 2], [25, 95, 90, 3]]


    match choice:
        case 1:
            return obs_world_1
        case 2:
            return obs_world_2
        case 3:
            return obs_world_3
        case 4:
            return obs_world_4
        case 5:
            return obs_world_5
        case _:
            print("Invalid world")


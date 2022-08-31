from pathfind import Robot
import heapq
from functools import total_ordering

# valid_facings = Robot.valid_facings
moveset = Robot.moveset

@total_ordering
class heapableList(list):
    def __lt__(self, __x: list) -> bool:
        return len(self) < len(__x)

    def __eq__(self, other):
        return len(self) == len(other)

valid_facings = ['N', 'S', 'E','W']

for start_facing in valid_facings:
    for end_facing in valid_facings:
        if start_facing == end_facing:
            continue
        print()
        print('-'*50)
        print(start_facing, end_facing)
        start_coords = (0,0)
        # all previous facings, current facing,start_coords, moves
        queue = [(heapableList(start_facing), start_facing, start_coords, [])]

        done = False
        c = 0
        while queue and not done and c<50000:
            memo = {}
            c+=1
            prev_facing_list, facing, current_coords, prev_moves = heapq.heappop(queue)
            cx, cy = current_coords
            for move in moveset:
                (dx,dy), new_facing = Robot.move_w_facing(facing, move)
                new_coords = (cx+dx,cy+dy)
                if any(abs(i) >7 for i in new_coords):
                    continue
                facing_list = [*prev_facing_list, new_facing]
                moves = [*prev_moves, move]
                if tuple(moves) in memo:
                    continue
                """ eval function just needs to be on same axis """
                if new_facing == end_facing:
                    if end_facing in ['N','S']:
                        if new_coords[0] == start_coords[0]:
                            done = True
                    elif end_facing in ['E','W']:
                        if new_coords[1] == start_coords[1]:
                            done = True
                    # print(f'{start_facing=}, {end_facing=}')
                    if done:
                        print(facing_list, moves, new_coords)
                heapq.heappush(queue, (facing_list, new_facing,new_coords, moves))
                memo[tuple(moves)] = True



# cx,cy = (0,0)
# facing = 'N'
# for move in ['RIGHT_FWD', 'LEFT_FWD', 'LEFT_FWD', 'LEFT_FWD', 'FORWARD', 'FORWARD','LEFT_FWD']:
#     (dx,dy), facing = Robot.move_w_facing(facing, move)
#     cx,cy = (cx+dx,cy+dy)
#     print(cx,cy)
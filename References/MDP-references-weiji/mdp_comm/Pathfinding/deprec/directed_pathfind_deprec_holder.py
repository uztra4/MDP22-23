
# @classmethod
# def get_path_between_points(cls, node_list, obstacles: list,
#                          update_callback=None, facing_direction_for_node_list=None,
#                          starting_facing='N'):
#     '''
#     Returns a path using the internal Pathfinder get_path function that will travel between the given nodes, in the order they were given.

#     Parameters
#     ----------
#     node_list: List(Nodes)
#         The nodes to travel to, in the order of travel
#     facing_direction_for_node_list: List(str)
#         List of direction that the agent should be facing at each target node
#     obstacles: List(Nodes or Tuples) - mixed Tuple and Nodes are allowed
#         A list of the location of obstacles, of which travel on is prohibited

#     Returns
#     -------
#     {
#         'path': List of n List(Tuples), for n targets
#         'distance': int
#     }
#     '''
#     if not facing_direction_for_node_list:
#         facing_direction_for_node_list = []

#     output = {'path': [], 'distance': 0, 'final_facing': None, 'moves':[]}

#     if facing_direction_for_node_list:
#         if len(facing_direction_for_node_list) == len(node_list):
#             raise ValueError('List of facing directions should be one less than the list of target nodes, because there is no target direction at the first node')

#     facing_direction_at_end = None

#     facing = starting_facing
#     for i in range(len(node_list)-1):
#         start_node, end_node = node_list[i], node_list[i+1]
#         if facing_direction_for_node_list:
#             facing_direction_at_end = facing_direction_for_node_list[i]
#         res = Pathfinder.find_path_to_point(start=start_node, target=end_node, obstacles=obstacles,
#                                                     update_callback=update_callback, direction_at_target=facing_direction_at_end,
#                                                     starting_face=facing)
#         if not res:
#             return output
#         output['path'].append(res['path'])
#         output['distance'] += res['distance']
#         output['final_facing'] = res['final_facing']
#         output['moves'].append(res['moves'])
#         facing = res['final_facing']

#     return output

# @classmethod
# def shortest_path_to_n_points(cls, start_node: Node, node_list, obstacles: list,
#          approximate=True, update_callback=None, facing_direction_for_node_list=None,
#          starting_facing='N'):
#     '''
#     Given a unordered list of nodes, and a starting location, try to find the shortest path that will travel to all of the given nodes

#     Parameters
#     ----------
#     start_node: Node
#         The starting node from which the path will be started on the given
#     node_list: List(Node)
#         An unordered list of target nodes which will be visited. The order of visting is not guaranteed to be the same as the order in the list
#     facing_direction_for_node_list: List(str)
#         List of direction that the agent should be facing at each target node
#     obstacles: List(Tuple or Node)
#         A list of the location of obstacles, of which travel on is prohibited

#     Returns
#     -------
#     {
#         'path': List of n List(Tuples), for n targets
#         'distance': int
#     }
#     '''
#     if not facing_direction_for_node_list:
#         facing_direction_for_node_list = []
#     if len(node_list) == 0:
#         raise ValueError('Cannot find path if there are no target nodes')

#     if len(node_list) <= 3 or not approximate:
#         raise NotImplementedError('update code to follow path between two points function to use this function')

#         temp_result = []
#         all_paths = list(itertools.permutations(node_list))
#         all_paths = [(start_node, *path) for path in all_paths]
#         for path in all_paths:
#             _ = cls.get_path_between_points(node_list=path, obstacles=obstacles, update_callback=update_callback, facing_direction_for_node_list=facing_direction_for_node_list)
#             temp_result.append((_['path'], _['distance']))

#         path, distance = sorted(temp_result, key=lambda x: x[1])[0]
#         return {'path': path, 'distance': distance}

#     else:
#         traversal_order = [start_node]
#         facing_direction_in_traversal_order = []

#         current_node = start_node
#         node_list = node_list[:]

#         while node_list:
#             closest_node = sorted(node_list, key=lambda node:Pathfinder.h_function(*current_node, *node))[0]
#             traversal_order.append(closest_node)
#             if facing_direction_for_node_list:
#                 facing_direction_in_traversal_order.append(facing_direction_for_node_list[node_list.index(closest_node)])
#             node_list.remove(closest_node)
#             current_node = closest_node

#         print(f'{traversal_order=}')
#         return cls.get_path_between_points(node_list=traversal_order, obstacles=obstacles,
#             update_callback=update_callback, facing_direction_for_node_list=facing_direction_for_node_list,
#             starting_facing=starting_facing)

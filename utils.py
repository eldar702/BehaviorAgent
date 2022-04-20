# Eldar Shlomi 205616634

from pddlsim.parser_independent import Literal, Disjunction, Conjunction
from collections import defaultdict
import random


###############   Domain Helper Functions      ##############
#
# def get_goal(goal_raw_details):
#     if isinstance(goal_raw_details, Literal):
#         return [goal_raw_details.args]
#     goals = []
#     for sub_goal in goal_raw_details.parts:
#         goals = get_goal(sub_goal)
#     return goals


def get_goal(goal_raw_details):
    if isinstance(goal_raw_details, Literal):
        return [goal_raw_details.args]
    goals_list = []
    for sub_goal in goal_raw_details.parts:
         goals = get_goal(sub_goal)
         if len(goals) == 1:
             goals_list.append(goals[0])
         else:
             goals_list.extend(goals)
    return goals_list

def is_same(str1, str2):
    if str1 == str2:
        return True
    return False

############       Graph Helper Function       ###############
def get_min_path(bfs_nodes, destination_node):
    if destination_node is None:
        return None
    path = []
    node = destination_node
    path.append(node)

    while bfs_nodes[node] != -1:
        path.append(bfs_nodes[node])
        node = bfs_nodes[node]
    path.reverse()
    path_length = len(path)
    return path, path_length



############       GRAPH Class       ################
class Graph:
    def __init__(self):
        self.graph = defaultdict(list)
        self.prediction_dictionary = {}

    # function for adding edge to graph
    def addEdge(self, u, v):
        self.graph[u].append(v)

    def get_min_path_length(self, src_node, dest_node):
        bfs_network, distance = self.bfs(src_node)
        min_path, min_path_len = get_min_path(bfs_network, dest_node)
        return min_path_len

    def bfs(self, src_node):
        if src_node in self.prediction_dictionary:
            return self.prediction_dictionary[src_node]
        visited = {}
        bfs_nodes = {}
        distance = {}
        for v in self.graph:
            visited[v] = False
            bfs_nodes[v] = -1
            distance[v] = 10000000
        queue = []  # Initialize a queue

        visited[src_node] = True
        distance[src_node] = 0
        queue.append(src_node)

        while len(queue) != 0:
            u = queue[0]
            queue.pop(0)
            for i in self.graph[u]:
                if not visited[i]:
                    visited[i] = True
                    distance[i] = distance[u] + 1
                    bfs_nodes[i] = u
                    queue.append(i)

        self.prediction_dictionary[src_node] = (bfs_nodes, distance)
        return bfs_nodes, distance

# Eldar Shlomi 205616634
import random
import sys
import utils
###############     Imports      ###############
from pddlsim.executors.executor import Executor
from pddlsim.local_simulator import LocalSimulator
from pddlsim.parser_independent import Literal, Disjunction, Conjunction

# read files:
domain = sys.argv[1]
problem = sys.argv[2]


###############     Imports      ###############
class BehaviorBaseAgent(Executor):

    def __init__(self):
        super(BehaviorBaseAgent, self).__init__()
        self.graph = utils.Graph()
        self.domain_flag = None

    def graph_initialize(self, domain_type):
        actions = []
        if "football" in domain_type:
            actions, = ["connected"]
            self.domain_flag = "football"
        elif "maze" in domain_type:
            actions = ["west", "east", "south", "north"]
            self.domain_flag = "maze"
            # create the edges - edge for each state
        init_states_list = [state for state in self.services.parser.initial_state if state in actions]
        for action in init_states_list:
            initial_states = self.services.parser.initial_state[action]
            for specific_action in initial_states:
                self.graph.addEdge(specific_action[0], specific_action[1])

    def initialize(self, services):
        self.services = services
        self.graph_initialize(self.services.parser.domain_name)

    def next_action(self):
        if self.services.goal_tracking.reached_all_goals():
            return None
        valid_actions = self.services.valid_actions.get()
        if len(valid_actions) == 0:
            return None
        if len(valid_actions) == 1:
            return valid_actions[0]
        elif self.domain_flag == "football":
            return self.best_football_option(valid_actions)
        elif self.domain_flag == "maze":
            return self.best_maze_option(valid_actions)

    def best_football_option(self, valid_actions):
        return "kululu"

    def best_maze_option(self, all_options):
        best_path, best_path_len = [], float('inf')
        for option in all_options:
            option = ''.join([char for char in option if (char != '(' and char != ')')])
            player_name = option.split(' ')[1]
            src_tile = option.split(' ')[2]
            dest_tile = option.split(' ')[3]
            checked_path_len = self.shortest_distance(player_name, dest_tile)
            if checked_path_len == best_path_len:
                best_path.append(option)
            elif checked_path_len < best_path_len:
                best_path = []
                best_path.append(option)
                best_path_len = checked_path_len
        if len(best_path) > 1:
            best_path = random.choice(best_path)
        return best_path[0]

    def shortest_distance(self, player_name, start_tile):
        best_path = []
        best_path_len = float('inf')
        for raw_sub_goal in self.services.goal_tracking.uncompleted_goals:
            sub_goal = utils.get_goal(raw_sub_goal)
            sub_goal_player = sub_goal[0][0]
            sub_goal_dest = sub_goal[0][1]
            if player_name != sub_goal_player:
                continue
            checked_path_len = self.graph.get_min_path_length(start_tile, sub_goal_dest)
            if checked_path_len == best_path_len:
                best_path.append(sub_goal)
            elif checked_path_len < best_path_len:
                best_path = sub_goal
                best_path_len = checked_path_len
        # if len(best_path) > 1:
        #     return random.choice(best_path),
        return best_path_len

    def heuristic(self, option):
        pass


print LocalSimulator().run(domain, problem, BehaviorBaseAgent())
import random
import sys
import utils
##############################            Imports & Globals              #################################
from pddlsim.executors.executor import Executor
from pddlsim.local_simulator import LocalSimulator

# read files:
domain = sys.argv[1]
problem = sys.argv[2]

#########################################################################################################
###########################            BehaviorBaseAgent Class              #############################
#########################################################################################################
class BehaviorBaseAgent(Executor):

##############################             Init Fumctions               #################################
    def __init__(self):
        super(BehaviorBaseAgent, self).__init__()
        self.graph = utils.Graph()
        self.domain_flag = None
        self.chosen_ball, self.desire_place, self.not_desire_place = None, None, None
    def graph_initialize(self, domain_type):
        actions = []
        if "football" in domain_type:
            actions = ["connected"]
            self.domain_flag = "football"
            self.derive_ball_details()
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
        if self.domain_flag == "football":
            self.derive_ball_details()

    def derive_ball_details(self):
        self.balls_place, self.balls_goal, self.finished_balls = {}, {}, []
        for state in self.services.parser.initial_state["at-ball"]:
            ball_name = state[0]
            ball_location = state[1]
            if ball_name not in self.balls_place.keys():
                self.balls_place[ball_name] = ball_location

        for raw_sub_goal in self.services.goal_tracking.uncompleted_goals:

            sub_goal = utils.get_goal(raw_sub_goal)
            for idx in range(len(sub_goal)):
                subGoal_ball_name = sub_goal[idx][0]
                subGoal_ball_goal = sub_goal[idx][1]
                if subGoal_ball_name not in self.balls_goal.keys():
                    self.balls_goal[subGoal_ball_name] = subGoal_ball_goal

##############################            Next Action Functions               #################################
    def next_action(self):
        if self.services.goal_tracking.reached_all_goals():
            return None
        valid_actions = self.services.valid_actions.get()
        if len(valid_actions) == 0:
            return None
        if len(valid_actions) == 1:
            return valid_actions[0]
        elif self.domain_flag == "football":
            self.update_balls_places(self.services.perception.get_state())
            return self.best_football_option(valid_actions)
        elif self.domain_flag == "maze":
            return self.best_maze_option(valid_actions)

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
                best_path = [option]
                best_path_len = checked_path_len
        if len(best_path) > 1:
            best_path = [random.choice(best_path)]
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
        return best_path_len

    def best_football_option(self, all_options):
        actions_lists = [[] for tt in range(4)]
        best_path, best_path_profit, profit = [], float('inf'), float('inf')
        action_name, src_tile, dest_tile, ball_name, dest_tile_2 = None, None, None, None, None

        for option in all_options:
            option = ''.join([char for char in option if (char != '(' and char != ')')])
            action_name = option.split(' ')[0]
            if action_name == "move":
                src_tile = option.split(' ')[1]
                dest_tile = option.split(' ')[2]
                profit = self.heuristic_move_FB(src_tile, dest_tile)

            elif action_name == "kick":
                ball_name = option.split(' ')[1]
                src_tile = option.split(' ')[2]
                dest_tile = option.split(' ')[3]
                dest_tile_2 = option.split(' ')[4]
                if utils.is_same(dest_tile, dest_tile_2): continue
                if ball_name not in self.balls_goal.keys(): continue
                profit = self.heuristic_kick_FB(ball_name, src_tile, dest_tile, dest_tile_2)

            if profit == best_path_profit:
                best_path.append(option)
            elif profit < best_path_profit:
                best_path = [option]
                best_path_profit = profit

        if len(best_path) > 1:
            best_path = [random.choice(best_path)]

        return random.choice(best_path)

##############################            Heuristic Functions               #################################
    def heuristic_move_FB(self, src_tile, dest_tile):
        profit_src, profit_dst = None, None
        closest_ball_tile = self.find_closet_ball(dest_tile)
        profit_src = self.graph.get_min_path_length(src_tile, closest_ball_tile)
        profit_dst = self.graph.get_min_path_length(dest_tile, closest_ball_tile)

        return self.heuristic("football-move", profit_src, profit_dst)

    def heuristic_kick_FB(self, ball_name, src_tile, dest_tile, dest_tile_2):
        profit_src, profit_dst, profit_dst_2 = None, None, None
        goal_tile = self.balls_goal[ball_name]
        profit_src = self.graph.get_min_path_length(src_tile, goal_tile)
        profit_dst = self.graph.get_min_path_length(dest_tile, goal_tile)
        profit_dst_2 = self.graph.get_min_path_length(dest_tile_2, goal_tile)
        return self.heuristic("football-kick", profit_src, profit_dst, profit_dst_2)

    def update_balls_places(self, state):
        balls_details = list(state["at-ball"])

        for ball_name, ball_location in balls_details:
            if ball_name in self.finished_balls: continue
            if ball_name not in self.balls_goal.keys(): continue
            self.balls_place[ball_name] = ball_location
            if ball_location == self.balls_goal[ball_name]:
                self.finished_balls.append(ball_name)
                self.balls_goal.pop(ball_name, None)
                self.balls_place.pop(ball_name, None)

    def find_closet_ball(self, tile):
        min_dist, chosen_ball_location = float('inf'), None
        for ball_place in self.balls_place.values():
            checked_dist = self.graph.get_min_path_length(tile, ball_place)
            if checked_dist < min_dist:
                min_dist = checked_dist
                chosen_ball_location = ball_place
        return chosen_ball_location


    #desc: the heuristic function for all the policies
    # We choose the move whose value is the smallest, so first we will check if the move is "good", and the intention -
    # we will check if it brings us closer to the goal. When in the kick we will check if the kick brings us closer to
    # the goal and in "move" we will check if the movement brings us closer to a ball.
    def heuristic(self, name, *var):
        epsilon = 0.001
        if name == "football-move":
            return -10 / ((var[1] * (var[0] - var[1])) + epsilon)

        if name == "football-kick":
            return ((-1000 - 200 * (var[0] - var[2])) * (var[0] - var[1])) / ((0.8 * (var[1])) + epsilon)
        pass


print LocalSimulator().run(domain, problem, BehaviorBaseAgent())

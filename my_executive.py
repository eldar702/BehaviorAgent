# Eldar Shlomi 205616634
import random
import sys
import utils
##############################            Imports & Globals              #################################
from pddlsim.executors.executor import Executor
from pddlsim.local_simulator import LocalSimulator
from pddlsim.parser_independent import Literal, Disjunction, Conjunction

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
        self.balls_place, self.balls_goal = {}, {}
        for state in self.services.parser.initial_state["at-ball"]:
            ball_name = state[0]
            ball_location = state[1]
            if ball_name not in self.balls_place.keys():
                self.balls_place[ball_name] = ball_location

        for raw_sub_goal in self.services.goal_tracking.uncompleted_goals:
            sub_goal = utils.get_goal(raw_sub_goal)
            subGoal_ball_name = sub_goal[0][0]
            subGoal_ball_goal = sub_goal[0][1]
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

        best_path, best_path_profit, profit = [], float('inf'), float('inf')
        action_name, src_tile, dest_tile, ball_name, dest_tile_2 = None, None, None, None, None
        self.change_ball_location() # TODO: THIS FUNC
        for option in all_options:
            option = ''.join([char for char in option if (char != '(' and char != ')')])
            action_name = option.split(' ')[0]
            if action_name == "move":
                src_tile = option.split(' ')[1]
                dest_tile = option.split(' ')[2]
                profit = self.heuristic_move_FB(src_tile, dest_tile)
                # print(option + " " + )
            elif action_name == "kick":
                ball_name = option.split(' ')[1]
                src_tile = option.split(' ')[2]
                dest_tile = option.split(' ')[3]
                dest_tile_2 = option.split(' ')[4]
                if utils.is_same(dest_tile, dest_tile_2): continue
                profit = self.heuristic_kick_FB(ball_name, src_tile, dest_tile, dest_tile_2)

            if profit == best_path_profit:
                best_path.append(option)
            elif profit < best_path_profit:
                best_path = [option]
                best_path_profit = profit

        if len(best_path) > 1:
            best_path = [random.choice(best_path)]
        if "kick" in best_path:
            self.chosen_ball = best_path[0].split(' ')[1]
            self.desire_place = best_path[0].split(' ')[3]
            self.not_desire_place = best_path[0].split(' ')[4]

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
        closest_ball_tile = self.find_closet_ball(dest_tile)
        profit_src = self.graph.get_min_path_length(src_tile, goal_tile)
        profit_dst = self.graph.get_min_path_length(dest_tile, goal_tile)
        profit_dst_2 = self.graph.get_min_path_length(dest_tile_2, goal_tile)
        return self.heuristic("football-kick", profit_src, profit_dst, profit_dst_2)

    def find_closet_ball(self, tile):
        min_dist, chosen_ball_location = float('inf'), None
        for ball_place in self.balls_place.values():
            checked_dist = self.graph.get_min_path_length(tile, ball_place)
            if checked_dist < min_dist:
                min_dist = checked_dist
                chosen_ball_location = ball_place
            # elif checked_dist == min_dist:
            #     chosen_ball_location.append(ball_place)
        # if len(chosen_ball_location) > 1:
        #     chosen_ball_location = random.choice(chosen_ball_location)
        return chosen_ball_location


    # TODO: CREATE A SENTENCE OF THE LAST ACTION. CAN TAKE IT FROM LAST CHOSEN ACTION
    def change_ball_location(self):
        if self.chosen_ball is None or self.desire_place is None: return
        string = "(at-ball " + self.chosen_ball + " " + self.desire_place + ")"
        current_state = self.services.perception.get_state()
        if self.services.parser.test_condition(string, current_state):
            return
        self.balls_place[self.chosen_ball] = self.not_desire_place
    #desc: the heuristic function for all the policies
    # the move policy: calculate the difference between the distance to
    # the closet ball before and after the move (and multiple by 100)
    # the kick policy: First we will prefer a kick at the expense of move, which is why
    # we will multiply by 1000 (compared to 100 in displacement). Now - the formula will
    # be the difference in distance between the ball after the displacement to its goal
    def heuristic(self, name, *var):
        ### first version:
        # if name == "football-move":
        #     return 100 * (var[0] - var[1])  # 100 * (before_move_dist - after_move_dist )
        # if name == "football-kick":
        #     # 1000 * (before_kick_dist - after_kick_desire_dist) + (before_kick_dist - after_kick_not_desired_dist)
        #     return 1000 * (0.8 * (var[0] - var[1]) + (0.2 * var[0] - var[2]))
        epsilon = 0.001
        if name == "football-move":
            return -10 / ((var[1] * (var[0] - var[1])) + epsilon)  # 100 * (before_move_dist - after_move_dist )
        if name == "football-kick":
            # 1000 * (before_kick_dist - after_kick_desire_dist) + (before_kick_dist - after_kick_not_desired_dist)
            return ((-1000 - 200 * (var[0] - var[2])) * (var[0] - var[1])) / ((0.8 * (var[1])) + epsilon)
        pass




print LocalSimulator().run(domain, problem, BehaviorBaseAgent())

# ./python3 game.py <randomness> [loop, red]
# randomness between 0 and 100

import random
import copy
import os
import yaml

from pyniryo import *

from object_detection import ObjectDetection
from robot_player import RobotPlayer


def sort_score(obj):
    score = (obj.cx_rel + obj.cy_rel * 0.5)
    return score


class TicTacToe:
    def __init__(self, robot_client, grid_size, slope_pos, param, button_pin):

        self.__client = robot_client
        self.__params = param

        self.__objects = []
        self.__grid = None
        self.__grid_size = grid_size

        self.__workspace_name = ""
        self.__observation_pose = None
        self.__global_z_offset = 0.

        self.init_from_yaml()

        self.__object_detector = ObjectDetection(robot_client, self.__observation_pose, self.__workspace_name,
                                                 self.__global_z_offset,
                                                 grid_size)
        self.__robot = RobotPlayer(robot_client, self.__object_detector, self.__grid_size, slope_pos)
        self.__button_pin = button_pin

    def init_from_yaml(self):
        file_path = os.path.join(os.getcwd(), "tic_tac_toe_config.yaml")
        config = load_yaml(file_path)

        self.__workspace_name = config["workspace_name"]
        self.__global_z_offset = config["global_z_offset"]
        self.__observation_pose = config["observation_pose"]

    # put green pieces back in storage and put all other pieces out of the workspace
    def init_game(self):
        self.__objects = self.__object_detector.detect_all_object()
        self.__grid = self.__object_detector.put_objects_on_grid(self.__objects)

        # sort from top left to bottom right
        self.__objects.sort(key=sort_score)
        self.__display_grid()
        i, j, k, l_ = 0, 0, 0, 0
        x, y, z = 0, 0, 0

        while len(self.__objects) > 0:
            obj = self.__objects.pop(0)

            if obj.color == ObjectColor.GREEN and i <= len(self.__grid[0]) and self.__params["slope"] == 0:
                if i <= 3:
                    x, y, z = i, 0, 0
                elif i == 4:
                    x, y, z = i - 1, 1, 0
                elif i > 4:
                    x, y, z = 3, -1, 0
                i += 1

            elif obj.color == ObjectColor.BLUE and l_ == 0 and self.__params["slope"] == 0:
                x, y, z = -0.7, -0.7, 0
                l_ = 1

            else:
                x, y, z = -0.7, 1.5, j
                j += 0.01
                k += 1

            print('pick and place :', x, y, z)
            self.__robot.pick_place_on_grid(self.__grid, obj, x, y, z=z)

    def play_game(self):
        self.init_game()

        self.__objects = self.__object_detector.detect_all_object()
        self.__grid = self.__object_detector.put_objects_on_grid(self.__objects)

        end_game = False
        while not end_game:  # loop for each turn
            grid_o = self.__grid
            # Player's turn
            self.__objects = self.__robot.wait_other_player(self.__params["turn_end"], self.__button_pin)
            self.__grid = self.__object_detector.put_objects_on_grid(self.__objects)
            self.__display_grid()

            self.cheat_detection(grid_o, self.__grid)
            compiled_grid = self.__compile_grid(self.__grid)
            point = self.ai_play(compiled_grid, self.__params["rand"])
            end_game = self.check_end_game(False)
            if end_game:
                return

            # Robot's turn
            self.__robot.robot_play(point[:2], self.__grid, self.__params["slope"])

            self.__objects = self.__object_detector.detect_all_object()
            self.__grid = self.__object_detector.put_objects_on_grid(self.__objects)
            end_game = self.check_end_game(True)

    def check_end_game(self, is_robot_turn):
        compiled_grid = self.__compile_grid(self.__grid)
        if not is_robot_turn and self.ai_win_check(compiled_grid) == -1:
            self.__robot.lose_action()
            return True

        elif is_robot_turn and self.ai_win_check(compiled_grid) == 1:
            self.__robot.win_action()
            return True
        elif self.count_plays(compiled_grid) == 9:
            self.__robot.none_action()
            return True

        return False

    def cheat_detection(self, grid_view_1, grid_view_2):
        grid_1 = self.__compile_grid(grid_view_1)
        grid_2 = self.__compile_grid(grid_view_2)
        change = 0
        for x in range(0, 3):
            for y in range(0, 3):
                if grid_1[x][y] != 0 and grid_2[x][y] == 0:
                    print("a circle has been removed", x, y)
                    self.__robot.cheat_action()
                elif grid_1[x][y] != 0 and grid_2[x][y] != grid_1[x][y]:
                    print("a circle has been replace", x, y)
                    self.__robot.cheat_action()
                elif grid_1[x][y] == 0 and grid_2[x][y] == -1:
                    change += 1
                    if change > 1:
                        print("player has play multiple time")
                        self.__robot.cheat_action()

    # count how many pieces are on the play grid
    @staticmethod
    def count_plays(compiled_grid):
        tot = 0
        for x in range(0, 3):
            for y in range(0, 3):
                tot += abs(compiled_grid[x][y])
        return tot

    # take a grid and check for winner
    # return the value of the winner (-1 = human / 1 = AI)
    @staticmethod
    def ai_win_check(compiled_grid):
        for case in compiled_grid:
            if case[0] == case[1] == case[2] != 0:
                return case[0]
        for x in range(0, 3):
            if compiled_grid[0][x] == compiled_grid[1][x] == compiled_grid[2][x] != 0:
                return compiled_grid[0][x]
        if compiled_grid[0][0] == compiled_grid[1][1] == compiled_grid[2][2] != 0:
            return compiled_grid[0][0]
        if compiled_grid[0][2] == compiled_grid[1][1] == compiled_grid[2][0] != 0:
            return compiled_grid[0][2]
        return 0

    # in the work space: BLUE = AI, RED = human
    def ai_play(self, compiled_grid, randomness):
        score_grid = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ]
        for x in range(3):
            for y in range(3):
                score_grid[x][y] = random.randint(0, randomness * 100) / 100000.0
        # Fill the score grid
        for x in range(0, 3):
            for y in range(0, 3):
                if compiled_grid[x][y] == 0:
                    compiled_grid[x][y] = 1
                    score_grid[x][y] += self.ai_recursive(compiled_grid, -1)
                    compiled_grid[x][y] = 0
                else:
                    score_grid[x][y] = -9 ** 9 - 1
        # take the best score from the grid
        best_x, best_y = 0, 0
        best = -128
        for x in range(0, 3):
            for y in range(0, 3):
                if score_grid[x][y] > best:
                    best = score_grid[x][y]
                    best_x, best_y = x, y

        print("AI evaluation ...")
        print("AI pLays: " + str([best_x, best_y]))
        return [best_x, best_y, best]

    # recursive scan for the best play and return a score for each grid (1.0 = win, 1.0 = draw, -1.0 = lose)
    def ai_recursive(self, compiled_grid, turn):
        win = self.ai_win_check(compiled_grid)
        if win != 0:
            return win
        score = [0, 0, 0]
        # lose, equal, win
        for x in range(0, 3):
            for y in range(0, 3):
                if compiled_grid[x][y] == 0:
                    compiled_grid[x][y] = turn
                    tmp = self.ai_recursive(compiled_grid, -turn)
                    if tmp > 0:
                        score[turn + 1] += tmp
                    elif tmp == 0:
                        score[1] += 1
                    else:
                        score[-turn + 1] += -tmp
                    compiled_grid[x][y] = 0
        if score[2] > 0:
            return turn * score[2] / 9
        elif score[1] > 0:
            return 0
        elif score[0] > 0:
            return -turn * score[0] / 9
        return 0

    # compile view_grid in grid (0 empty space, 1 blue circle, -1 red circle)
    def __compile_grid(self, grid):
        game_grid = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ]
        if self.__params["slope"] == 1:
            grid = copy.deepcopy(grid)
            for x in range(0, 3):
                grid[x].insert(0, 0)

        for x in range(0, 3):
            for y in range(0, 3):
                if len(grid[x][y + 1]) > 0:
                    if grid[x][y + 1][0].color == ObjectColor.GREEN:
                        game_grid[x][y] = 1
                    elif grid[x][y + 1][0].color == ObjectColor.BLUE:
                        game_grid[x][y] = -1
        return game_grid

    # get an array of class Object and put it on a grid
    def __display_grid(self):
        rgb_dict = {ObjectColor.RED.value: " R ", ObjectColor.GREEN.value: " G ", ObjectColor.BLUE.value: " B "}
        case = []
        for case in self.__grid:
            print("+" + "---+" * len(case) + "\n|", end="")
            for objs in case:
                if len(objs) > 0:
                    print(rgb_dict[objs[0].color.value] if objs[0].color.value in rgb_dict else "", end="")
                else:
                    print("   ", end='')
                print("|", end='')
            print("")
        print("+" + "---+" * len(case))


def load_yaml(path_):
    if os.path.exists(path_):  # check if file exists
        if os.stat(path_).st_size == 0:  # check if file not empty
            saved_items = {}
        else:
            with open(path_, 'r') as f:
                saved_items = yaml.load(f)
        return saved_items
    else:
        print("Empty or missing file: {}".format(path_))
        return {}

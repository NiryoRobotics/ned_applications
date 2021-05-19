import os
import math
import time
import yaml
from pyniryo import *


class RobotPlayer:
    def __init__(self, client, object_detector, grid_size, slope_pose):
        self.__client = client
        self.__object_detector = object_detector
        self.__grid_size_ = grid_size

        self.__slope_pos = slope_pose
        self.__sleep_joints = None
        self.__cheat_pose = None

        self.__global_z_offset = 0.
        self.__workspace_name = ""

        self.init_from_yaml()

    def init_from_yaml(self):
        file_path = os.path.join(os.getcwd(), "tic_tac_toe_config.yaml")
        config = load_yaml(file_path)

        self.__workspace_name = config["workspace_name"]
        self.__global_z_offset = config["global_z_offset"]

        self.__sleep_joints = config["sleep_joints"]
        self.__cheat_pose = config["cheat_pose"]

    def robot_play(self, point, grid, slope):
        if slope:
            self.__client.push_air_vacuum_pump()
            self.__client.move_joints(*self.__sleep_joints)
            self.__client.move_joints(*self.__slope_pos)
            self.__client.pull_air_vacuum_pump()
            self.__client.move_joints(*self.__sleep_joints)
            self.place_on_grid(grid, *point)
        else:
            obj = None
            while obj is None:
                for case in grid:
                    if len(case[0]) > 0 and case[0][0].color == ObjectColor.GREEN:
                        obj = case[0][0]
                        break
                if obj is None:
                    self.no_more_circles_action()
                    objs = self.__object_detector.detect_all_object()
                    grid = self.__object_detector.put_objects_on_grid(objs)
            self.pick_place_on_grid(grid, obj, point[0], point[1] + 1)

    # wait player turn
    def wait_other_player(self, turn_end_type, button_pin=PinID.GPIO_1A):
        print("Player Turn")
        if turn_end_type == 1:
            # wait for button press
            self.wait_for_button_press(button_pin)
            return self.__object_detector.detect_all_object()

        elif turn_end_type == 2:
            # wait for arm movement
            self.wait_for_moving_arm()
            return self.__object_detector.detect_all_object()

        else:
            # wait for the red circle
            return self.wait_for_red_circle_move()

    def wait_for_button_press(self, button_pin):
        self.__client.move_joints(*self.__sleep_joints)
        print("waiting button press...")
        self.__client.set_learning_mode(True)
        while True:
            a, pin = self.__client.digital_read(button_pin)
            if pin == 0:
                break
        self.__client.set_learning_mode(False)

    def wait_for_moving_arm(self):
        print("waiting for player to move the arm...")
        self.__client.move_joints(*self.__sleep_joints)
        self.__client.set_learning_mode(True)
        time.sleep(3)
        joints_o = self.__client.get_joints()

        robot_moved = False
        while not robot_moved:
            joints = self.__client.get_joints()
            for x in range(0, len(joints)):
                if abs(joints[x] - joints_o[x]) > 0.0872665:
                    robot_moved = True
                    break
                joints_o[x] = joints_o[x] * 0.9 + joints[x] * 0.1
            time.sleep(0.1)

        time.sleep(1)
        self.__client.set_learning_mode(False)

    def wait_for_red_circle_move(self):
        print("waiting red circle to be move...")
        red_circle = None
        grid = None
        objects_list = []
        while not red_circle:
            objects_list = self.__object_detector.detect_all_object()
            grid = self.__object_detector.put_objects_on_grid(objects_list)

            for obj_index, obj in enumerate(objects_list):
                if obj.color == ObjectColor.RED:
                    red_circle = obj
                    objects_list.pop(obj_index)
                    break

        self.pick_place_on_grid(grid, red_circle, -0.7, -0.7)
        return objects_list

    @staticmethod
    def no_more_circles_action():
        print("no more circle in stock")

    def win_action(self):
        print("robot win")
        self.__client.move_joints(*[0, 0, 0.6, 0, 0, 0])
        self.__client.move_joints(*[0, 0.6, -0.5, 0, 0, 0])
        self.__client.move_joints(*[0, 0, 0.6, 0, 0, 0])
        self.__client.wait(1)

    def lose_action(self):
        print("player win")
        self.__client.move_joints(*[0.950, 0.35, -1.02, 0.025, 0.129, -1.45])
        self.__client.move_joints(*[0.950, 0.35, -1.02, 0.025, -0.444, -1.45])
        self.__client.move_joints(*[0.800, 0.35, -1.02, 0.025, -0.444, -1.45])
        self.__client.move_joints(*[1.05, 0.35, -1.02, 0.025, -0.444, -1.45])
        self.__client.move_joints(*[0.800, 0.35, -1.02, 0.025, -0.444, -1.45])
        self.__client.move_joints(*[1.05, 0.35, -1.02, 0.025, -0.444, -1.45])
        self.__client.move_joints(*self.__sleep_joints)
        self.__client.wait(1)

    def none_action(self):
        print("nobody win")
        self.__client.move_joints(*[0, 0, 0, 0, 0, 0])
        self.__client.move_joints(*[0, 0, 0, -1.570, 0, 0])
        self.__client.move_joints(*[0, 0, 0, 1.570, 0, 0])
        self.__client.move_joints(*[0, 0, 0, 0, 0, 0])

    def cheat_action(self):
        print('Ned thinks you cheated ! :( ')
        self.__client.move_pose(*self.__cheat_pose)
        time.sleep(1)

    def place_on_grid(self, grid, x, y, z=0):
        obj_pos = [0.0, 0.0, 0.0, 0.0, math.pi / 2, 0.0]
        pos_min = self.__client.get_target_pose_from_rel(self.__workspace_name, height_offset=0.0, x_rel=0, y_rel=0,
                                                         yaw_rel=0)
        pos_max = self.__client.get_target_pose_from_rel(self.__workspace_name, height_offset=0.0, x_rel=1, y_rel=1,
                                                         yaw_rel=0)
        size_x = (pos_max.x - pos_min.x) / len(grid)
        size_y = (pos_max.y - pos_min.y) / len(grid[0])

        obj_pos[0] = pos_min.x + size_x * (x + 0.5)
        obj_pos[1] = pos_min.y + size_y * (y + 0.5)
        obj_pos[2] = pos_min.z + z + self.__global_z_offset

        self.__client.place_from_pose(*obj_pos)

    # take a shape and put it in a case
    def pick_place_on_grid(self, grid, obj, x, y, z=0):
        print(obj.color, "pawn =>", x, y)
        pos_min = self.__client.get_target_pose_from_rel(self.__workspace_name, height_offset=0.0, x_rel=0, y_rel=0,
                                                         yaw_rel=0)
        pos_max = self.__client.get_target_pose_from_rel(self.__workspace_name, height_offset=0.0, x_rel=1, y_rel=1,
                                                         yaw_rel=0)

        size_x = (pos_max.x - pos_min.x) / len(grid)
        size_y = (pos_max.y - pos_min.y) / len(grid[0])

        self.__client.pick_from_pose(obj.obj_pos.to_list())

        obj.obj_pos.x = pos_min.x + size_x * (x + 0.5)
        obj.obj_pos.y = pos_min.y + size_y * (y + 0.5)
        obj.obj_pos.z = pos_min.z + z + self.__global_z_offset

        self.__client.place_from_pose(*obj.obj_pos.to_list())

    def move_in_grid(self, grid, x1, y1, x2, y2):
        self.pick_place_on_grid(grid, grid[x1][y1][0], x2, y2)
        grid[x2][y2].insert(grid[x1][y1].pop(0), 0)


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

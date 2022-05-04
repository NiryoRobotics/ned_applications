#!/usr/bin/env python

import os
import sys
import math
import copy
import random
import time
import yaml
import packaging
from threading import Lock, Thread
from pyniryo import *

# - Set up
workspace1 = "ws_multirobot_line"  # Name of your workspace
conf_file_name = "saved_joints_poses.yaml"

# robot 1
front_ned_ip_address = "10.10.10.10"  # "192.168.1.26"  # Replace by front Ned ip address

# robot 2
back_ned_ip_address = "169.254.200.200"  # Replace by back Ned ip address

# define the height offset of the workspace used by the vision pick method
z_offset = 0.002
sleep_joints = [0.0, 0.55, -1.2, 0.0, 0.0, 0.0]
vision_brightness = 1  # value between 0.5 and 2 to determine how bright the camera vision is

# conveyor
conveyor_id = ConveyorID.ID_1  # define the id of the conveyor connected to the front Ned
conveyor_speed = 50  # speed of the conveyor between 0 and 100
conveyor_place_interval = 1.5  # minimal interval between conveyor places  (in seconds)
rand_drop = 1  # value between 0 and 1 how much random is use to determine the drop position


# - Main loop for the two robots

class RobotsMains:
    def __init__(self, front_ned, back_ned, workspace, saved_joints_poses):
        self.saved_joints_poses = saved_joints_poses

        self.client1 = front_ned
        self.client2 = back_ned

        self.client1.set_brightness(vision_brightness)

        self.slope_lock = Lock()

        self.workspace = workspace
        self.conveyor_id = conveyor_id
        self.stock = 0
        self.conveyor_move_time = 0

    # starting a thread of each robot
    def run(self):
        self.conveyor_id = self.client1.set_conveyor()

        front_ned_thread = FrontNed(self.client1, self, self.workspace)
        back_ned_thread = BackNed(self.client2, self)
        front_ned_thread.start()
        back_ned_thread.start()


# this class correpond to a robot, this class is run on a separate thread
# and permit multiple robots to move simultaneously
class RobotLoop(Thread):
    def __init__(self, client, parent):
        self.parent = parent
        self.saved_joints_poses = self.parent.saved_joints_poses
        self.client = client
        self.slope_lock = self.parent.slope_lock

        Thread.__init__(self)  # init this class has a thread

    def run(self):
        self.client.move_joints(*sleep_joints)
        self.robot_loop()

    def robot_loop(self):
        pass


class FrontNed(RobotLoop):
    def __init__(self, client, parent, workspace):
        super().__init__(client, parent)

        self.workspace = workspace

    def robot_loop(self):
        print("Front Ned loop start")
        self.client.update_tool()
        self.client.release_with_tool()
        self.client.control_conveyor(self.parent.conveyor_id, False, conveyor_speed, ConveyorDirection.BACKWARD)

        while True:
            # run the conveyor until a pawns get in the workspace
            while True:
                obj_found, *_ = self.client.vision_pick(workspace1, z_offset, shape=ObjectShape.ANY,
                                                        color=ObjectColor.ANY)
                if obj_found:
                    break
                self.wait_obj()

            self.slope_lock.acquire()  # waiting for the slope zone to be clear
            print("Front Ned | going over slope ")
            self.client.move_joints(*self.saved_joints_poses["client1_intermediate_pos"])  # up
            print("Front Ned | dropping pawn ")
            self.client.move_joints(*self.saved_joints_poses["drop_positions_of_client1"])  # drop
            self.client.release_with_tool()
            self.parent.stock += 1

            self.client.move_joints(*self.saved_joints_poses["client1_observation_pose"])
            self.slope_lock.release()  # set the slope zone as clear

            time.sleep(0.1)  # let back_ned to catch the lock

    # wait for an object to come on the workspace
    def wait_obj(self):
        self.client.move_joints(*self.saved_joints_poses["client1_observation_pose"])  # observation
        obj_found, pos, shape, color = self.client.detect_object(workspace1, shape=ObjectShape.ANY,
                                                                 color=ObjectColor.ANY)
        if obj_found and pos[0] < 0.90:
            return
        self.client.control_conveyor(self.parent.conveyor_id, True, conveyor_speed, ConveyorDirection.BACKWARD)
        while obj_found is False or pos[0] > 0.90:
            t = time.time()
            obj_found, pos, shape, color = self.client.detect_object(workspace1, shape=ObjectShape.ANY,
                                                                     color=ObjectColor.ANY)
            self.parent.conveyor_move_time += time.time() - t
        time.sleep(0.3)
        self.client.control_conveyor(conveyor_id, False, 0, ConveyorDirection.BACKWARD)


class BackNed(RobotLoop):
    def __init__(self, client, parent):
        super().__init__(client, parent)

    def robot_loop(self):
        print("Back Ned loop start")
        self.client.update_tool()
        self.client.release_with_tool()

        while True:
            print_count = 0
            while True:
                print_count += 1
                if print_count == 10:
                    print("stock in slope : ", self.parent.stock, " | move_time :",
                          self.parent.conveyor_move_time)
                    print_count = 0
                if self.parent.stock > 0:
                    break
                time.sleep(0.1)

            self.slope_lock.acquire()  # waiting for the trajectory to be clear
            print("Back Ned | going above slope ")
            self.client.move_joints(*self.saved_joints_poses["client2_intermediate_pos"])  # up
            print("Back Ned | grab a pawn ")
            self.client.move_joints(*self.saved_joints_poses["pick_positions_of_client2"])  # grab
            self.client.grasp_with_tool()
            self.parent.stock -= 1

            print("Back Ned | going above slope ")
            self.client.move_joints(*self.saved_joints_poses["client2_intermediate_pos"])  # up
            tmp = copy.deepcopy(self.saved_joints_poses["drop_positions_of_client2"])
            tmp[2] += (random.randrange(1000) / 1000.0 - 0.5) / 4.0 * rand_drop
            tmp[5] += (random.randrange(1000) / 1000.0 - 0.5) * 0.5 * math.pi * rand_drop
            print("Back Ned | dropping pawn on conveyor ")
            self.client.move_joints(*tmp)  # drop

            self.slope_lock.release()

            while True:
                print("stock in slope : ", self.parent.stock, " | move_time :", self.parent.conveyor_move_time)
                if self.parent.conveyor_move_time > conveyor_place_interval:
                    break
                time.sleep(0.1)

            self.client.release_with_tool()
            self.parent.conveyor_move_time = 0


# - Initialize positions

def ask_position():
    joints_pose_dict = {}

    client1.move_joints(*sleep_joints)
    client2.move_joints(*sleep_joints)
    client1.set_learning_mode(True)
    client2.set_learning_mode(True)

    # the text displayed for each ask
    questions = ["FRONT NED | Set the observation pose so the 4 landmarks are detected",
                 "FRONT NED | A position of a few centimeters above the slope (at the top of the slope) ...",
                 "FRONT NED | The position from which Ned can drop the pawn on the slope ...",
                 "BACK NED | The position from which Ned can grab the pawn at the bottom of the slope ...",
                 "BACK NED | A position a few centimeters above the previous position ...",
                 "BACK NED | A position at the back of the Conveyor Belt where Ned can drop the pawn ..."]

    # name of the position (cannot contain spaces)
    names = ["client1_observation_pose",
             "client1_intermediate_pos",
             "drop_positions_of_client1",
             "pick_positions_of_client2",
             "client2_intermediate_pos",
             "drop_positions_of_client2"]

    # function execute when position is given [function, args...]
    function = [[nothing],
                [nothing],
                [client1.release_with_tool],
                [client2.grasp_with_tool],
                [nothing],
                [client2.release_with_tool]]

    # client from which the position is taken
    client = [client1, client1, client1, client2, client2, client2]

    for question_index in range(len(questions)):
        input(questions[question_index])
        print('data joint : ', client[question_index].get_joints())
        joints_pose_dict[names[question_index]] = client[question_index].get_joints()
        function[question_index][0]()
        client[question_index].set_learning_mode(True)

    client1.move_joints(*sleep_joints)
    client2.move_joints(*sleep_joints)
    client2.set_learning_mode(True)
    client1.set_learning_mode(True)

    return joints_pose_dict


# - Useful functions

def create_new_workspace():
    print('Setting a new workspace : ')
    points = []
    id_point = 1

    for id_point in range(4):  # Iterating over 4 markers
        input("Press enter when on point".format(id_point + 1))
        # Getting pose
        points.append(client1.get_pose())
    input("Equip the operating tool")

    # Creating workspace
    client1.save_workspace_from_robot_poses(workspace1, *points)


# load all the robots pose or ask for new ones
def load_saved_joint_poses():
    file_path = os.path.join(os.getcwd(), conf_file_name)

    saved_joints_poses = load_yaml(file_path)
    if saved_joints_poses:
        print('setup positions retrieved from {} file'.format(conf_file_name))
    else:
        print('{} file not found ... asking for new setup positions'.format(conf_file_name))
        saved_joints_poses = ask_position()  # ask for new pose
        save_yaml(file_path, saved_joints_poses)
    return saved_joints_poses


# save dictionary in yaml
def save_yaml(path_, dict_):
    with open(path_, 'w') as f:
        yaml.dump(dict_, f, default_flow_style=False)


# load dictionary from yaml
def load_yaml(path_):
    if os.path.exists(path_):  # check if file exists
        if os.stat(path_).st_size == 0:  # check if file not empty
            saved_items = {}
        else:
            with open(path_, 'r') as f:
                saved_items = yaml.safe_load(f)
        return saved_items
    else:
        print("Empty or missing file: {}".format(path_))
        return {}


def main():
    saved_joint_poses = load_saved_joint_poses()  # load all the robots poses

    client1.release_with_tool()
    client2.release_with_tool()

    ws_list = client1.get_workspace_list()
    if workspace1 not in ws_list:
        print('Error : ', workspace1, 'not found in Front Ned workspace list..')
        create_new_workspace()

    main_loops = RobotsMains(client1, client2, workspace1, saved_joint_poses)
    main_loops.run()


def nothing():
    pass


# - Start Flag

def reset():  # delete saved pose
    print("reset saved positions ...")
    if conf_file_name in os.listdir("."):
        os.remove(conf_file_name)


flags = {
    "--reset": reset
}

# - MAIN

if __name__ == '__main__':

    # argument parsing
    for av in sys.argv[1:]:
        if av in flags:
            flags[av]()
        else:
            print("unknown flag: ", av)

    client1 = NiryoRobot(front_ned_ip_address)
    client2 = NiryoRobot(back_ned_ip_address)

    calib_thread_r1 = Thread(target=client1.calibrate, args=[CalibrateMode.AUTO, ])
    calib_thread_r2 = Thread(target=client2.calibrate, args=[CalibrateMode.AUTO, ])
    calib_thread_r1.start()
    calib_thread_r2.start()
    calib_thread_r1.join()
    calib_thread_r2.join()

    client1.update_tool()
    client2.update_tool()

    main()

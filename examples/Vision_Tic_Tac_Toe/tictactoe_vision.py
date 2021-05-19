# ./python3 game.py <randomness> [loop, red]
# randomness between 0 and 100

import os
import time
import sys
import yaml

from pyniryo import *

from game import TicTacToe, load_yaml

# Connecting to robot
robot_ip_address = "192.168.1.23"  # Replace by robot ip address, "10.10.10.10" in hotspot mode
client = NiryoRobot(robot_ip_address)

slope_pos = []
button_pin = PinID.GPIO_1A  # GPIO_1A for the button only if you use the --button or -b flags
grid_size = [4, 4]

file_path = os.path.join(os.getcwd(), "tic_tac_toe_config.yaml")
sleep_joints = load_yaml(file_path)["sleep_joints"]

mode = [
    [
        "\nLoop mode (automatically restart at the end of each game)?",
        "1: Off (default)",
        "2: On [--loop]"
    ], [
        "\nRobot turn mod (how you want the robot to know when to play)?",
        "1: Red circle [--red] ",
        "2: Button press [--button]",
        "3: Learning mode (default)"
    ], [
        "\nUse the slope?",
        "1: No  [--noslope]",
        "2: Yes (default)",
        "3: Reset slope position [--reset]"
    ]
]


def int_input(min_, max_):
    while True:
        try:
            a = int(input())
            if min_ <= a <= max_:
                return a
        except NiryoRobotException:
            pass


# display menu and ask for user input
def menu():
    for elem in mode[0]:
        print(elem)
    param["loop"] = int_input(1, len(mode[0]) - 1) - 1
    for elem in mode[1]:
        print(elem)
    param["turn_end"] = int_input(1, len(mode[1]) - 1) - 1

    for elem in mode[2]:
        print(elem)
    param["slope"] = int_input(1, len(mode[2]) - 1) - 1
    if param["slope"] == 2:  # if reset slope position
        param["slope"] = 1
        reset_slop_pose()

    print("Randomness between 0 and 1000 (default: 5)")
    param["rand"] = int_input(0, 1000)
    return param


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
                saved_items = yaml.load(f)
        return saved_items
    else:
        print("Empty or missing file: {}".format(path_))
        return {}


def reset_slop_pose():
    config = load_yaml(file_path)
    try:
        del config["slop_pos"]
        with open(file_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
    except KeyError:
        pass


def get_slope_pos():
    config = load_yaml(file_path)
    try:
        # If the slope pose is already defined
        if config["slop_pos"]:
            str1 = config["slop_pos"]
            for x in range(6):
                slope_pos.append(str1[x])
            print("slope pos:" + str(slope_pos))
            return
    except KeyError:
        pass

    # If the slope pose is not defined
    client.set_learning_mode(True)
    print("the slope pickup position is undefined, put the robot arm on the picking "
          "position and press enter or use --noslope to play without it")
    input()
    joints = client.get_joints()
    time.sleep(1)
    client.move_joints(*sleep_joints)
    config["slop_pos"] = joints
    save_yaml(file_path, config)
    get_slope_pos()


if __name__ == '__main__':

    param = {
        "loop": 0,
        "turn_end": 0,
        "menu": 0,
        "rand": 5,
        "slope": 1
    }
    # param is a table containing all the parameters that the user can change
    # loop off(0)/on(1), turn mode red(0)/button(1)/learning_mode(2), menu off(0)/on(1), randomness, slope off(0)/on(1)
    try:
        client.calibrate(CalibrateMode.AUTO)
        client.update_tool()
    except NiryoRobotException:
        print("calibration failed")

    for av in sys.argv[1:]:
        if av == "--menu" or av == "-m":
            param["menu"] = 1
        elif av == "--loop" or av == "-l":
            param["loop"] = 1
        elif av == "--red" or av == "-g":
            param["turn_end"] = 0  # red shape
        elif av == "--button" or av == "-b":
            param["turn_end"] = 1  # button press
        elif av == "--noslope" or av == "-s":
            param["slope"] = 0
        elif av == "--reset":
            reset_slop_pose()
        else:
            param["rand"] = max(min(int(av), 1000), 0)

    if param["menu"] == 1:
        param = menu()

    if param["slope"] == 1:
        get_slope_pos()
        grid_size[0] -= 1
        grid_size[1] -= 1

    print("#" * 10)
    param["rand"] /= 10

    if param["turn_end"] == 1:
        client.set_pin_mode(button_pin, PinMode.INPUT)

    tic_tac_toe = TicTacToe(client, grid_size, slope_pos, param, button_pin)
    try:
        while True:
            tic_tac_toe.play_game()

            client.move_joints(*sleep_joints)
            if param["loop"] == 0:
                break
            time.sleep(3)
        client.set_learning_mode(True)
    except Exception as e:
        print('exception : ', e)
        client.move_joints(*sleep_joints)
        client.set_learning_mode(True)

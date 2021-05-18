#!/usr/bin/env python


import time
import math
import os
import utils
from pyniryo import *

robot_ip_address = "192.168.1.52"  # Replace by robot ip address
workspace = "ws_tensorflow"

observation_pose = PoseObject(  # position for the robot to watch the workspace
    x=0.20, y=0, z=0.36,
    roll=0.0, pitch=math.pi / 2, yaw=0.0,
)


def labelling(client, name):
    try:
        os.mkdir("./data/" + name)
    except FileExistsError:
        pass
    print("label ", name)
    a, img_work = utils.take_workspace_img(client)

    mask = utils.objs_mask(img_work)

    debug = concat_imgs((img_work, mask), 1)
    if __name__ == '__main__':
        show_img("robot view", debug, wait_ms=1)

    objs = utils.extract_objs(img_work, mask)

    if len(objs) != 0:
        print("*** ", str(len(objs)) + " object detected ! ***")

        objs[0].img = resize_img(objs[0].img, width=64, height=64)
        if __name__ == '__main__':
            show_img("robot view2", img_work, wait_ms=50)
        print("saved", name)
        cv2.imwrite("./data/" + name + "/" + str(time.time()) + ".jpg", img_work)
    else:
        print("*** ", str(len(objs)) + " object detected ... ***")
    return img_work


def nothing():
    pass


if __name__ == '__main__':
    # Connecting to robot
    client = NiryoRobot(robot_ip_address)
    try:
        client.calibrate(CalibrateMode.AUTO)
        client.update_tool()
    except NiryoRobotException:
        print("calibration failed")
    name = input("object name :")
    try:
        os.mkdir("./data")
    except FileExistsError:
        pass
    try:
        os.mkdir("./data/" + name)
    except FileExistsError:
        pass
    a, img_work = utils.take_workspace_img(client)
    show_img("robot view", img_work, wait_ms=50)
    cv2.createTrackbar("threshold", "robot view", 130, 256, nothing)
    while "user doesn't quit":
        input("press enter to take picture")
        labelling(client, name)

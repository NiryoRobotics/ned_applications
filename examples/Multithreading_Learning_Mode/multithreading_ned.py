#!/usr/bin/env python
"""
This script aims to use the multithreading to control a Ned with an other Ned.
One will be the leader one and the other one will be the follower.
With this script you will be able to control with the learning mode one Ned and the following one will execute the same
movement.
All the information to run this script are given in Niryo's Documentation

Please, execute the program with python 3
"""

import time
from pyniryo import *
from threading import Thread

# ----------------------------------------Part to change ------------------------------------------------
leader_ip = "192.168.1.52"  # ip of the Leading robot
clients_ip = ["127.0.0.1"]  # ips of the followers
# -------------------------------------------------------------------------------------------------------

sensitivity = 30  # in deg/s
stable_time = 0.25  # time under the sensitivity to queue the new position

tool = ToolID.GRIPPER_2
default_pose = [0.0, 0.55, -1.2, 0.0, 0.0, 0.0]

clients_lag = [0] * len(clients_ip) # how much instruction behind the Leader each corresponding follower is


class RobotsThreading:
    def __init__(self, robot_leader_ip, robot_clients_ip, robot_clients_lag, robot_tool, sleep_joints):
        # initialization attributes of the robots
        self.robot_leader_ip = robot_leader_ip
        self.robot_clients_ip = robot_clients_ip
        self.robot_clients_lag = robot_clients_lag
        self.robot_tool = robot_tool
        self.sleep_joints = sleep_joints

        # communication
        self.go = True
        self.desired_joint = sleep_joints

    # starting a thread of each robot
    def run(self):
        # start Leader thread
        robot_server_thread = self.RobotLeader(self.robot_leader_ip, self.robot_tool, self)
        robot_server_thread.start()

        # start followers
        robot_clients_threads = []
        for x in range(len(self.robot_clients_ip)):  # check if there are multiple followers or not
            robot_clients_threads.append(
                self.RobotFollower(self.robot_clients_ip[x], self.robot_tool, self, self.robot_clients_lag[x]))
            robot_clients_threads[x].start()

        input("press enter to exit")

        self.go = False
        robot_server_thread.join()
        for client_thread in robot_clients_threads:
            client_thread.join()
        robot_server_thread.client.set_learning_mode(True)
        for client_thread in robot_clients_threads:
            client_thread.client.set_learning_mode(True)
        exit()

    class RobotLeader(Thread):
        def __init__(self, robot_ip, robot_tool, parent):
            Thread.__init__(self)  # init this class has a thread
            self.robot_ip = robot_ip
            self.robot_tool = robot_tool
            self.parent = parent

            self.client = None

        def run(self):
            # initialize the robot
            self.client = NiryoRobot()
            self.client.connect(self.robot_ip)
            self.client.calibrate(CalibrateMode.AUTO)
            self.client.set_learning_mode(False)
            self.client.move_joints(*self.parent.desired_joint)
            self.client.set_learning_mode(True)

            time_interval = 15
            inv_time_interval = 1 / time_interval
            t = time.time()

            # main loop of the robot
            while self.parent.go:
                joints = self.client.get_joints()
                self.parent.desired_joint = joints
                # tps Limitation
                t = time.time() - t
                if t < inv_time_interval:
                    time.sleep(inv_time_interval - t)
                t = time.time()

    class RobotFollower(Thread):
        def __init__(self, robot_ip, robot_tool, parent, lag):
            Thread.__init__(self)  # init this class has a thread
            self.robot_ip = robot_ip
            self.robot_tool = robot_tool
            self.parent = parent
            self.lag = lag
            self.client = None

        def run(self):
            # initialize the robot
            self.client = NiryoRobot()
            self.client.connect(self.robot_ip)
            self.client.calibrate(CalibrateMode.AUTO)
            self.client.move_joints(*self.parent.desired_joint)

            # main loop of the robot
            jog_joints = [0, 0, 0, 0, 0, 0]
            # i=0
            while self.parent.go:

                joints = self.client.get_joints()
                jog_joints = [(desired - current)*0.4 for desired, current in zip(self.parent.desired_joint, joints)]

                try:
                    self.client.jog_joints(jog_joints)  # executing instruction
                except NiryoRobotException:
                    pass


main = RobotsThreading(leader_ip, clients_ip, clients_lag, tool, default_pose)
main.run()
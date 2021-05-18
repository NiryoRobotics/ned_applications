"""
This script aims to use the leap_motion to control a Ned with your hand.
To do use this script you will ned to set properly the leap_motion and the Ned.
All the information to run this script are given in Niryo's Documentation
"""

# Libraries
import os
import sys
import time
import math
import copy
import threading
from threading import *
import pygame
from pygame.locals import *
from pyniryo import *

# -------------------------------------------Part to change-----------------------------------------------
directory_name = os.path.join(os.path.dirname(__file__), "leaplib")
sys.path.insert(0, directory_name)

print(directory_name)

robot_ip = "127.0.0.1"  # change ip of the robot
window_y = 1000  # set height of the window if you need
# --------------------------------------------------------------------------------------------------------
from leaplib import Leap

if len(sys.argv) > 1:
    robot_ip = sys.argv[1]

# range of the robot compare to his height
limit = [(-1, 0), (-0.04, 0), (-0.031, 0.376), (+0.060, 0.412), (+0.112, 0.422), (+0.160, 0.427), (+0.212, 0.421),
         (+0.245, 0.412), (+0.311, 0.399), (+0.360, 0.371), (0.371, 0.355), (0.402, 0.322), (0.421, 0.279),
         (+0.436, 0.0), (+1, 0.0)]

window_x = int(window_y * 1.5)

client = NiryoRobot(robot_ip)
client.calibrate(CalibrateMode.AUTO)

client.move_pose(0.24, 0.0, 0.41, 0, 0, 0)
client.update_tool()
client.release_with_tool()
client.set_jog_control(True)


# convert robot pose to window x and y
def robot_to_screen(robot):
    screen = [
        robot[0] * 1.0 / -10e-1 * window_y + window_y / 2.0,
        robot[1] * 1.0 / -10e-1 * window_y + window_y / 2.0,
    ]
    screen.reverse()
    return screen


def screen_to_robot(screen):
    robot = [
        (screen[0] * 1.0 - window_y / 2) / window_y * -10e-1,
        (screen[1] * 1.0 - window_y / 2) / window_y * -10e-1,
    ]
    robot.reverse()
    return robot


def exit_game():
    client.set_jog_control(False)
    client.set_learning_mode(True)
    client.close_connection()
    exit()


# rendering and control thread
class Rendering(threading.Thread):
    def __init__(self):
        Thread.__init__(self)  # init this class has a thread
        self.arm_screen = [0, 0]
        self.screen_robot = [0, 0]
        self.surface = None
        self.logo = None
        self.logo_big = None
        self.my_font = None
        self.font_size = None
        self.is_rad = True
        self.is_meter = True
        self.name_inc = 1
        self.buttons = None
        self.buttons_description = None
        self.m_buttons = None
        self.m_buttons_descriptions = None
        self.help_menu = True

        # pygame init
        pygame.init()
        self.font_size = int(0.040 * window_y)
        self.my_font = pygame.font.Font("data/OpenSans-Regular.ttf", self.font_size)
        self.my_font_ExtraBold = pygame.font.Font("data/OpenSans-ExtraBold.ttf", self.font_size)
        self.my_font_Bold = pygame.font.Font("data/OpenSans-Bold.ttf", self.font_size)
        self.my_font_Bold_large = pygame.font.Font("data/OpenSans-Bold.ttf", self.font_size * 2)
        self.my_font_small = pygame.font.Font("data/OpenSans-Light.ttf", int(self.font_size / 2))

    class SampleListener(Leap.Listener):

        def on_frame(self, controller):
            global arm
            frame = controller.frame()
            hand = frame.hands.frontmost
            if not hand.is_valid:
                return
            arm[0] = -hand.palm_position[2] / 1000.0 * 1.5
            arm[1] = -hand.palm_position[0] / 1000.0 * 1.5
            arm[2] = hand.palm_position[1] / 1000.0 - 0.1

            normal = hand.palm_normal
            direction = hand.direction
            direction = [-direction[0], -direction[1], -direction[2]]

            # yaw
            v2d = [normal[0], normal[2]]  # 2d vector
            d2d = (v2d[0] ** 2 + v2d[1] ** 2) ** 0.5  # length
            if d2d == 0:
                return
            v2d = [v2d[0] / d2d, v2d[1] / d2d]  # normalise
            yaw = math.acos(-v2d[1])
            if v2d[0] < 0:
                yaw *= -1

            # pitch
            v2d = [(normal[0] ** 2 + normal[2] ** 2) ** 0.5, normal[1]]  # 2d vector
            d2d = (v2d[0] ** 2 + v2d[1] ** 2) ** 0.5  # length
            if d2d == 0:
                return
            v2d = [v2d[0] / d2d, v2d[1] / d2d]  # normalise
            pitch = math.acos(v2d[0])
            if v2d[1] < 0:
                pitch *= -1

            # cancel yaw
            v3d = [
                direction[0] * math.cos(-yaw) - direction[2] * math.sin(-yaw),
                direction[1],
                direction[0] * math.sin(-yaw) + direction[2] * math.cos(-yaw)
            ]

            # cancel pitch
            v3d = [
                v3d[0],
                v3d[1] * math.cos(-pitch) - v3d[2] * math.sin(-pitch),
                v3d[1] * math.sin(-pitch) + v3d[2] * math.cos(-pitch)
            ]
            # calc roll
            roll = math.acos(-v3d[1])
            if v3d[0] < 0:
                roll *= -1

            global tool_update
            global tool_open
            if hand.grab_strength > 0.5 and tool_open:
                tool_open = not tool_open
                tool_update = True
                print("closed")
            elif hand.grab_strength < 0.5 and not tool_open:
                tool_open = not tool_open
                tool_update = True
                print("open")
            arm[3] = -roll
            arm[4] = -pitch
            arm[5] = -yaw

    @staticmethod
    def img_resize(img, ratio, xy=0):
        size = img.get_size()
        desired_x = (window_x - window_y) * ratio
        size = [size[0], size[1]]
        ratio = desired_x / size[xy]
        size[0] *= ratio
        size[1] *= ratio
        size = [int(size[0]), int(size[1])]
        img = pygame.transform.smoothscale(img, size)
        return img

    def run(self):
        # leap motion init
        # Create a sample listener and controller
        listener = self.SampleListener()
        controller = Leap.Controller()
        # Have the sample listener receive events from the controller
        print("leap listener...")
        controller.add_listener(listener)

        # load keyboard icons
        k_escape = pygame.image.load("data/ESCAPE300px.png")
        k_escape = self.img_resize(k_escape, 0.1, 1)
        k_p = pygame.image.load("data/P300px.png")
        k_p = self.img_resize(k_p, 0.1, 1)
        k_x = pygame.image.load("data/X300px.png")
        k_x = self.img_resize(k_x, 0.1, 1)
        k_ctrl_s = pygame.image.load("data/CTRL_S300px.png")
        k_ctrl_s = self.img_resize(k_ctrl_s, 0.1, 1)

        self.buttons = [k_escape, k_p, k_x, k_ctrl_s]
        self.buttons_description = ["Quit", "Stand By", "switch Rad/Deg", "Save pose to robot"]

        # load leapmotion icons
        m_open = pygame.image.load("data/m_open.png")
        m_open = self.img_resize(m_open, 0.4, 1)

        m_close = pygame.image.load("data/m_close.png")
        m_close = self.img_resize(m_close, 0.4, 1)

        m_normal = pygame.image.load("data/m_normal.png")
        m_normal = self.img_resize(m_normal, 0.4, 1)

        self.m_buttons = [
            [m_open, m_close],
            [m_normal]
        ]

        self.m_buttons_descriptions = ["Open/Close tool", "The tool is in your hand"]

        # load logos
        self.logo_big = pygame.image.load('data/logo_big.png')
        self.logo_big = self.img_resize(self.logo_big, 0.9)
        self.logo = pygame.image.load('data/logo.png')
        pygame.display.set_icon(self.logo)

        self.surface = pygame.display.set_mode((window_x, window_y))
        pygame.display.set_caption("Niryo v3 mouse control")
        print("pygame init")

        t = time.time()
        fps = 60
        while 1:
            self.input()
            self.draw_gui()
            # fps Limiter
            t = time.time() - t
            if t < 1 / fps:
                time.sleep(1 / fps - t)
            t = time.time()

    @staticmethod
    def close_gui():
        global running
        global pause
        running = False
        pause = False
        exit()

    def input(self):
        global arm
        global tool_open
        global save_pos
        global pause

        for event in pygame.event.get():
            if event.type == QUIT:
                self.close_gui()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:
                    arm[2] -= 0.01
                if event.button == 5:
                    arm[2] += 0.01
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.close_gui()
                if event.key == pygame.K_x:
                    self.is_rad = not self.is_rad
                    self.is_meter = not self.is_meter
                if event.key == pygame.K_p:
                    pause = not pause
                if event.mod & pygame.KMOD_CTRL and event.key == pygame.K_s:
                    save_pos = True

        tmp = robot_to_screen(arm)
        self.arm_screen = [int(tmp[0]), int(tmp[1])]

        tmp = robot_to_screen(robot_pose)
        self.screen_robot = int(tmp[0]), int(tmp[1])

    def draw_gui(self):
        self.surface.fill((0x20, 0x35, 0x67))

        i = 0
        while limit[i][0] < arm[2] and i < len(limit):
            i += 1
        ratio = (arm[2] - limit[i - 1][0]) / (limit[i][0] - limit[i - 1][0])
        arm_range = (limit[i][1] * ratio + limit[i - 1][1] * (1 - ratio))

        # draw white area
        pygame.draw.circle(self.surface, (0xFF, 0xFF, 0xFF), (int(window_y / 2), int(window_y / 2)),
                           int(robot_to_screen((-arm_range, 0))[1] - window_y / 2))
        mid = [window_y / 2, window_y / 2]
        angle = 2.96705972839
        p1 = [window_y / 2 - math.sin(angle) * window_y, window_y / 2 - math.cos(angle) * window_y]
        p2 = [window_y / 2 - math.sin(-angle) * window_y, window_y / 2 - math.cos(-angle) * window_y]
        pygame.draw.polygon(self.surface, (0x20, 0x35, 0x67), [mid, p1, p2], 0)
        pygame.draw.circle(self.surface, (0x20, 0x35, 0x67), (int(window_y / 2), int(window_y / 2)),
                           int(robot_to_screen((-0.0887, 0))[1] - window_y / 2))

        # draw logo
        size = self.logo_big.get_size()
        self.surface.blit(self.logo_big, (window_y + (window_x - window_y) / 2 - size[0] / 2, 0.1 * window_y))
        pos_y = (0.2 * window_y + size[1])

        txt1 = ["x  : ", "y  : ", "z  : ", "rx : ", "ry : ", "rz : "]
        txt2 = ["m", "m", "m", "rad", "rad", "rad"]
        txt3 = ["j1 : ", "j2 : ", "j3 : ", "j4 : ", "j5 : ", "j6 : "]
        txt4 = ["rad", "rad", "rad", "rad", "rad", "rad"]

        # ms Display
        tps = (robot_tps[-1] - robot_tps[0]) / len(robot_tps)
        if tps == 0:
            tps = 0.001
        string = str(int(tps * 10 ** 3))
        string = string[0:5]
        if robot_tps[-1] < time.time() - 1:
            font_gui = self.my_font_small.render("ms : " + string, False, (0xFF, 0x00, 0x00))
        else:
            font_gui = self.my_font_small.render("ms : " + string, False, (0xFF, 0xFF, 0xFF))
        self.surface.blit(font_gui, (int(self.font_size / 4), 0))

        # change units
        robot_pose_tmp = copy.deepcopy(robot_pose)
        robot_joints_tmp = copy.deepcopy(robot_joints)
        if not self.is_meter:
            txt2[0:3] = ["mm", "mm", "mm"]
            for cpt in range(0, 3):
                robot_pose_tmp[cpt] = robot_pose_tmp[cpt] * 10 ** 3
        if not self.is_rad:
            txt2[3:6] = ["deg", "deg", "deg"]
            txt4[0:6] = ["deg", "deg", "deg", "deg", "deg", "deg"]
            for cpt in range(3, 6):
                robot_pose_tmp[cpt] = robot_pose_tmp[cpt] * 180 / math.pi
            for cpt in range(0, 6):
                robot_joints_tmp[cpt] = robot_joints_tmp[cpt] * 180 / math.pi

        # POSE Display
        font_gui = self.my_font_Bold.render("POSE", False, (0xFF, 0xFF, 0xFF))
        size = font_gui.get_size()
        self.surface.blit(font_gui, (window_y + (window_x - window_y) / 2 - size[0] / 2, pos_y))
        pos_y += self.font_size
        align_x = (0.0, 0.0)
        for cpt in range(len(txt1)):
            string = str(robot_pose_tmp[cpt])
            if string[0] != '-':
                string = " " + string
            string = string[0:6]
            font_gui = self.my_font.render(txt1[cpt] + string + txt2[cpt], False, (0xFF, 0xFF, 0xFF))
            if cpt == 0:
                align_x = font_gui.get_size()
            self.surface.blit(font_gui, (window_y + (window_x - window_y) / 2 - align_x[0] / 2, pos_y))
            pos_y += self.font_size
        pos_y += self.font_size

        # JOINTS Display
        font_gui = self.my_font_Bold.render("JOINTS", False, (0xFF, 0xFF, 0xFF))
        size = font_gui.get_size()
        self.surface.blit(font_gui, (window_y + (window_x - window_y) / 2 - size[0] / 2, pos_y))
        pos_y += self.font_size
        for cpt in range(len(txt1)):
            string = str(robot_joints_tmp[cpt])
            if string[0] != '-':
                string = " " + string
            string = string[0:6]
            font_gui = self.my_font.render(txt3[cpt] + string + txt4[cpt], False, (0xFF, 0xFF, 0xFF))
            self.surface.blit(font_gui, (window_y + (window_x - window_y) / 2 - align_x[0] / 2, pos_y))
            pos_y += self.font_size

        # display Pause / help menu
        size_o = self.buttons[1].get_size()
        if pause:
            self.surface.fill((0x80, 0x80, 0x80),
                              rect=(window_x / 10, window_y / 10, window_x / 10 * 8, window_y / 10 * 8))

            font_gui = self.my_font_Bold_large.render("STAND BY", False, (0xFF, 0xFF, 0xFF))
            size = font_gui.get_size()
            self.surface.blit(font_gui, (window_x / 2 - size[0] / 2, window_y / 10 + size[1] / 2))

            # draw keyboard shortcuts
            pos_x = window_x / 10 * 3 + size_o[0] * 0.25
            pos_y = window_y / 10 + size[1] / 2 + size[1] * 1.25
            for cpt in range(len(self.buttons)):
                size = self.buttons[cpt].get_size()

                self.surface.blit(self.buttons[cpt], (window_x / 2 - size[0] - size_o[0] * 0.2, pos_y))
                pos_x += size[0] + size_o[0] * 0.25

                font_gui = self.my_font.render(self.buttons_description[cpt], False, (0xFF, 0xFF, 0xFF))
                self.surface.blit(font_gui, (window_x / 2 + size_o[0] * 0.2, pos_y))
                pos_y += size[1] + size_o[1] * 0.5
                pos_x -= size[0] + size_o[0] * 0.25

            # draw mouse controls
            pos_y += size_o[1] * 0.5
            pos_x = window_x / (1 + len(self.m_buttons))

            for cpt in range(len(self.m_buttons)):
                xx = int(time.time()) % len(self.m_buttons[cpt])
                self.surface.blit(self.m_buttons[cpt][xx], (pos_x - self.m_buttons[cpt][0].get_size()[0] / 2, pos_y))
                font_gui = self.my_font.render(self.m_buttons_descriptions[cpt], False, (0xFF, 0xFF, 0xFF))
                self.surface.blit(font_gui, (pos_x - font_gui.get_size()[0] / 2, pos_y + 0.2 * window_y))
                pos_x += window_x / (1 + len(self.m_buttons))

        pos_y = window_y - size_o[1] * 1.5
        size = self.buttons[1].get_size()
        self.surface.blit(self.buttons[1], (window_y / 2 - size[0] - size_o[0] * 0.2, pos_y))
        font_gui = self.my_font.render(self.buttons_description[1], False, (0xFF, 0xFF, 0xFF))
        self.surface.blit(font_gui, (window_y / 2 + size_o[0] * 0.2, pos_y))

        # draw arm pose
        if jog_failed:
            color = [0xFF, 0x00, 0x00]
        else:
            color = [0x00, 0xFF, 0x00]
        pygame.draw.circle(self.surface, (0xFF, 0xFF, 0xFF), self.screen_robot, int(robot_pose[2] * 37.5 + 5) + 2)
        pygame.draw.line(self.surface, color, self.arm_screen, self.screen_robot, 2)
        if int(arm[2] * 37.5 + 5) > 0:
            pygame.draw.circle(self.surface, color, self.arm_screen, int(arm[2] * 37.5 + 5))
        if int(robot_pose_tmp[2] * 37.5 + 5) > 0:
            pygame.draw.circle(self.surface, (0x2A, 0xAB, 0xF2), self.screen_robot, int(robot_pose[2] * 37.5 + 5))

        pygame.display.flip()


pose = client.get_pose().to_list()

arm = [pose[0], pose[1], pose[2], 0, -math.pi / 2, 0]

# value share between the GUI and main Thread
jog_failed = False  # last jog failed ?
tool_open = True  # state of the tool
tool_update = False  # gui ask for tool action
running = True  # turn to false when the program is closed finished
save_pos = False  # ask the main thread to save robot pos
pause = True  # is the main thread on pause?

robot_pose = [0.0] * 6
robot_joints = [0.0] * 6

robot_tps = [time.time()]

name_inc = 1
pose_names = client.get_saved_pose_list()
while "mouse_control_" + str(name_inc) in pose_names:
    name_inc += 1

# start the GUI thread
render_thread = Rendering()
render_thread.start()

while running:

    while pause:
        time.sleep(0.25)

    try:
        arm_target_joints = client.inverse_kinematics(arm)
    except NiryoRobotException:
        continue
    robot_pose = client.get_pose().to_list()
    robot_joints = client.get_joints()

    # Calculate Jog value on each joints
    jog_pose = [arm_v - robot_joint for arm_v, robot_joint in zip(arm_target_joints, robot_joints)]

    # Ensure that Jog values are in the bounds [-0.1, 0.1]
    for x in range(len(jog_pose)):
        jog_pose[x] = min(0.1, max(-0.1, jog_pose[x] * 0.2))

    jog_inc = [0, 0, 0, 0, 0, 0]

    try:
        client.jog_joints(jog_pose)
        jog_failed = 0
    except NiryoRobotException:
        print("jog failed", jog_pose)
        jog_failed = 1
    robot_tps.append(time.time())
    if len(robot_tps) > 15:
        robot_tps.pop(0)

    if tool_update:
        print("switch tool")
        tool_update = False
        client.set_jog_control(False)

        if tool_open:
            client.release_with_tool()
        else:
            client.grasp_with_tool()
        client.set_jog_control(True)
    if save_pos:
        save_pos = False
        client.save_pose("mouse_control_" + str(name_inc), client.get_pose())
        print("pose saved as :", "mouse_control_" + str(name_inc))
        pose_names = client.get_saved_pose_list()
        while "mouse_control_" + str(name_inc) in pose_names:
            name_inc += 1
exit_game()

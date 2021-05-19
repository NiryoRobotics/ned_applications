import time
import math
import copy
import threading
from threading import *
import pygame
from pygame.locals import *

from pyniryo import *

robot_ip = "127.1.0.0"
window_y = 1000

# range of the robot compare to his height
# This limit variable is used to calculate the radius of the white circle displayed in the graphic interface
# according to the current height of the end effector.
# Higher the end's effector is (Or low, if Ned can access negative height) (represented by the first element of each
# tuple), smaller the radius of the white circle will be (represented by the second element
# of each tuple). When the end effector reach is optimal height, the accessible area is the larger, so is the radius.
limit = [(-1, 0), (-0.04, 0), (-0.031, 0.376), (+0.060, 0.412), (+0.112, 0.422), (+0.160, 0.427), (+0.212, 0.421),
         (+0.245, 0.412), (+0.311, 0.399), (+0.360, 0.371), (0.371, 0.355), (0.402, 0.322), (0.421, 0.279),
         (+0.436, 0.0), (+1, 0.0)]
window_x = int(window_y * 1.5)

client = NiryoRobot(robot_ip)
client.calibrate(CalibrateMode.AUTO)
client.move_joints(0.0, 0.0, 0.0, 0.0, -1.57, 0.0)
client.update_tool()
client.release_with_tool()
client.set_jog_control(True)


# - Pygame methods

# convert robot pose to window x and y
def robot_to_screen(robot):
    screen = [
        robot[0] / -10e-1 * window_y + window_y / 2,
        robot[1] / -10e-1 * window_y + window_y / 2,
    ]
    screen.reverse()
    return screen


def screen_to_robot(screen):
    robot = [
        (screen[0] - window_y / 2) / window_y * -10e-1,
        (screen[1] - window_y / 2) / window_y * -10e-1,
    ]
    robot.reverse()
    return robot


def exit_game():
    client.set_jog_control(False)
    client.set_learning_mode(True)
    client.close_connection()
    exit()


# rendering thread of the program
class Rendering(threading.Thread):
    def __init__(self):
        Thread.__init__(self)  # init this class has a thread
        self.arm_screen = [0, 0]
        self.screen_robot = [0, 0]
        self.surface = None
        self.logo = None
        self.logo_big = None
        self.myfont = None
        self.font_size = None
        self.is_rad = True
        self.is_meter = True
        self.name_inc = 1
        self.buttons = None
        self.buttons_description = None
        self.m_buttons = None
        self.m_buttons_descriptions = None
        self.help_menu = True

        # pygame init and variables definition
        pygame.init()
        self.font_size = int(0.040 * window_y)
        self.myfont = pygame.font.Font("data/OpenSans-Regular.ttf", self.font_size)
        self.myfont_ExtraBold = pygame.font.Font("data/OpenSans-ExtraBold.ttf", self.font_size)
        self.myfont_Bold = pygame.font.Font("data/OpenSans-Bold.ttf", self.font_size)
        self.myfont_Bold_large = pygame.font.Font("data/OpenSans-Bold.ttf", self.font_size * 2)
        self.myfont_small = pygame.font.Font("data/OpenSans-Light.ttf", int(self.font_size / 2))

    @staticmethod
    def img_resize(img, ratio, xy=0):
        size = img.get_size()

        desired_x = (window_x - window_y) * ratio
        size = [*size]

        ratio = desired_x / size[xy]

        size[0] *= ratio
        size[1] *= ratio

        size = [int(size[0]), int(size[1])]
        img = pygame.transform.smoothscale(img, size)
        return img

    def run(self):
        # load keybord icones
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

        # load mouse icones
        m_left = pygame.image.load("data/left300px.png")
        m_left = self.img_resize(m_left, 0.1, 1)
        m_scroll = pygame.image.load("data/Scroll300px.png")
        m_scroll = self.img_resize(m_scroll, 0.1, 1)
        m_right = pygame.image.load("data/right300px.png")
        m_right = self.img_resize(m_right, 0.1, 1)

        self.m_buttons = [m_left, m_scroll, m_right]
        self.m_buttons_descriptions = ["open/close gripper", "up/down", ""]

        # load logos
        self.logo_big = pygame.image.load('data/logo_big.png')
        self.logo_big = self.img_resize(self.logo_big, 0.9)
        self.logo = pygame.image.load('data/logo.png')
        pygame.display.set_icon(self.logo)

        self.surface = pygame.display.set_mode((window_x, window_y))
        pygame.display.set_caption("Niryo Ned mouse control")
        print("pygame init")

        t = time.time()
        fps = 60
        while 1:
            self.input()
            self.draw_gui()
            # FPS Limitor
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

        pos = list(pygame.mouse.get_pos())

        for event in pygame.event.get():
            if event.type == QUIT:
                self.close_gui()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:
                    arm[2] -= 0.01
                if event.button == 5:
                    arm[2] += 0.01
                if event.button == 1:
                    global tool_update
                    tool_open = not tool_open
                    tool_update = True
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

        arm[0:2] = screen_to_robot(pos)[0:2]
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
        angle = 2.585822241882568
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
        tps = (robot_TPS[-1] - robot_TPS[0]) / len(robot_TPS)
        if tps == 0:
            tps = 0.001
        string = str(int(tps * 10 ** 3))
        string = string[0:5]
        if robot_TPS[-1] < time.time() - 1:
            font = self.myfont_small.render("ms : " + string, False, (0xFF, 0x00, 0x00))
        else:
            font = self.myfont_small.render("ms : " + string, False, (0xFF, 0xFF, 0xFF))
        self.surface.blit(font, (int(self.font_size / 4), 0))

        # change units
        robot_pose_tmp = copy.deepcopy(robot_pose)
        robot_joints_tmp = copy.deepcopy(robot_joints)
        if not self.is_meter:
            txt2[0:3] = ["mm", "mm", "mm"]
            for x in range(0, 3):
                robot_pose_tmp[x] = robot_pose_tmp[x] * 10 ** 3
        if not self.is_rad:
            txt2[3:6] = ["°", "°", "°"]
            txt4[0:6] = ["°", "°", "°", "°", "°", "°"]
            for x in range(3, 6):
                robot_pose_tmp[x] = robot_pose_tmp[x] * 180 / math.pi
            for x in range(0, 6):
                robot_joints_tmp[x] = robot_joints_tmp[x] * 180 / math.pi

        # POSE Display
        font = self.myfont_Bold.render("POSE", False, (0xFF, 0xFF, 0xFF))
        size = font.get_size()
        self.surface.blit(font, (window_y + (window_x - window_y) / 2 - size[0] / 2, pos_y))
        pos_y += self.font_size
        align_x = 0
        for x in range(len(txt1)):
            string = str(robot_pose_tmp[x])
            if string[0] != '-':
                string = " " + string
            string = string[0:6]
            font = self.myfont.render(txt1[x] + string + txt2[x], False, (0xFF, 0xFF, 0xFF))

            if x == 0:
                align_x = font.get_size()
            self.surface.blit(font, (window_y + (window_x - window_y) / 2 - align_x[0] / 2, pos_y))
            pos_y += self.font_size
        pos_y += self.font_size

        # JOINTS Display
        font = self.myfont_Bold.render("JOINTS", False, (0xFF, 0xFF, 0xFF))
        size = font.get_size()
        self.surface.blit(font, (window_y + (window_x - window_y) / 2 - size[0] / 2, pos_y))
        pos_y += self.font_size
        for x in range(len(txt1)):
            string = str(robot_joints_tmp[x])
            if string[0] != '-':
                string = " " + string
            string = string[0:6]
            font = self.myfont.render(txt3[x] + string + txt4[x], False, (0xFF, 0xFF, 0xFF))
            self.surface.blit(font, (window_y + (window_x - window_y) / 2 - align_x[0] / 2, pos_y))
            pos_y += self.font_size

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

        # display Pause / help menu
        size_o = self.buttons[1].get_size()
        if pause:
            self.surface.fill((0x80, 0x80, 0x80),
                              rect=(window_x / 10, window_y / 10, window_x / 10 * 8, window_y / 10 * 8))

            font = self.myfont_Bold_large.render("STAND BY", False, (0xFF, 0xFF, 0xFF))
            size = font.get_size()
            self.surface.blit(font, (window_x / 2 - size[0] / 2, window_y / 10 + size[1] / 2))

            # draw keyborad shortcuts
            pos_x = window_x / 10 * 3 + size_o[0] * 0.25
            pos_y = window_y / 10 + size[1] / 2 + size[1] * 1.25
            for x in range(len(self.buttons)):
                size = self.buttons[x].get_size()

                self.surface.blit(self.buttons[x], (window_x / 2 - size[0] - size_o[0] * 0.2, pos_y))
                pos_x += size[0] + size_o[0] * 0.25

                font = self.myfont.render(self.buttons_description[x], False, (0xFF, 0xFF, 0xFF))
                self.surface.blit(font, (window_x / 2 + size_o[0] * 0.2, pos_y))
                pos_y += size[1] + size_o[1] * 0.5
                pos_x -= size[0] + size_o[0] * 0.25

            # draw mouse controls
            pos_y += size_o[1] * 0.5

            pos_x = window_x / 2
            size = self.m_buttons[0].get_size()
            pos_x -= size[0] + size_o[0] * 0.05
            self.surface.blit(self.m_buttons[0], (pos_x, pos_y))
            pos_x -= size_o[0] * 0.25

            font = self.myfont.render(self.m_buttons_descriptions[0], False, (0xFF, 0xFF, 0xFF))
            size = font.get_size()
            self.surface.blit(font, (pos_x - size[0], pos_y))

            pos_x = window_x / 2 + size_o[0] * 0.05
            size = self.m_buttons[1].get_size()
            self.surface.blit(self.m_buttons[1], (pos_x, pos_y))
            pos_x += size[0] + size_o[0] * 0.5

            font = self.myfont.render(self.m_buttons_descriptions[1], False, (0xFF, 0xFF, 0xFF))
            size = font.get_size()
            self.surface.blit(font, (pos_x, pos_y))
            pos_x += size[0] + size_o[0] * 0.5

        pos_y = window_y - size_o[1] * 1.5
        size = self.buttons[1].get_size()
        self.surface.blit(self.buttons[1], (window_y / 2 - size[0] - size_o[0] * 0.2, pos_y))
        font = self.myfont.render(self.buttons_description[1], False, (0xFF, 0xFF, 0xFF))
        self.surface.blit(font, (window_y / 2 + size_o[0] * 0.2, pos_y))
        pygame.display.flip()


# - Process methods

pose = client.get_pose().to_list()

arm = list(pose[0:3])

# value share between the GUI and main Thread
jog_failed = False  # last jog failed ?
tool_open = True  # state of the tool
tool_update = False  # gui ask for tool action
running = True  # turn to fasle when the program is closed finished
save_pos = False  # ask the main thread to save robot pos
pause = True  # is the main thread on pause?

robot_pose = [0.0] * 6
robot_joints = [0.0] * 6

robot_TPS = [time.time()]

name_inc = 1
pose_names = client.get_saved_pose_list()
while "mouse_control_" + str(name_inc) in pose_names:
    name_inc += 1

# start the GUI thread
render_thread = Rendering()
render_thread.start()

while running:

    robot_pose = client.get_pose().to_list()
    robot_joints = client.get_joints()

    # calculate jog value
    jog_pose = [
        arm[0] - robot_pose[0],
        arm[1] - robot_pose[1],
        arm[2] - robot_pose[2],
        0,
        0,
        0
    ]
    for x in range(len(jog_pose)):
        jog_pose[x] *= 0.2

    jog_inc = [0, 0, 0, 0, 0, 0]

    # limit max jog len
    max_d = 0.05
    dxy = (jog_pose[0] ** 2 + jog_pose[1] ** 2 + jog_pose[2] ** 2) ** 0.5
    if dxy > max_d:
        jog_pose[0] /= dxy / max_d
        jog_pose[1] /= dxy / max_d
        jog_pose[2] /= dxy / max_d

    try:
        client.jog_pose(*jog_pose)
        jog_failed = 0
    except NiryoRobotException:
        jog_failed = 1
    robot_TPS.append(time.time())
    if len(robot_TPS) > 15:
        robot_TPS.pop(0)

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

    while pause:
        time.sleep(0.5)

exit_game()

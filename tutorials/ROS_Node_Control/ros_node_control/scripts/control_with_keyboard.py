#!/usr/bin/env python

# Libs
import rospy
from pynput import keyboard

# Messages
from niryo_robot_msgs.msg import CommandStatus
from std_msgs.msg import Bool
from niryo_robot_msgs.msg import HardwareStatus

# Services
from niryo_robot_commander.srv import JogShift, JogShiftRequest
from niryo_robot_msgs.srv import SetBool, SetInt


class JogClient:
    def __init__(self):
        rospy.init_node('jog_client_node', anonymous=True)

        self.__jog_enabled = False
        self.__jog_enabled_subscriber = rospy.Subscriber('/niryo_robot/jog_interface/is_enabled',
                                                         Bool, self.__callback_subscriber_jog_enabled,
                                                         queue_size=1)
        self.current_learning_mode = True
        rospy.Subscriber('/niryo_robot/learning_mode/state', Bool, self.__update_learning_state)

        # Shifting values
        self.shift_pose_value = 0.01
        self.shift_angle_value = 0.05

        # check if calibration is needed and deactivate learning mode
        self.check_calibration()
        if self.current_learning_mode:
            self.set_learning_mode(False)

    # - CALLBACKS
    def __update_learning_state(self, data):
        self.current_learning_mode = data.data

    def __callback_subscriber_jog_enabled(self, ros_data):
        self.__jog_enabled = ros_data.data

    # - METHODS FOR INITIALISATION
    @staticmethod
    def set_learning_mode(bool_):
        # deactivate learning mode to control with jog
        try:
            rospy.wait_for_service('/niryo_robot/learning_mode/activate', 2)
            service = rospy.ServiceProxy('/niryo_robot/learning_mode/activate', SetBool)
            result = service(bool_)
            if result.status != CommandStatus.SUCCESS:
                return CommandStatus.ABORTED, "Cannot change learning mode"

            rospy.sleep(0.1)
        except (rospy.ROSException, rospy.ServiceException):
            return CommandStatus.ABORTED, "Error while trying to turn off/on learning mode"

    @staticmethod
    def check_calibration():
        hw_status = rospy.wait_for_message('/niryo_robot_hardware_interface/hardware_status',
                                           HardwareStatus, timeout=5)
        if not hw_status.calibration_needed:
            return CommandStatus.SUCCESS, "Calibration not needed"
        # auto calibration
        calib_type_int = 1
        try:
            rospy.wait_for_service('/niryo_robot/joints_interface/calibrate_motors', 2)
            service = rospy.ServiceProxy('/niryo_robot/joints_interface/calibrate_motors', SetInt)
            response = service(calib_type_int)
            rospy.sleep(0.1)
        except (rospy.ROSException, rospy.ServiceException):
            return CommandStatus.ABORTED, "Error while trying to calibrate motors"

        if response.status < 0:
            return CommandStatus.ABORTED, "Could not launch motor calibration"
        # self.__check_result_status(result) #if result.status < 0, throw issue
        # Wait until calibration start
        rospy.sleep(0.2)
        calibration_finished = False
        while not calibration_finished:
            try:
                hw_status = rospy.wait_for_message('/niryo_robot_hardware_interface/hardware_status',
                                                   HardwareStatus, timeout=5)
                if not (hw_status.calibration_needed or hw_status.calibration_in_progress):
                    calibration_finished = True
                else:
                    rospy.sleep(0.1)
            except rospy.ROSException:
                return CommandStatus.ABORTED, "Error from Hardware Interface while trying to calibrate motors"

        # Little delay to be sure calibration is over
        rospy.sleep(0.5)

    # - JOG SHIFT METHODS
    def set_jog(self, set_bool):
        if set_bool == self.__jog_enabled:
            return CommandStatus.SUCCESS, "Already enable"
        rospy.wait_for_service('/niryo_robot/jog_interface/enable')
        try:
            enable_service = rospy.ServiceProxy('/niryo_robot/jog_interface/enable', SetBool)
            response = enable_service(set_bool)
        except rospy.ServiceException as e:
            raise Exception("Service call failed: {}".format(e))
        rospy.sleep(0.1)
        return response

    def ask_for_jog_shift(self, cmd, shift_values):
        init_time = rospy.get_time()
        if not self.__jog_enabled:
            self.set_jog(True)
        service_name = '/niryo_robot/jog_interface/jog_shift_commander'
        rospy.wait_for_service(service_name)
        try:
            jog_commander_service = rospy.ServiceProxy(service_name, JogShift)
            req = JogShiftRequest()
            req.cmd = cmd
            req.shift_values = shift_values
            response = jog_commander_service(req)
            print(response)
        except rospy.ServiceException as e:
            raise Exception("Service call failed: {}".format(e))
        rospy.sleep(0.15 - (rospy.get_time() - init_time))
        return response


# - KEYBOARD LISTENER METHODS
def on_press(key):
    global current_key
    try:
        if key == keyboard.Key.shift:
            print('shift key!')
            current_key = ''
        else:
            current_key = key.char
        # print('current key pressed', current_key)
    except AttributeError:
        print('special key {0} pressed'.format(
            key))
        current_key = ''


def on_release(key):
    global current_key
    current_key = ''
    if key == keyboard.Key.esc:
        return False
    if key == keyboard.Key.ctrl:
        return False
    elif key == keyboard.Key.shift:
        current_key = ''
    else:
        current_key = ''


if __name__ == "__main__":
    # Creating Client Object
    jc = JogClient()
    jc.set_jog(True)

    current_key = ''

    print "press CTRL or ESC  to finish the programm"

    # starts keyboard listener
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    while listener.is_alive():

        while current_key != '':
            print'key pressed : ', current_key

            # - SHIFT POSE
            if current_key == 'i':  # i : z axis (+)
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.POSE_SHIFT,
                                                shift_values=[0.0, 0.0, jc.shift_pose_value, 0.0, 0.0, 0.0])

            elif current_key == 'k':  # k : z axis (-)
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.POSE_SHIFT,
                                                shift_values=[0.0, 0.0, -jc.shift_pose_value, 0.0, 0.0, 0.0])

            elif current_key == 'l':  # l : y axis (+)
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.POSE_SHIFT,
                                                shift_values=[0.0, jc.shift_pose_value, 0.0, 0.0, 0.0, 0.0])

            elif current_key == 'j':  # j : y axis (-)
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.POSE_SHIFT,
                                                shift_values=[0.0, -jc.shift_pose_value, 0.0, 0.0, 0.0, 0.0])

            elif current_key == 'p':  # p : x axis (+)
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.POSE_SHIFT,
                                                shift_values=[jc.shift_pose_value, 0.0, 0.0, 0.0, 0.0, 0.0])

            elif current_key == 'm':  # m : x axis (-)
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.POSE_SHIFT,
                                                shift_values=[-jc.shift_pose_value, 0.0, 0.0, 0.0, 0.0, 0.0])

            # - SHIFT JOINTS

            # Positive change
            if current_key == 'a':  # i : 1st Joint
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                                shift_values=[jc.shift_angle_value, 0.0, 0.0, 0.0, 0.0, 0.0])
            if current_key == 'z':  # i : 2nd Joint
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                                shift_values=[0.0, jc.shift_angle_value, 0.0, 0.0, 0.0, 0.0])
            if current_key == 'e':  # i : 3d Joint
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                                shift_values=[0.0, 0.0, jc.shift_angle_value, 0.0, 0.0, 0.0])
            if current_key == 'r':  # i : 4th Joint
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                                shift_values=[0.0, 0.0, 0.0, jc.shift_angle_value, 0.0, 0.0])
            if current_key == 't':  # i : 5th Joint
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                                shift_values=[0.0, 0.0, 0.0, 0.0, jc.shift_angle_value, 0.0])
            if current_key == 'y':  # i : 6th Joint
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                                shift_values=[0.0, 0.0, 0.0, 0.0, 0.0, jc.shift_angle_value])

            # Negative change
            if current_key == 'q':  # i : 1st Joint
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                                shift_values=[-jc.shift_angle_value, 0.0, 0.0, 0.0, 0.0, 0.0])
            if current_key == 's':  # i : 2nd Joint
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                                shift_values=[0.0, -jc.shift_angle_value, 0.0, 0.0, 0.0, 0.0])
            if current_key == 'd':  # i : 3d Joint
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                                shift_values=[0.0, 0.0, -jc.shift_angle_value, 0.0, 0.0, 0.0])
            if current_key == 'f':  # i : 4th Joint
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                                shift_values=[0.0, 0.0, 0.0, -jc.shift_angle_value, 0.0, 0.0])
            if current_key == 'g':  # i : 5th Joint
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                                shift_values=[0.0, 0.0, 0.0, 0.0, -jc.shift_angle_value, 0.0])
            if current_key == 'h':  # i : 6th Joint
                response = jc.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                                shift_values=[0.0, 0.0, 0.0, 0.0, 0.0, -jc.shift_angle_value])

    if not listener.running:
        print 'keyboard listener died'

    jc.set_learning_mode(True)

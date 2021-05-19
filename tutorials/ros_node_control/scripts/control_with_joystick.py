#!/usr/bin/env python

# Libs
import rospy
import numpy as np
import threading

# Messages
from niryo_robot_msgs.msg import CommandStatus
from std_msgs.msg import Bool
from niryo_robot_msgs.msg import HardwareStatus
from sensor_msgs.msg import Joy

# Services
from niryo_robot_commander.srv import JogShift, JogShiftRequest
from niryo_robot_msgs.srv import SetBool, SetInt


class JogClient:
    def __init__(self):
        rospy.init_node('jog_client_node', anonymous=True)

        self.__jog_enabled = False
        self.current_learning_mode = True

        # Subscribers
        self.__jog_enabled_subscriber = rospy.Subscriber('/niryo_robot/jog_interface/is_enabled',
                                                         Bool, self.__callback_subscriber_jog_enabled,
                                                         queue_size=1)
        rospy.Subscriber('/niryo_robot/learning_mode/state', Bool, self.__callback_update_learning_state)
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.buttonA = 0
        self.buttonB = 0
        self.buttonX = 0
        self.buttonY = 0

        self.zAxis = 0
        self.yAxis = 0
        self.xAxis = 0

        self.joint5 = 0
        self.joint6 = 0

        self.current_mode = "Pose"  # either "Pose" or "Joint" according to how you want to control Ned

        self.shift_values = []

        self.check_calibration()
        if self.current_learning_mode:
            self.set_learning_mode(False)

        self.thread_move = threading.Thread(target=self.move_arm)
        self.thread_move.daemon = True
        self.thread_move.start()

    # - CALLBACKS
    def __callback_update_learning_state(self, data):
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

    # - JOYSTICK METHODS
    def joy_callback(self, data):
        """
        update self.shift_values according to the current joystick state
        """
        listbutton = data.buttons
        listaxes = data.axes

        self.buttonA = listbutton[0]  # set learning mode
        self.buttonB = listbutton[1]
        self.buttonX = listbutton[2]
        self.buttonY = listbutton[3]

        # left joystick
        self.zAxis = listaxes[7]
        self.yAxis = listaxes[6]
        self.xAxis = listaxes[1]

        # right joystick
        self.joint5 = listaxes[3]
        self.joint6 = listaxes[4]

        if self.buttonA != 0:
            self.set_learning_mode(True)

        elif self.buttonB != 0:
            self.set_learning_mode(False)

        # Here, the number of axis are : z:2, y:1, x:0, joint5:3, joint6 : 5 Here, we update the self.shift_values list used by the move thread
        elif self.zAxis != 0:
            self.current_mode = "Pose"
            self.shift_values = self.create_shift_values(np.sign(self.zAxis), 2)

        elif self.yAxis != 0:
            self.current_mode = "Pose"
            self.shift_values = self.create_shift_values(np.sign(-1 * self.yAxis), 1)

        elif abs(self.xAxis) == 1.0:
            self.current_mode = "Pose"
            self.shift_values = self.create_shift_values(np.sign(self.xAxis), 0)

        elif abs(self.joint5) == 1.0:
            self.current_mode = "Joint"  # Meaning we will send a SHIFT_JOINT command
            self.shift_values = self.create_shift_values(np.sign(self.joint5), 4)

        elif abs(self.joint6) == 1.0:
            self.current_mode = "Joint"
            self.shift_values = self.create_shift_values(np.sign(self.joint6), 5)

    def create_shift_values(self, sign, axis):
        """
        return list of shift values according to the sign, current shifting mode (self.current_mode) and axis chosen
        """
        if self.current_mode == "Pose":
            shift_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            shift_values[axis] = sign * 0.01
            return shift_values
        elif self.current_mode == "Joint":
            shift_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            shift_values[axis] = sign * 0.05
            return shift_values

    # - THREADED METHOD
    def move_arm(self):
        """
        Move arm accordingly to the current joystick state
        """
        while 1:
            try:
                i = 0
                if self.zAxis != 0 or self.yAxis != 0 or abs(self.xAxis) == 1.0 or abs(self.joint5) == 1.0 or abs(
                        self.joint6) == 1.0:
                    if self.current_mode == "Pose":
                        if i < 3:  # just to make sure we don't ask for too many jog shift at a time
                            self.ask_for_jog_shift(cmd=JogShiftRequest.POSE_SHIFT,
                                                   shift_values=self.shift_values)
                            i += 1
                    elif self.current_mode == "Joint":
                        if i < 3:  # just to make sure we don't ask for too many jog shift at a time
                            self.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                                   shift_values=self.shift_values)
                            i += 1

            except KeyboardInterrupt:
                print('quit')
                self.thread_move.join()
                break


if __name__ == "__main__":
    # Creating Client Object
    jc = JogClient()
    jc.set_jog(True)

    current_key = ''
    rospy.spin()

#!/usr/bin/env python

# Libs
import rospy

# Messages
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_msgs.msg import HardwareStatus
from std_msgs.msg import Bool

# Services
from niryo_robot_commander.srv import JogShift, JogShiftRequest
from niryo_robot_msgs.srv import SetBool, SetInt

# -- Const
JOINT_SHIFT_VALUE = 0.05
POSE_SHIFT_VALUE = 0.01


class JogClient:
    def __init__(self):
        rospy.init_node('jog_client_node', anonymous=True)

        # -- Subscribers
        self.__jog_enabled = False
        rospy.Subscriber('/niryo_robot/jog_interface/is_enabled',
                         Bool, self.__callback_subscriber_jog_enabled,
                         queue_size=1)

        self.__current_learning_mode = True
        rospy.Subscriber('/niryo_robot/learning_mode/state',
                         Bool, self.__callback_update_learning_state,
                         queue_size=1)

        # - Calibrate if necessary
        self.check_calibration()
        if self.__current_learning_mode:
            self.set_learning_mode(False)

    def __callback_update_learning_state(self, data):
        self.__current_learning_mode = data.data

    def __callback_subscriber_jog_enabled(self, ros_data):
        self.__jog_enabled = ros_data.data

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
            resp = service(calib_type_int)
            rospy.sleep(0.1)
        except (rospy.ROSException, rospy.ServiceException):
            return CommandStatus.ABORTED, "Error while trying to calibrate motors"

        if resp.status < 0:
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

    def set_jog(self, set_bool):
        if set_bool == self.__jog_enabled:
            return CommandStatus.SUCCESS, "Already enable"
        rospy.wait_for_service('/niryo_robot/jog_interface/enable')
        try:
            enable_service = rospy.ServiceProxy(
                '/niryo_robot/jog_interface/enable', SetBool)
            resp = enable_service(set_bool)
        except rospy.ServiceException as e:
            raise Exception("Service call failed: {}".format(e))
        rospy.sleep(0.1)
        return resp

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
            resp = jog_commander_service(req)
            rospy.logdebug(resp)
        except rospy.ServiceException as e:
            raise Exception("Service call failed: {}".format(e))
        rospy.sleep(0.15 - (rospy.get_time() - init_time))
        return resp


if __name__ == "__main__":
    # Creating Client Object
    jc = JogClient()
    jc.set_jog(True)

    for sign in [1, -1]:
        # this 'stop' boolean is used so that if one jog shift is not doable, instead of doing the
        # next one, it abort the loop.
        # Indeed, doing some jog shifts (with this sign method we use) when some were skipped
        # can lead the robot to strange positions.
        stop = False
        for i in range(10):
            rospy.loginfo('Joints jog number {}'.format(i))
            response = jc.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                            shift_values=[0.0, 0.0, 0.0, 0.0, sign * JOINT_SHIFT_VALUE, 0.0])
            if response.status == CommandStatus.NO_PLAN_AVAILABLE:
                stop = True
                break
        if stop:
            break

    for sign in [1, -1]:
        stop = False
        for i in range(10):
            rospy.loginfo('Pose jog number {}'.format(i))
            response = jc.ask_for_jog_shift(cmd=JogShiftRequest.POSE_SHIFT,
                                            shift_values=[0.0, sign * POSE_SHIFT_VALUE, sign * POSE_SHIFT_VALUE,
                                                          0.0, 0.0, 0.0])
            if response.status == CommandStatus.NO_PLAN_AVAILABLE:
                stop = True
                break
        if stop:
            break

    jc.set_learning_mode(True)

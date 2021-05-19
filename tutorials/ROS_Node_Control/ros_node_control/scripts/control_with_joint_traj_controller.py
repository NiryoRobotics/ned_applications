#!/usr/bin/env python

# Libs
import rospy
from pynput import keyboard
import tf

# Messages
from niryo_robot_msgs.msg import CommandStatus
from std_msgs.msg import Bool
from niryo_robot_msgs.msg import HardwareStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest
from sensor_msgs.msg import JointState

# Services
from niryo_robot_msgs.srv import SetBool, SetInt
from moveit_msgs.srv import GetPositionIK

# -- Const
JOINT_SHIFT_VALUE = 0.05
POSE_SHIFT_VALUE = 0.01


class ControlJointCommand:
    def __init__(self):
        # - Direct publisher to joint controller
        self._joint_trajectory_publisher = rospy.Publisher('/niryo_robot_follow_joint_trajectory_controller/command',
                                                           JointTrajectory, queue_size=1)

        self.current_learning_mode = True
        rospy.Subscriber('/niryo_robot/learning_mode/state', Bool, self.__callback_update_learning_state)
        rospy.Subscriber('/joint_states', JointState, self.__callback_update_current_joint)

        self.listener_tf = tf.TransformListener()
        self.current_pose = PoseStamped()
        self.current_joint = JointState()
        self.key_list = ["i", "k", "l", "j", "p", "m", "a", "q", "z", "s", "e", "d"]  # List of keys to control Ned
        self.current_key = ''  # Current pressed key

        self.rate = rospy.Rate(10)

        # Check if calibration is needed and deactivate learning mode
        self.check_calibration()
        if self.current_learning_mode:
            self.set_learning_mode(False)

        # Start the thread that listen to keyboard and update "self.current_key"
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()

        # main loop
        while listener.is_alive():

            # When a key is pressed
            if self.current_key != '':
                rospy.loginfo('****')
                rospy.loginfo('key pressed : {}'.format(self.current_key))

                # Target pose = current pose
                target_pose = self.get_current_pose()

                if self.current_key not in self.key_list:
                    continue

                # Update target pose according to the key pressed
                if self.current_key == 'i':
                    target_pose.pose.position.z += POSE_SHIFT_VALUE

                elif self.current_key == 'k':
                    target_pose.pose.position.z -= POSE_SHIFT_VALUE

                elif self.current_key == 'l':
                    target_pose.pose.position.y += POSE_SHIFT_VALUE

                elif self.current_key == 'j':
                    target_pose.pose.position.y -= POSE_SHIFT_VALUE

                elif self.current_key == 'p':
                    target_pose.pose.position.x += POSE_SHIFT_VALUE

                elif self.current_key == 'm':
                    target_pose.pose.position.x -= POSE_SHIFT_VALUE

                elif self.current_key == 'a':
                    target_pose = self._change_orientation(target_pose, "roll", JOINT_SHIFT_VALUE)

                elif self.current_key == 'q':
                    target_pose = self._change_orientation(target_pose, "roll", -JOINT_SHIFT_VALUE)

                elif self.current_key == 'z':
                    target_pose = self._change_orientation(target_pose, "pitch", JOINT_SHIFT_VALUE)

                elif self.current_key == 's':
                    target_pose = self._change_orientation(target_pose, "pitch", -JOINT_SHIFT_VALUE)

                elif self.current_key == 'e':
                    target_pose = self._change_orientation(target_pose, "yaw", JOINT_SHIFT_VALUE)

                elif self.current_key == 'd':
                    target_pose = self._change_orientation(target_pose, "yaw", -JOINT_SHIFT_VALUE)

                solution, result = self.get_position_ik(target_pose)

                if result:
                    self.send_joint_trajectory(solution)
                    rospy.sleep(0.05)  # To avoid an issue regarding trajectory time

                elif not result:
                    rospy.logwarn("IK calculation impossible")
                    rospy.sleep(0.05)

            if not listener.running:
                rospy.logerr('listener died')
                break

            self.rate.sleep()

        self.set_learning_mode(True)

    # - CALLBACKS

    def __callback_update_current_joint(self, data):
        self.current_joint = data

    def __callback_update_learning_state(self, data):
        self.current_learning_mode = data.data

    # - PUBLIC METHODS

    def get_position_ik(self, target_pose):
        """
        Return a list of joints position (solution) from a PoseStamped (target_pose), using the /compute_ik service
        """
        try:
            rospy.wait_for_service('/compute_ik', 2)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Arm commander - Impossible to connect to IK service : " + str(e))
            return [], False

        service = rospy.ServiceProxy('/compute_ik', GetPositionIK)

        get_pose_ik = PositionIKRequest()
        get_pose_ik.group_name = "arm"
        get_pose_ik.ik_link_name = "tool_link"
        get_pose_ik.robot_state.joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        get_pose_ik.robot_state.joint_state.position = self.current_joint.position
        get_pose_ik.pose_stamped.pose = target_pose.pose

        response = service(get_pose_ik)
        solution = []
        if response.error_code.val == response.error_code.SUCCESS:
            for i in range(6):
                solution.append(response.solution.joint_state.position[i])
            result = True
        else:
            result = False
        return solution, result

    def send_joint_trajectory(self, solution):
        """
        Publish a joint trajectory on the follow joint trajectory controller using joints positions previously
        computed (solution)
        """
        joint_trajectory = JointTrajectory()
        joint_trajectory.header.stamp = rospy.Time.now()
        joint_trajectory.header.frame_id = "base_link"
        joint_trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        joint_trajectory_points = JointTrajectoryPoint()
        joint_trajectory_points.positions = solution
        joint_trajectory_points.time_from_start = rospy.Duration(0.1)

        joint_trajectory.points = [joint_trajectory_points]
        self._joint_trajectory_publisher.publish(joint_trajectory)

    def get_current_pose(self):
        """
        Return current PoseStamped of the end effector
        """
        current_pose = PoseStamped()
        (trans, rot) = self.listener_tf.lookupTransform('base_link', 'tool_link', rospy.Time(0))
        current_pose.header.frame_id = "tool_link"
        current_pose.header.stamp = rospy.Time(0)
        current_pose.pose.position.x = trans[0]
        current_pose.pose.position.y = trans[1]
        current_pose.pose.position.z = trans[2]
        current_pose.pose.orientation.x = rot[0]
        current_pose.pose.orientation.y = rot[1]
        current_pose.pose.orientation.z = rot[2]
        current_pose.pose.orientation.w = rot[3]
        return current_pose

    # - Computational methods

    def _change_orientation(self, target_pose, angle_type, value):
        """
        Get PoseStamped "target_pose", apply a shift of float "value" along the string "angle_type" and return
        the updated PoseStamped.
        """
        euler = self._euler_from_quaternion(target_pose.pose.orientation)
        if angle_type == "roll":
            euler[0] = euler[0] + value
        if angle_type == "pitch":
            euler[1] = euler[1] + value
        if angle_type == "yaw":
            euler[2] = euler[2] + value

        quaternion = self._quaternion_from_euler(euler)
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        return target_pose

    @staticmethod
    def _euler_from_quaternion(quaternion):
        """
        get roll, pitch, yaw from end effector orientation quaternion
        """
        explicit_quat = [quaternion.x, quaternion.y,
                         quaternion.z, quaternion.w]
        euler = []
        euler_tuple = tf.transformations.euler_from_quaternion(explicit_quat)

        euler.append(euler_tuple[0])
        euler.append(euler_tuple[1])
        euler.append(euler_tuple[2])

        return euler

    @staticmethod
    def _quaternion_from_euler(euler):
        """
        return quaternion from euler
        """
        quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
        return quaternion

    # - KEYBOARD LISTENER FUNCTIONS

    def on_press(self, key):
        """
        Function used by the keyboard listener thread
        """
        try:
            if key == keyboard.Key.shift:
                rospy.loginfo('shift key!')
                self.current_key = ''
            else:
                self.current_key = key.char
        except AttributeError:
            rospy.loginfo('special key {0} pressed'.format(key))
            self.current_key = ''

    def on_release(self, key):
        """
        Function used by the keyboard listener thread
        """
        self.current_key = ''
        if key == keyboard.Key.esc:
            return False
        if key == keyboard.Key.ctrl:
            return False
        elif key == keyboard.Key.shift:
            self.current_key = ''
        else:
            self.current_key = ''

    @staticmethod
    def set_learning_mode(bool_):
        """
        deactivate learning mode to control with jog
        """
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


def display_controls_help():
    rospy.loginfo("Press CTRL or ESC  to finish the program")
    rospy.loginfo("Press i,k (z axis) or j,l (y axis) or p,m (x axis) to control Ned in translation")
    rospy.loginfo("Press a,q (roll) or z,s (pitch) or e,d (yaw) to control Ned in orientation")


if __name__ == "__main__":
    rospy.init_node('controlJointCommand', anonymous=True)

    # Display infos on how to control the robot
    display_controls_help()

    # Creating Client Object
    jc = ControlJointCommand()

    rospy.spin()

#!/usr/bin/env python
"""
Node to send a command to Ned from Webots. The robot will move like a digital-twin from the simulator to the real robot.
"""

import rospy

from std_msgs.msg import String 
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class SendWebotsCommand:
    def __init__(self):
        rospy.init_node('send_webots_command')
        # Set rate in Hz
        rate = 50
        r = rospy.Rate(rate)

        self._webots_joint1 = None
        self._webots_joint2 = None
        self._webots_joint3 = None
        self._webots_joint4 = None
        self._webots_joint5 = None
        self._webots_joint6 = None

        # Subscribers
        rospy.wait_for_message('/niryo_robot_webots/joint_1', String)
        rospy.Subscriber('/niryo_robot_webots/joint_1', String,
                         self.__callback_webots_joint_1)

        rospy.wait_for_message('/niryo_robot_webots/joint_2', String)
        rospy.Subscriber('/niryo_robot_webots/joint_2', String,
                         self.__callback_webots_joint_2)
        
        rospy.wait_for_message('/niryo_robot_webots/joint_3', String)
        rospy.Subscriber('/niryo_robot_webots/joint_3', String,
                         self.__callback_webots_joint_3)
        
        rospy.wait_for_message('/niryo_robot_webots/joint_4', String)
        rospy.Subscriber('/niryo_robot_webots/joint_4', String,
                         self.__callback_webots_joint_4)
    
        rospy.wait_for_message('/niryo_robot_webots/joint_5', String)
        rospy.Subscriber('/niryo_robot_webots/joint_5', String,
                         self.__callback_webots_joint_5)
        
        rospy.wait_for_message('/niryo_robot_webots/joint_6', String)
        rospy.Subscriber('/niryo_robot_webots/joint_6', String,
                         self.__callback_webots_joint_6)

        self._joint_states = None
        rospy.Subscriber('/joint_states', JointState,
                         self.__callback_joint_states)

        # Publishers
        self._joint_trajectory_publisher = rospy.Publisher('/niryo_robot_follow_joint_trajectory_controller/command',
                                                           JointTrajectory, queue_size=1)

        while not rospy.is_shutdown():

            self.joint_target_tuple = [self._webots_joint1, -self._webots_joint2, -self._webots_joint3, self._webots_joint4, self._webots_joint5, self._webots_joint6]
            self.joint_target = list(self.joint_target_tuple)
            # send command to Ned
            self.publish_joint_trajectory(self.joint_target)
            r.sleep()

    # Callbacks
    def __callback_joint_states(self, _joint_states_msg):
        self._joint_states = _joint_states_msg.position[:6]
    
    def __callback_webots_joint_1(self, _joint_1):
        self._webots_joint1 = float(_joint_1.data)
    
    def __callback_webots_joint_2(self, _joint_2):
        self._webots_joint2 = float(_joint_2.data)
    
    def __callback_webots_joint_3(self, _joint_3):
        self._webots_joint3 = float(_joint_3.data)
    
    def __callback_webots_joint_4(self, _joint_4):
        self._webots_joint4 = float(_joint_4.data)
    
    def __callback_webots_joint_5(self, _joint_5):
        self._webots_joint5 = float(_joint_5.data)
    
    def __callback_webots_joint_6(self, _joint_6):
        self._webots_joint6 = float(_joint_6.data)

    def publish_joint_trajectory(self, joint_target):

        joint_trajectory = JointTrajectory()
        joint_trajectory.header.stamp = rospy.Time.now()
        joint_trajectory.header.frame_id = "arm"
        joint_trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        joint_trajectory_points = JointTrajectoryPoint()
        joint_trajectory_points.positions = self.joint_target
        joint_trajectory_points.time_from_start = rospy.Duration(0.1)

        joint_trajectory.points = [joint_trajectory_points]
        self._joint_trajectory_publisher.publish(joint_trajectory)


if __name__ == '__main__':
    try:
        SendWebotsCommand()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

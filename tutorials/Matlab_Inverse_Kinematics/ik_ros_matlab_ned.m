%% Script to communicate between the Ned and Matlab using the ROS Toolbox 
% Made by Nicolas Guy
% Made the 05/03/2021
% Matlab Script to control the Ned with the ROS Toolbox


%% Setup Matlab with the Ned to have all the Messages and the Services

% Use this part when you launch the script for the first time to have
% access to all the messages of the Ned

% folderpath = "/YOUR_PATH/ned_ros_stack";
% rosgenmsg(folderpath)


%% Init the communication between matlab and Ned

rosshutdown; %to be sure that an other ROS network is not actually working

%You can add the two line if you don't want to directly set up your host computer and the Ned 
setenv('ROS_MASTER_URI','http://192.168.1.52:11311') %IP of the Ned
setenv('ROS_IP','192.168.1.96') %IP of the computer

ipaddress = "http://192.168.1.52:11311"; %IP of the Ned
rosinit(ipaddress) 


%% Call Service FK Ned

% client = rossvcclient('/niryo_robot/kinematics/forward');
% ReqMsg = rosmessage(client);
% ReqMsg.Joints = [1,0,0,0,0,0];
% FK = call(client, ReqMsg);

%% Call Service IK Ned

client = rossvcclient('/niryo_robot/kinematics/inverse');
ReqMsg = rosmessage(client);

ReqMsg.Pose.Position.X = 0.156000000000000;
ReqMsg.Pose.Position.Y = 0.243000000000000;
ReqMsg.Pose.Position.Z = 0.434000000000000;

ReqMsg.Pose.Rpy.Roll = -4.689665592537737e-12;
ReqMsg.Pose.Rpy.Pitch = 1.110222783151426e-16;
ReqMsg.Pose.Rpy.Yaw = 1.000000000015311;


ReqMsg.Pose.Orientation.X = 0;
ReqMsg.Pose.Orientation.Y = 0;
ReqMsg.Pose.Orientation.Z = 0.479000000000000;
ReqMsg.Pose.Orientation.W = 0.878000000000000;

IK = call(client, ReqMsg);
pause(1);

%% Move Ned with Matlab using the IK service

NedState = rossubscriber("/niryo_robot_follow_joint_trajectory_controller/state");

NedCmd = rospublisher("/niryo_robot_follow_joint_trajectory_controller/command");
CmdMsg = rosmessage(NedCmd);

CmdPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
CmdPoint.Positions = IK.Joints; %We get the Joints goal from the IK service
CmdPoint.Velocities = zeros(1,6);
CmdPoint.Accelerations = zeros(1,6);
CmdPoint.TimeFromStart = ros.msg.Duration(3);

CmdMsg.Header.Stamp = rostime("now") + rosduration(0.05);
CmdMsg.JointNames = {'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'}; 
CmdMsg.Points = CmdPoint;

send(NedCmd,CmdMsg);









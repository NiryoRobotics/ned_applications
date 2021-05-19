%% Script to simulate Ned with matlab and send a command to a real Ned
% Made by Nicolas Guy
% Made the 23/03/2021
% Matlab script to simulate Ned with Matlab tools and use the ROS Toolbox to send
% the command to the real Ned.

clear All
clc

%% Simulate the Ned on Matlab
ned = importrobot("ned.urdf");
axes = show(ned);
axes.CameraPositionMode = 'auto';
showdetails(ned);


%% Direct Geometry Model
inititial_configuration = ned.homeConfiguration;

T1_2 = ned.Bodies{1,2}.Joint.JointToParentTransform;
T2_3 = ned.Bodies{1,3}.Joint.JointToParentTransform;
T3_4 = ned.Bodies{1,4}.Joint.JointToParentTransform;
T4_5 = ned.Bodies{1,5}.Joint.JointToParentTransform;
T5_6 = ned.Bodies{1,6}.Joint.JointToParentTransform;
T6_7 = ned.Bodies{1,7}.Joint.JointToParentTransform;
T7_8 = ned.Bodies{1,8}.Joint.JointToParentTransform;

T = T1_2*T2_3*T3_4*T4_5*T5_6*T6_7*T7_8;

eeoffset = 0;
eeBody = robotics.RigidBody("end_effector");
setFixedTransform(eeBody.Joint, trvec2tform([eeoffset,0,0]));
addBody(ned, eeBody, "tool_link");
T_M = getTransform(ned, ned.homeConfiguration,"end_effector","base_link");

%% Inverse Geometry Model

ik = inverseKinematics("RigidBodyTree", ned);
weight = [0.1 0.1 0 1 1 1];
initialguess = ned.homeConfiguration;
pose_M = [0.25 0 0.3];
tform = trvec2tform(pose_M);
configSoln = ik("end_effector", tform, weight,initialguess);

%%Convert configSoln into Joint values readable by the ROS Toolbox 

% cell = struct2cell(configSoln); 
% Joint = cell(2,:,:);
% matrixJoints = cell2mat(Joint);

%% Publish the Inverse Geometry Model on a real Ned via the ROS Toolbox of Matlab

% Use this part when you launch the script for the first time to have
% access to all the messages of the Ned
% 
% folderpath = "/home/niryodev1/Bureau/Nicolas/Niryo_Tool/ned_ros_stack";
% rosgenmsg(folderpath)

% rosshutdown; %to be sure that an other ROS network is not actually working
% setenv('ROS_MASTER_URI','http://127.0.0.1:11311') %IP of the Ned
% setenv('ROS_IP','192.168.1.96') %IP of the computer
% 
% ipaddress = "http://127.0.0.1:11311"; %IP of the Ned
% rosinit(ipaddress) 
% 
% NedState = rossubscriber("/niryo_robot_follow_joint_trajectory_controller/state");
% 
% NedCmd = rospublisher("/niryo_robot_follow_joint_trajectory_controller/command");
% CmdMsg = rosmessage(NedCmd);
% 
% CmdPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
% 
% CmdPoint.Positions(1) = matrixJoints(1);
% CmdPoint.Positions(2) = matrixJoints(2);
% CmdPoint.Positions(3) = matrixJoints(3);
% CmdPoint.Positions(4) = matrixJoints(4);
% CmdPoint.Positions(5) = matrixJoints(5);
% CmdPoint.Positions(6) = matrixJoints(6);
% 
% CmdPoint.Velocities = zeros(1,6);
% CmdPoint.Accelerations = zeros(1,6);
% CmdPoint.TimeFromStart = ros.msg.Duration(3);
% 
% CmdMsg.Header.Stamp = rostime("now") + rosduration(0.05);
% CmdMsg.JointNames = {'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'}; 
% CmdMsg.Points = CmdPoint;
% 
% send(NedCmd,CmdMsg);











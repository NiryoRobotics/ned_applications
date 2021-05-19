%% Script to communicate between the Ned and Matlab using the ROS Toolbox 
% Made by Nicolas Guy
% Made the 05/03/2021
% Matlab Script to control the Ned with the ROS Toolbox


%% Setup Matlab with the Ned to have all the Messages and the Services

% Use this part when you launch the script for the first time to have
% access to all the messages of the Ned

% folderpath = "/YOUR_PATH/ned_ros_stack"
% rosgenmsg(folderpath)


%% Init the communication between matlab and Ned

rosshutdown; %to be sure that an other ROS network is not actually working

%You can add the two line if you don't want to directly set up your host computer and the Ned 
setenv('ROS_MASTER_URI','http://192.168.1.52:11311') %IP of the Ned
setenv('ROS_IP','192.168.1.96') %IP of the computer

ipaddress = "http://192.168.1.52:11311"; %IP of the Ned
rosinit(ipaddress) 

%% Subscribe to a topics of the Ned on Matlab via ROS

joint_states = rossubscriber('/joint_states');
pause(2)
scandata = receive(joint_states,10);

%% Publish to a topics of the Ned on Matlab via ROS

robotCmd = rospublisher("/niryo_robot/learning_mode/state") ;
velMsg = rosmessage(robotCmd);

velMsg.Data = true;
send(robotCmd,velMsg)

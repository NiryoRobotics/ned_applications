// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Includes
#include "ros/ros.h"

#include <webots_ros/Int32Stamped.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/robot_get_device_list.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <sstream>


#include "niryo_robot_webots/webots_core.hpp"

namespace webots {

  constexpr int TIME_STEP = 32;

  WebotsCore::WebotsCore()
  {
    initController();
    initServices();
    initPublishers();
  }

  WebotsCore::~WebotsCore() 
  {

  }

  int WebotsCore::initController()
  {
      // subscribe to the topic model_name to get the list of availables controllers
      ros::Subscriber nameSub = _nh.subscribe("model_name", 100, &WebotsCore::controllerNameCallback, this);
      while (_controllerCount == 0 || _controllerCount < nameSub.getNumPublishers()) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
      }
      ros::spinOnce();

      // if there is more than one controller available, let the user choose
      if (_controllerCount == 1)
        _controllerName = _controllerList[0];
      else {
        int wantedController = 0;
        std::cout << "Choose the # of the controller you want to use:\n";
        std::cin >> wantedController;
        if (1 <= wantedController && wantedController <= _controllerCount)
          _controllerName = _controllerList[wantedController - 1];
        else {
          ROS_ERROR("Invalid number for controller choice.");
          return 1;
        }
      }

      // leave topic once it's not necessary anymore
      nameSub.shutdown();

      return 0;
  }
  /**
        * @brief initServices
  */
  void WebotsCore::initServices()
  {
    // Get the joints of Ned on Webots
    _joint_1Client = _nh.serviceClient<webots_ros::set_float>(_controllerName + "/joint_1/set_position");
    _joint_2Client = _nh.serviceClient<webots_ros::set_float>(_controllerName + "/joint_2/set_position");
    _joint_3Client = _nh.serviceClient<webots_ros::set_float>(_controllerName + "/joint_3/set_position");
    _joint_4Client = _nh.serviceClient<webots_ros::set_float>(_controllerName + "/joint_4/set_position");
    _joint_5Client = _nh.serviceClient<webots_ros::set_float>(_controllerName + "/joint_5/set_position");
    _joint_6Client = _nh.serviceClient<webots_ros::set_float>(_controllerName + "/joint_6/set_position");
    _jaw_1Client = _nh.serviceClient<webots_ros::set_float>(_controllerName + "/joint_base_to_jaw_1/set_position");
    _jaw_2Client = _nh.serviceClient<webots_ros::set_float>(_controllerName + "/joint_base_to_jaw_2/set_position");
  }

  /**
        * @brief initPublishers
  */
  void WebotsCore::initPublishers()
  {
    // Publisher Joints Ned
    _joint_1Pub = _nh.advertise<std_msgs::String>("niryo_robot_webots/joint_1", 1000);
    _joint_2Pub = _nh.advertise<std_msgs::String>("niryo_robot_webots/joint_2", 1000);
    _joint_3Pub = _nh.advertise<std_msgs::String>("niryo_robot_webots/joint_3", 1000);
    _joint_4Pub = _nh.advertise<std_msgs::String>("niryo_robot_webots/joint_4", 1000);
    _joint_5Pub = _nh.advertise<std_msgs::String>("niryo_robot_webots/joint_5", 1000);
    _joint_6Pub = _nh.advertise<std_msgs::String>("niryo_robot_webots/joint_6", 1000);
    _jaw_1Pub = _nh.advertise<std_msgs::String>("niryo_robot_webots/jaw_1", 1000);
    _jaw_2Pub = _nh.advertise<std_msgs::String>("niryo_robot_webots/jaw_2", 1000);
  }

  /**
        * @brief controllerNameCallback
        * @param std_msgs::String::ConstPtr &name
        * @return controllerList
  */
  void WebotsCore::controllerNameCallback(const std_msgs::String::ConstPtr &name) 
  {
    // catch names of the controllers availables on ROS network
    _controllerCount++;
    _controllerList.push_back(name->data);
    ROS_INFO("Controller #%d: %s.", _controllerCount, _controllerList.back().c_str());
  }

  /**
      * @brief quit
      * @param sig
  */
  void WebotsCore::quit(int sig) 
  {
    // shut down webots with the ESCAPE keyboard key
    _enableKeyboardSrv.request.value = 0;
    _enableKeyboardClient.call(_enableKeyboardSrv);
    _timeStepSrv.request.value = 0;
    _timeStepClient.call(_timeStepSrv);
    ROS_INFO("User stopped the 'webots_node' node.");
    ros::shutdown();
    exit(0);
  }

  /**
      * @brief keyboardCallback
      * @param webots_ros::Int32Stamped::ConstPtr &value
  */
  void WebotsCore::keyboardCallback(const webots_ros::Int32Stamped::ConstPtr &value)
  {
    // Move Ned according to the keyboard key pressed
    int key = value->data;
    int send = 0;

    switch (key) 
    {
      case 'A':
        if (_joint_1 > -2.78)
        {
          _joint_1 += -0.05;
          send = 1;
        }
        break;
      case 'Z':
        if (_joint_1 < 2.78)
        {
          _joint_1 += 0.05;
          send = 1;
        }
        break;
      case 'S':
        if (_joint_2 > -0.38)
        {
          _joint_2 += -0.05;
          send = 1;
        }
        break;
      case 'Q':
        if (_joint_2 < 0.48)
        {
          _joint_2 += 0.05;
          send = 1;
        }
        break;
      case 'X':
        if (_joint_3 > -1.28)
        {
          _joint_3 += -0.05;
          send = 1;
        }
        break;
      case 'W':
        if (_joint_3 < 1.18)
        {
        _joint_3 += 0.05;
        send = 1;
        }
        break;
      case 'U':
        if (_joint_4 > -1.98)
        {
          _joint_4 += -0.05;
          send = 1;
        }
        break;
      case 'Y':
        if (_joint_4 < 1.98)
        {
          _joint_4 += 0.05;
          send = 1;
        }
        break;
      case 'H':
        if (_joint_5 > -1.48)
        {
          _joint_5 += -0.05;
          send = 1;
        }
        break;
      case 'J':
        if (_joint_5 < 0.88)
        {
          _joint_5 += 0.05;
          send = 1;
        }
        break;
      case 'N':
        if (_joint_6 > -2.48)
        {
          _joint_6 += -0.05;
          send = 1;
        }
        break;
      case 'B':
        if (_joint_6 < 2.48)
        {
          _joint_6 += 0.05;
          send = 1;
        }
        break;
      case 'L':
        if (_jaw_1 < 0.009)
        {
          _jaw_1 += 0.001;
          _jaw_2 += 0.001;
          send = 1;
        }
        send = 1;
        break;
      case 'M':
        if (_jaw_1 > -0.009)
        {
          _jaw_1 += -0.001;
          _jaw_2 += -0.001;
          send = 1;
        }
        break;
      case 27:
        quit(-1);
        break;
      default:
        send = 0;
        break;
    }
  
    webots_ros::set_float joint_1Srv;
    webots_ros::set_float joint_2Srv;
    webots_ros::set_float joint_3Srv;
    webots_ros::set_float joint_4Srv;
    webots_ros::set_float joint_5Srv;
    webots_ros::set_float joint_6Srv;
    webots_ros::set_float jaw_1Srv;
    webots_ros::set_float jaw_2Srv;
    joint_1Srv.request.value = _joint_1;
    joint_2Srv.request.value = _joint_2;
    joint_3Srv.request.value = _joint_3;
    joint_4Srv.request.value = _joint_4;
    joint_5Srv.request.value = _joint_5;
    joint_6Srv.request.value = _joint_6;
    jaw_1Srv.request.value = _jaw_1;
    jaw_2Srv.request.value = _jaw_2;
    
    // Check if one of the Joint_XSrv was a success or not 
    if (send) 
    {
      if (!_joint_1Client.call(joint_1Srv) || 
          !_joint_2Client.call(joint_2Srv) || 
          !_joint_3Client.call(joint_3Srv) || 
          !_joint_4Client.call(joint_4Srv) || 
          !_joint_5Client.call(joint_5Srv) || 
          !_joint_6Client.call(joint_6Srv) || 
          !_jaw_1Client.call(jaw_1Srv) || 
          !_jaw_2Client.call(jaw_2Srv) || 
          !joint_1Srv.response.success || 
          !joint_2Srv.response.success || 
          !joint_3Srv.response.success || 
          !joint_4Srv.response.success || 
          !joint_5Srv.response.success || 
          !joint_6Srv.response.success || 
          !jaw_1Srv.response.success || 
          !jaw_2Srv.response.success)
        ROS_ERROR("Failed to send new position commands to the robot.");
    }
    return;
  }

  /**
      * @brief castDoubleToString
      * @param double joint_to_cast
      * @return string s 
  */
  std::string WebotsCore::castDoubleToString(double joint_to_cast)
  {
    // function to cast a double into string
    std::string s;
    std::ostringstream oss;
    oss << joint_to_cast;
    s = oss.str();
    return s;
  }

  int WebotsCore::loop() 
  {
    // enable the keyboard
    _enableKeyboardClient = _nh.serviceClient<webots_ros::set_int>(_controllerName + "/keyboard/enable");
    _enableKeyboardSrv.request.value = TIME_STEP;
    if (_enableKeyboardClient.call(_enableKeyboardSrv) && _enableKeyboardSrv.response.success) {
      ros::Subscriber sub_keyboard;
      sub_keyboard = _nh.subscribe(_controllerName + "/keyboard/key", 1, &WebotsCore::keyboardCallback, this);
      while (sub_keyboard.getNumPublishers() == 0) {
      }
      ROS_INFO("Keyboard enabled.");
      ROS_INFO("Use the keyboard keys in Webots window to move the robot.");
      ROS_INFO("Press the ESC key to stop the node.");

      // publish joints values 
      while (ros::ok()) {

        std_msgs::String joint_str;
        joint_str.data = castDoubleToString(_joint_1);
        _joint_1Pub.publish(joint_str);

        joint_str.data = castDoubleToString(_joint_2);
        _joint_2Pub.publish(joint_str);

        joint_str.data = castDoubleToString(_joint_3);
        _joint_3Pub.publish(joint_str);

        joint_str.data = castDoubleToString(_joint_4);
        _joint_4Pub.publish(joint_str);

        joint_str.data = castDoubleToString(_joint_5);
        _joint_5Pub.publish(joint_str);

        joint_str.data = castDoubleToString(_joint_6);
        _joint_6Pub.publish(joint_str);

        joint_str.data = castDoubleToString(_jaw_1);
        _jaw_1Pub.publish(joint_str);

        joint_str.data = castDoubleToString(_jaw_2);
        _jaw_2Pub.publish(joint_str);

        _timeStepClient = _nh.serviceClient<webots_ros::set_int>(_controllerName + "/robot/time_step");
        _timeStepSrv.request.value = TIME_STEP;
        ros::spinOnce();
        if (!_timeStepClient.call(_timeStepSrv) || !_timeStepSrv.response.success)
          ROS_ERROR("Failed to call service time_step for next step.");
      }
    } 
    // error with the keyboard 
    else
      ROS_ERROR("Could not enable keyboard, success = %d.", _enableKeyboardSrv.response.success);

    // keyboard failed to be enabled
    _enableKeyboardSrv.request.value = 0;
    if (!_enableKeyboardClient.call(_enableKeyboardSrv) || 
        !_enableKeyboardSrv.response.success)
      ROS_ERROR("Could not disable keyboard, success = %d.", _enableKeyboardSrv.response.success);
    _timeStepSrv.request.value = 0;
    _timeStepClient.call(_timeStepSrv);
    ros::shutdown();
    return (0);
  }

} // namespace webots


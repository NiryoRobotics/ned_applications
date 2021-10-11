/*
    webots_node.hpp
    Copyright (C) 2020 Niryo
    All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// ros
#include <ros/ros.h>
#include "niryo_robot_webots/webots_core.hpp"


// main webots_core.cpp
int main(int argc, char **argv) {
  
  // create a node named 'webots_node' on ROS network
  ros::init(argc, argv, "webots_node");
  ros::NodeHandle _nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  auto myWebots_shared_ptr = std::make_shared<webots::WebotsCore>();
  myWebots_shared_ptr->loop();

  ros::waitForShutdown();

  ROS_INFO("Webots_node - Shutdown node");

}


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

#ifndef WEBOTSCORE_HPP
#define WEBOTSCORE_HPP

// ros 
#include "ros/ros.h"
#include <std_msgs/String.h>

// webots
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/robot_get_device_list.h>


namespace webots {

    class WebotsCore
    {
    public:
        WebotsCore();
        virtual ~WebotsCore(); 
        int loop();

    private:
        int initController();
        void initServices();
        void initPublishers();

        // define methodes
        void controllerNameCallback(const std_msgs::String::ConstPtr &name);
        void quit(int sig);

        // Callbacks
        void keyboardCallback(const webots_ros::Int32Stamped::ConstPtr &value);
        std::string castDoubleToString(double joint_to_cast);

    private:
        ros::NodeHandle _nh;

        static constexpr int TIME_STEP = 32;

        // define services
        ros::ServiceClient _joint_1Client;
        ros::ServiceClient _joint_2Client;
        ros::ServiceClient _joint_3Client;
        ros::ServiceClient _joint_4Client;
        ros::ServiceClient _joint_5Client;
        ros::ServiceClient _joint_6Client;
        ros::ServiceClient _jaw_1Client;
        ros::ServiceClient _jaw_2Client;

        ros::ServiceClient _timeStepClient;
        webots_ros::set_int _timeStepSrv;

        ros::ServiceClient _enableKeyboardClient;
        webots_ros::set_int _enableKeyboardSrv;

        // Publishers
        ros::Publisher _joint_1Pub;
        ros::Publisher _joint_2Pub;
        ros::Publisher _joint_3Pub;
        ros::Publisher _joint_4Pub;
        ros::Publisher _joint_5Pub;
        ros::Publisher _joint_6Pub;
        ros::Publisher _jaw_1Pub;   
        ros::Publisher _jaw_2Pub;

        // define attributes
        int _controllerCount;
        std::vector<std::string> _controllerList;
        std::string _controllerName;

        _Float64 _joint_1 {0};
        _Float64 _joint_2 {-0.5};
        _Float64 _joint_3 {1.25};
        _Float64 _joint_4 {0};
        _Float64 _joint_5 {0};
        _Float64 _joint_6 {0};
        _Float64 _jaw_1 {0};
        _Float64 _jaw_2 {0};
    };

}


#endif // WEBOTSCORE_HPP
#ifndef _IPC_DIAGNOSTIC_PUB_
#define _IPC_DIAGNOSTIC_PUB_

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/String.h>

#include <katech_diagnostic_msgs/lidar_diagnostic_msg.h>
#include <katech_diagnostic_msgs/ipc_diagnostic_msg.h>
#include <perception_ros_msg/RsPerceptionMsg.h>
#include <perception_ros_msg/object_msg.h>
#include <perception_ros_msg/object_array_msg.h>

class IPC_DIAGNOSTIC_PUB
{
    public:
        IPC_DIAGNOSTIC_PUB();
        ~IPC_DIAGNOSTIC_PUB();

    private:
        ros::NodeHandle nh;

        ros::Publisher pub;
        ros::Subscriber sub;

        ros::Timer timer_;

        uint8_t percept_callback_cnt, percept_callback_cnt_old;

        katech_diagnostic_msgs::ipc_diagnostic_msg ipc_msg;

        void timer_callback(const ros::TimerEvent&);
        void percept_callback(const perception_ros_msg::RsPerceptionMsg::ConstPtr& msg);
};

#endif
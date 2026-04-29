#ifndef _LIDAR_DIAGNOSTIC_PUB_
#define _LIDAR_DIAGNOSTIC_PUB_

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>

#include <atomic>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/String.h>

#include <katech_diagnostic_msgs/lidar_diagnostic_msg.h>
#include <perception_ros_msg/RsPerceptionMsg.h>
#include <perception_ros_msg/object_msg.h>
#include <perception_ros_msg/object_array_msg.h>

class LIDAR_DIAGNOSTIC_PUB
{
    public:
        LIDAR_DIAGNOSTIC_PUB();
        ~LIDAR_DIAGNOSTIC_PUB();

    private:
        struct struct_LIDAR{
            std::string ip;
            uint16_t port;
        };
        std::vector<struct_LIDAR> lidar;

        ros::NodeHandle nh;

        ros::Publisher pub;
        ros::Subscriber sub;

        ros::Timer timer_;

        katech_diagnostic_msgs::lidar_diagnostic_msg lidar_msg;

        std::atomic<bool> center_failed;
        std::atomic<bool> right_failed;
        std::atomic<bool> left_failed;
        std::atomic<bool> ping_thread_stop;
        std::thread ping_thread;

        void timer_callback(const ros::TimerEvent&);
        void percept_callback(const perception_ros_msg::RsPerceptionMsg::ConstPtr& msg);

        bool pingCheck(const std::string& ip);
        void pingLoop();
};

#endif

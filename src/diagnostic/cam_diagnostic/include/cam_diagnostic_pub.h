#ifndef _CAM_DIAGNOSTIC_PUB_
#define _CAM_DIAGNOSTIC_PUB_

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

#include <katech_diagnostic_msgs/cam_diagnostic_msg.h>
#include <mmc_msgs/object_array_msg.h>
#include <mmc_msgs/lane_array_msg.h>

class CAM_DIAGNOSTIC_PUB
{
    public:
        CAM_DIAGNOSTIC_PUB();
        ~CAM_DIAGNOSTIC_PUB();

    private:
        ros::NodeHandle nh;

        ros::Publisher pub;
        ros::Subscriber lane_sub, obj_sub;

        ros::Timer timer_;

        katech_diagnostic_msgs::cam_diagnostic_msg cam_msg;
        mmc_msgs::object_array_msg msg_obj;
        mmc_msgs::lane_array_msg msg_lane;

        uint8_t lane_callback_cnt, lane_callback_cnt_old;
        uint8_t obj_callback_cnt, obj_callback_cnt_old;

        void timer_callback(const ros::TimerEvent&);
        void cam_lane_callback(const mmc_msgs::lane_array_msg::ConstPtr& msg);
        void cam_obj_callback(const mmc_msgs::object_array_msg::ConstPtr& msg);
};

#endif
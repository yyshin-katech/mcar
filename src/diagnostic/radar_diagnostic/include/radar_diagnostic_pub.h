#ifndef _RADAR_DIAGNOSTIC_PUB_
#define _RADAR_DIAGNOSTIC_PUB_

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <unistd.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/String.h>

#include <katech_diagnostic_msgs/radar_diagnostic_msg.h>
#include <mmc_msgs/object_array_msg.h>
#include <mmc_msgs/object_msg.h>

class RADAR_DIAGNOSTIC_PUB
{
    public:
        RADAR_DIAGNOSTIC_PUB();
        ~RADAR_DIAGNOSTIC_PUB();

    private:
        ros::NodeHandle nh;

        ros::Publisher pub;
        ros::Subscriber sub;

        ros::Timer timer_;

        katech_diagnostic_msgs::radar_diagnostic_msg radar_msg;
        mmc_msgs::object_array_msg radar_object_msg;

        uint8_t radar_callback_cnt, radar_callback_cnt_old;

        void timer_callback(const ros::TimerEvent&);
        void radar_object_callback(const mmc_msgs::object_array_msg::ConstPtr& msg);

};

#endif
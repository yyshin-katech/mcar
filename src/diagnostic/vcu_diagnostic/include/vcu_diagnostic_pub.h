#ifndef _VCU_DIAGNOSTIC_PUB_
#define _VCU_DIAGNOSTIC_PUB_

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

#include <katech_diagnostic_msgs/vcu_diagnostic_msg.h>
#include <katech_custom_msgs/v_can_msg.h>

class VCU_DIAGNOSTIC_PUB
{
    public:
        VCU_DIAGNOSTIC_PUB();
        ~VCU_DIAGNOSTIC_PUB();

    private:
        ros::NodeHandle nh;

        ros::Publisher pub;
        ros::Subscriber sub;

        ros::Timer timer_;

        katech_diagnostic_msgs::vcu_diagnostic_msg vcu_msg;

        uint8_t life_count_cur[6], life_count_old[6];
        ros::Time last_change_time[6];

        void timer_callback(const ros::TimerEvent&);
        void vcu_callback(const katech_custom_msgs::v_can_msg::ConstPtr& msg);
};

#endif
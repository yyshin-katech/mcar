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
#include <katech_custom_msgs/ioniq5_ad_can_msg.h>

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

        // AutonomousState(ID 16) life_count staleness + error_code 로 VCU 고장 판정
        uint8_t life_count_cur, life_count_old;
        ros::Time last_change_time;
        uint16_t error_code_cur;

        void timer_callback(const ros::TimerEvent&);
        void vcu_callback(const katech_custom_msgs::ioniq5_ad_can_msg::ConstPtr& msg);
};

#endif
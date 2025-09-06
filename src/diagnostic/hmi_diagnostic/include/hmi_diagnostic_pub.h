#ifndef _V2X_DIAGNOSTIC_PUB_
#define _V2X_DIAGNOSTIC_PUB_

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

#include <katech_diagnostic_msgs/hmi_diagnostic_msg.h>

class HMI_DIAGNOSTIC_PUB
{
    public:
        HMI_DIAGNOSTIC_PUB();
        ~HMI_DIAGNOSTIC_PUB();

    private:
        ros::NodeHandle nh;

        ros::Publisher pub;
        ros::Subscriber sub;

        ros::Timer timer_;

        katech_diagnostic_msgs::hmi_diagnostic_msg hmi_msg;

        uint8_t hmie_callback_cnt, hmie_callback_cnt_old;
        uint8_t conn_stat;

        void timer_callback(const ros::TimerEvent&);

        // bool checkConnection(const std::string& ip, uint16_t port);

};

#endif
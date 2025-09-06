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

#include <katech_diagnostic_msgs/v2x_diagnostic_msg.h>
#include <v2x_msgs/intersection_msg.h>
#include <v2x_msgs/intersection_array_msg.h>

class V2X_DIAGNOSTIC_PUB
{
    public:
        V2X_DIAGNOSTIC_PUB();
        ~V2X_DIAGNOSTIC_PUB();

    private:
        struct struct_V2X{
            std::string ip;
            uint16_t port;
        };
        std::vector<struct_V2X> v2x_obu;

        ros::NodeHandle nh;

        ros::Publisher pub;
        ros::Subscriber sub;

        ros::Timer timer_;

        katech_diagnostic_msgs::v2x_diagnostic_msg v2x_msg;
        v2x_msgs::intersection_array_msg v2x_intersection_msg;

        uint8_t v2x_callback_cnt, v2x_callback_cnt_old;
        uint8_t conn_stat;

        void timer_callback(const ros::TimerEvent&);
        void v2x_callback(const v2x_msgs::intersection_array_msg::ConstPtr& msg);

        bool checkConnection(const std::string& ip, uint16_t port);

};

#endif
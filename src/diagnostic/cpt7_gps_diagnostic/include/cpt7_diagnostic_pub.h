#ifndef _CPT7_DIAGNOSTIC_PUB_
#define _CPT7_DIAGNOSTIC_PUB_

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <netinet/ip_icmp.h>

#include <atomic>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/String.h>
#include <katech_diagnostic_msgs/cpt7_gps_diagnostic_msg.h>
#include <ublox_msgs/NavPVT.h>

class CPT7_DIAGNOSTIC_PUB
{
    public:
        CPT7_DIAGNOSTIC_PUB();
        ~CPT7_DIAGNOSTIC_PUB();

        static void end(int sig);

    private:
        ros::NodeHandle nh;

        ros::Publisher pub;
        ros::Subscriber sub;

        ros::Timer timer_;

        katech_diagnostic_msgs::cpt7_gps_diagnostic_msg cpt7_msg;

        unsigned char alive_cnt;
        bool msg_received;

        std::atomic<bool> network_failed;
        std::atomic<bool> ping_thread_stop;
        std::thread ping_thread;

        void navpvt_callback(const ublox_msgs::NavPVT::ConstPtr& msg);
        void timerCallback(const ros::TimerEvent&);
        bool pingCheck(const std::string& ip);
        void pingLoop();
};


#endif

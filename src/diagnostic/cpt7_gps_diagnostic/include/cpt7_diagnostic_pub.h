#ifndef _CPT7_DIAGNOSTIC_PUB_
#define _CPT7_DIAGNOSTIC_PUB_

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/String.h>
#include <katech_diagnostic_msgs/cpt7_gps_diagnostic_msg.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/Inspva.h>

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
        ros::Subscriber sub_inspva;

        ros::Timer timer_;

        katech_diagnostic_msgs::cpt7_gps_diagnostic_msg cpt7_msg;

        unsigned char alive_cnt;
        unsigned char bestpos_cb_cnt, inspva_cb_cnt;
        unsigned char bestpos_cb_cnt_old, inspva_cb_cnt_old;

        void bestpos_callback(const novatel_gps_msgs::NovatelPosition::ConstPtr& msg);
        void inspva_callback(const novatel_gps_msgs::Inspva::ConstPtr& msg);
        void timerCallback(const ros::TimerEvent&);

};


#endif
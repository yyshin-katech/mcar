#include <radar_diagnostic_pub.h>

RADAR_DIAGNOSTIC_PUB::RADAR_DIAGNOSTIC_PUB()
{
    pub = nh.advertise<katech_diagnostic_msgs::radar_diagnostic_msg>("/diagnostic/radar", 1);
    sub = nh.subscribe("/sensors/front_radar", 5, &RADAR_DIAGNOSTIC_PUB::radar_object_callback, this);

    timer_ = nh.createTimer(ros::Duration(1.0), &RADAR_DIAGNOSTIC_PUB::timer_callback, this);

    radar_callback_cnt = 0;
    radar_callback_cnt_old = 0;
}

RADAR_DIAGNOSTIC_PUB::~RADAR_DIAGNOSTIC_PUB()
{

}

void RADAR_DIAGNOSTIC_PUB::timer_callback(const ros::TimerEvent&)
{
    if(radar_callback_cnt == radar_callback_cnt_old)
    {

    }
    else
    {
        radar_callback_cnt_old = radar_callback_cnt;
        radar_msg.RADAR_AliveCount++;
    }
    radar_msg.time = ros::Time::now();
    pub.publish(radar_msg);
}

void RADAR_DIAGNOSTIC_PUB::radar_object_callback(const mmc_msgs::object_array_msg::ConstPtr& msg)
{
    radar_callback_cnt++;
}
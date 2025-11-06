#include <hmi_diagnostic_pub.h>

HMI_DIAGNOSTIC_PUB::HMI_DIAGNOSTIC_PUB()
{
    pub = nh.advertise<katech_diagnostic_msgs::hmi_diagnostic_msg>("/diagnostic/hmi", 1);
    // sub = nh.subscribe("/katri_spat", 5, &HMI_DIAGNOSTIC_PUB::hmi_callback, this);

    timer_ = nh.createTimer(ros::Duration(0.1), &HMI_DIAGNOSTIC_PUB::timer_callback, this);
}

HMI_DIAGNOSTIC_PUB::~HMI_DIAGNOSTIC_PUB()
{

}

void HMI_DIAGNOSTIC_PUB::timer_callback(const ros::TimerEvent&)
{
    hmi_msg.HMI_AliveCount++;
    hmi_msg.HMI_StatCode = 0;
    
    pub.publish(hmi_msg);
}
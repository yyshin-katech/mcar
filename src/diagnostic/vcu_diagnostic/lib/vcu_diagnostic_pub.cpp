#include <vcu_diagnostic_pub.h>

VCU_DIAGNOSTIC_PUB::VCU_DIAGNOSTIC_PUB()
{
    pub = nh.advertise<katech_diagnostic_msgs::vcu_diagnostic_msg>("/diagnostic/vcu", 1);
    sub = nh.subscribe("/sensors/chassis", 5, &VCU_DIAGNOSTIC_PUB::vcu_callback, this);

    timer_ = nh.createTimer(ros::Duration(0.1), &VCU_DIAGNOSTIC_PUB::timer_callback, this);
}

VCU_DIAGNOSTIC_PUB::~VCU_DIAGNOSTIC_PUB()
{

}

void VCU_DIAGNOSTIC_PUB::timer_callback(const ros::TimerEvent&)
{
    if(vcu_callback_cnt == vcu_callback_cnt_old)
    {
        vcu_msg.VCU_StatCode = 1;
    }
    else
    {
        vcu_callback_cnt_old = vcu_callback_cnt;
        vcu_msg.VCU_StatCode = 0;
        vcu_msg.VCU_AliveCount++;
    }
    pub.publish(vcu_msg);
}

void VCU_DIAGNOSTIC_PUB::vcu_callback(const mmc_msgs::chassis_msg::ConstPtr& msg)
{
    vcu_callback_cnt++;
}
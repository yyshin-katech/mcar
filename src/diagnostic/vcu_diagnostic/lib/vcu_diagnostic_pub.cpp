#include <vcu_diagnostic_pub.h>

VCU_DIAGNOSTIC_PUB::VCU_DIAGNOSTIC_PUB()
{
    pub = nh.advertise<katech_diagnostic_msgs::vcu_diagnostic_msg>("/diagnostic/vcu", 1);
    sub = nh.subscribe("/sensors/v_can", 5, &VCU_DIAGNOSTIC_PUB::vcu_callback, this);

    life_count_cur = 0;
    life_count_old = 0;

    timer_ = nh.createTimer(ros::Duration(0.1), &VCU_DIAGNOSTIC_PUB::timer_callback, this);
}

VCU_DIAGNOSTIC_PUB::~VCU_DIAGNOSTIC_PUB()
{

}

void VCU_DIAGNOSTIC_PUB::timer_callback(const ros::TimerEvent&)
{
    if(life_count_cur == life_count_old)
    {
        vcu_msg.VCU_StatCode = 1;
    }
    else
    {
        life_count_old = life_count_cur;
        vcu_msg.VCU_StatCode = 0;
        vcu_msg.VCU_AliveCount++;
    }
    pub.publish(vcu_msg);
}

void VCU_DIAGNOSTIC_PUB::vcu_callback(const katech_custom_msgs::v_can_msg::ConstPtr& msg)
{
    life_count_cur = msg->life_count;
}
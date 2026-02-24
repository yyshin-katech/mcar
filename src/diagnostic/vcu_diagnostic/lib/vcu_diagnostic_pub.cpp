#include <vcu_diagnostic_pub.h>

VCU_DIAGNOSTIC_PUB::VCU_DIAGNOSTIC_PUB()
{
    pub = nh.advertise<katech_diagnostic_msgs::vcu_diagnostic_msg>("/diagnostic/vcu", 1);
    sub = nh.subscribe("/sensors/v_can", 5, &VCU_DIAGNOSTIC_PUB::vcu_callback, this);

    life_count_cur = 0;
    life_count_old = 0;
    msg_received = false;

    timer_ = nh.createTimer(ros::Duration(0.1), &VCU_DIAGNOSTIC_PUB::timer_callback, this);
}

VCU_DIAGNOSTIC_PUB::~VCU_DIAGNOSTIC_PUB()
{

}

void VCU_DIAGNOSTIC_PUB::timer_callback(const ros::TimerEvent&)
{
    if(msg_received)
    {
        vcu_msg.VCU_StatCode = 0;
        vcu_msg.VCU_AliveCount++;
        msg_received = false;
    }
    else
    {
        vcu_msg.VCU_StatCode = 1;
    }
    pub.publish(vcu_msg);
}

void VCU_DIAGNOSTIC_PUB::vcu_callback(const katech_custom_msgs::v_can_msg::ConstPtr& msg)
{
    ROS_INFO("vcu_callback called, life_count: %d", msg->life_count);
    life_count_cur = msg->life_count;
    if(life_count_cur != life_count_old)
    {
        msg_received = true;
        life_count_old = life_count_cur;
    }
}
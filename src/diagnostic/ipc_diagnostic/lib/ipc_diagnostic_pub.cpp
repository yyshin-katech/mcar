#include <ipc_diagnostic_pub.h>

IPC_DIAGNOSTIC_PUB::IPC_DIAGNOSTIC_PUB()
{
    pub = nh.advertise<katech_diagnostic_msgs::ipc_diagnostic_msg>("/diagnostic/ipc", 1);
    sub = nh.subscribe("/percept_topic", 5, &IPC_DIAGNOSTIC_PUB::percept_callback, this);

    timer_ = nh.createTimer(ros::Duration(1.0), &IPC_DIAGNOSTIC_PUB::timer_callback, this);

    percept_callback_cnt = 0;
    percept_callback_cnt_old = 0;
}

IPC_DIAGNOSTIC_PUB::~IPC_DIAGNOSTIC_PUB()
{

}

void IPC_DIAGNOSTIC_PUB::timer_callback(const ros::TimerEvent&)
{
    if(percept_callback_cnt == percept_callback_cnt_old)
    {
        ipc_msg.IPC_SWC_StatCode = 1;
    }
    else
    {
        ipc_msg.IPC_AliveCount++;
        ipc_msg.IPC_SWC_StatCode = 0;
        percept_callback_cnt_old = percept_callback_cnt;
    }
    pub.publish(ipc_msg);
    
}

void IPC_DIAGNOSTIC_PUB::percept_callback(const perception_ros_msg::RsPerceptionMsg::ConstPtr& msg)
{
    percept_callback_cnt++;
}
#include <ipc_diagnostic_pub.h>

IPC_DIAGNOSTIC_PUB::IPC_DIAGNOSTIC_PUB()
{
    pub = nh.advertise<katech_diagnostic_msgs::ipc_diagnostic_msg>("/diagnostic/ipc", 1);
    sub = nh.subscribe("/percept_topic", 5, &IPC_DIAGNOSTIC_PUB::percept_callback, this);

    timer_ = nh.createTimer(ros::Duration(0.1), &IPC_DIAGNOSTIC_PUB::timer_callback, this);

    percept_callback_cnt = 0;
    percept_callback_cnt_old = 0;
    miss_cnt = 0;
}

IPC_DIAGNOSTIC_PUB::~IPC_DIAGNOSTIC_PUB()
{

}

void IPC_DIAGNOSTIC_PUB::timer_callback(const ros::TimerEvent&)
{
    // /percept_topic은 10Hz 발행이라 100ms 타이머 윈도우와 거의 동기 — 단일 틱 miss는 정상 지터.
    // 3틱(≈300ms) 연속 miss일 때만 fault로 보고하여 false yellow blink 방지.
    if(percept_callback_cnt == percept_callback_cnt_old)
    {
        miss_cnt++;
        if(miss_cnt > 2)
        {
            ipc_msg.IPC_SWC_StatCode = 1;
        }
    }
    else
    {
        miss_cnt = 0;
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
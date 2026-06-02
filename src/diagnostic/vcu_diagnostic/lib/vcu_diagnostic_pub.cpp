#include <vcu_diagnostic_pub.h>

VCU_DIAGNOSTIC_PUB::VCU_DIAGNOSTIC_PUB()
{
    pub = nh.advertise<katech_diagnostic_msgs::vcu_diagnostic_msg>("/diagnostic/vcu", 1);
    sub = nh.subscribe("/sensors/v_can", 5, &VCU_DIAGNOSTIC_PUB::vcu_callback, this);

    for(int i = 0; i < 6; i++)
    {
        life_count_cur[i] = 0;
        life_count_old[i] = 0;
        last_change_time[i] = ros::Time(0);  // 초기엔 stale -> 메시지 수신 전까지 StatCode=1
    }

    timer_ = nh.createTimer(ros::Duration(0.1), &VCU_DIAGNOSTIC_PUB::timer_callback, this);
}

VCU_DIAGNOSTIC_PUB::~VCU_DIAGNOSTIC_PUB()
{

}

void VCU_DIAGNOSTIC_PUB::timer_callback(const ros::TimerEvent&)
{
    const double STALE_TIMEOUT = 0.5;  // s. 가장 느린 TurnSignalInfo(200ms 주기)를 여유 있게 덮음
    ros::Time now = ros::Time::now();

    bool all_alive = true;
    for(int i = 0; i < 6; i++)
    {
        if((now - last_change_time[i]).toSec() > STALE_TIMEOUT)
        {
            all_alive = false;
            break;
        }
    }

    if(all_alive)
    {
        vcu_msg.VCU_StatCode = 0;
        vcu_msg.VCU_AliveCount++;
    }
    else
    {
        vcu_msg.VCU_StatCode = 1;
    }
    pub.publish(vcu_msg);
}

void VCU_DIAGNOSTIC_PUB::vcu_callback(const katech_custom_msgs::v_can_msg::ConstPtr& msg)
{
    ros::Time now = ros::Time::now();

    uint8_t cur[6] = {
        msg->life_count_gearinfo,
        msg->life_count_turnsignalinfo,
        msg->life_count_longitudinalinfo,
        msg->life_count_steeringinfo,
        msg->life_count_wheelinfo,
        msg->life_count_dynamicinfo
    };

    for(int i = 0; i < 6; i++)
    {
        life_count_cur[i] = cur[i];
        if(life_count_cur[i] != life_count_old[i])
        {
            life_count_old[i] = life_count_cur[i];
            last_change_time[i] = now;
        }
    }
}

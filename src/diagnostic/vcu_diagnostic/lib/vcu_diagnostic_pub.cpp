#include <vcu_diagnostic_pub.h>

VCU_DIAGNOSTIC_PUB::VCU_DIAGNOSTIC_PUB()
{
    pub = nh.advertise<katech_diagnostic_msgs::vcu_diagnostic_msg>("/diagnostic/vcu", 1);
    sub = nh.subscribe("/sensors/ioniq5_ad_can", 5, &VCU_DIAGNOSTIC_PUB::vcu_callback, this);

    life_count_cur = 0;
    life_count_old = 0;
    last_change_time = ros::Time(0);  // 초기엔 stale -> 메시지 수신 전까지 StatCode=1
    error_code_cur = 0;

    timer_ = nh.createTimer(ros::Duration(0.1), &VCU_DIAGNOSTIC_PUB::timer_callback, this);
}

VCU_DIAGNOSTIC_PUB::~VCU_DIAGNOSTIC_PUB()
{

}

void VCU_DIAGNOSTIC_PUB::timer_callback(const ros::TimerEvent&)
{
    const double STALE_TIMEOUT = 0.5;  // s. AutonomousState(~40Hz) life_count 동결/침묵 검출
    ros::Time now = ros::Time::now();

    // life_count 가 STALE_TIMEOUT 내 안 변하면(메시지 침묵 or 동결) 고장
    bool life_count_stale = (now - last_change_time).toSec() > STALE_TIMEOUT;

    if(life_count_stale)
    {
        vcu_msg.VCU_StatCode = 1;  // life_count 안 들어옴/동결
    }
    else if(error_code_cur != 0)
    {
        vcu_msg.VCU_StatCode = 1;  // life_count 는 살아있으나 error_code 비정상
    }
    else
    {
        vcu_msg.VCU_StatCode = 0;
        vcu_msg.VCU_AliveCount++;
    }
    pub.publish(vcu_msg);
}

void VCU_DIAGNOSTIC_PUB::vcu_callback(const katech_custom_msgs::ioniq5_ad_can_msg::ConstPtr& msg)
{
    ros::Time now = ros::Time::now();

    life_count_cur = msg->autonomous_life_count;
    if(life_count_cur != life_count_old)
    {
        life_count_old = life_count_cur;
        last_change_time = now;
    }

    error_code_cur = msg->error_code;
}

#include <cam_diagnostic_pub.h>

CAM_DIAGNOSTIC_PUB::CAM_DIAGNOSTIC_PUB()
{
    pub = nh.advertise<katech_diagnostic_msgs::cam_diagnostic_msg>("/diagnostic/cam", 1);
    lane_sub = nh.subscribe("/sensors/vision_lane", 5, &CAM_DIAGNOSTIC_PUB::cam_lane_callback, this);
    obj_sub = nh.subscribe("/sensors/vision_pos", 5, &CAM_DIAGNOSTIC_PUB::cam_obj_callback, this);

    timer_ = nh.createTimer(ros::Duration(0.1), &CAM_DIAGNOSTIC_PUB::timer_callback, this);
}

CAM_DIAGNOSTIC_PUB::~CAM_DIAGNOSTIC_PUB()
{

}

void CAM_DIAGNOSTIC_PUB::timer_callback(const ros::TimerEvent&)
{
    if((obj_callback_cnt == obj_callback_cnt_old) && (lane_callback_cnt == lane_callback_cnt_old))
    {

    }
    else
    {
        cam_msg.time = ros::Time::now();

        cam_msg.CAM_AliveCount++;
        ROS_INFO("timer_callback");

        obj_callback_cnt_old = obj_callback_cnt;
        lane_callback_cnt_old = lane_callback_cnt;
    }

    pub.publish(cam_msg);
}

void CAM_DIAGNOSTIC_PUB::cam_lane_callback(const mmc_msgs::lane_array_msg::ConstPtr& msg)
{
    lane_callback_cnt++;
}

void CAM_DIAGNOSTIC_PUB::cam_obj_callback(const mmc_msgs::object_array_msg::ConstPtr& msg)
{
    obj_callback_cnt++;
}

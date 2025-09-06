#include <ros/ros.h>
#include <rviz_filter.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_filter");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    RVIZ_FILTER rviz_filter_;

    rviz_filter_.sub1 = node.subscribe("/track_Multi_RS", 1, &RVIZ_FILTER::percept_callback, &rviz_filter_);
    rviz_filter_.sdsm_sub = node.subscribe("/obu/sdsm", 1, &RVIZ_FILTER::sdsm_callback, &rviz_filter_);
    rviz_filter_.local_sub = node.subscribe("/localization/to_control_team", 1, &RVIZ_FILTER::local_callback, &rviz_filter_);

    rviz_filter_.marker_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    ros::waitForShutdown();

    return 0;
}
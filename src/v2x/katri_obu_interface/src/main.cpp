#include <ros/ros.h>
#include <katri_obu_interface/katri_v2x.h>
#include <v2x_msgs/intersection_array_msg.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "katri_v2x_node");
    ros::NodeHandle node("~");

    KATRI_V2X katri;

    katri.pub = node.advertise<v2x_msgs::intersection_array_msg>("katri_spat", 1);

    katri.loop();
}
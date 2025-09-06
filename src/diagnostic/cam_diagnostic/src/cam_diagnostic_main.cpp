#include <cam_diagnostic_pub.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cam_diagnostic_pub_node");

    CAM_DIAGNOSTIC_PUB cam_diagnostic;

    ros::spin();

    return 0;
}
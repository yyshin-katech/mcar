#include <lidar_diagnostic_pub.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_diagnostic_node");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    LIDAR_DIAGNOSTIC_PUB lidar_diagnostic;

    ros::waitForShutdown();
    return 0;
}
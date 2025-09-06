#include <radar_diagnostic_pub.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_diagnostic_node");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    RADAR_DIAGNOSTIC_PUB radar_diagnostic;

    ros::waitForShutdown();
    return 0;
}
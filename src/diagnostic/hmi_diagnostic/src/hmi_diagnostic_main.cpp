#include <hmi_diagnostic_pub.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hmi_diagnostic_node");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    HMI_DIAGNOSTIC_PUB hmi_diagnostic;

    ros::waitForShutdown();
    return 0;
}
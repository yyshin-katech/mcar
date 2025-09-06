#include <v2x_diagnostic_pub.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "v2x_diagnostic_node");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    V2X_DIAGNOSTIC_PUB v2x_diagnostic;

    ros::waitForShutdown();
    return 0;
}
#include <vcu_diagnostic_pub.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vcu_diagnostic_node");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    VCU_DIAGNOSTIC_PUB vcu_diagnostic;

    ros::waitForShutdown();
    return 0;
}
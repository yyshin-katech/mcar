#include <ipc_diagnostic_pub.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ipc_diagnostic_pub_node");

    IPC_DIAGNOSTIC_PUB ipc_diagnostic;

    ros::spin();

    return 0;
}
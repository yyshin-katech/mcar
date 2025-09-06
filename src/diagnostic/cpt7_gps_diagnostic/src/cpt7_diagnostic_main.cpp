#include <cpt7_diagnostic_pub.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpt7_diagnostic_pub_node");

    CPT7_DIAGNOSTIC_PUB CPT7_CW;

    ros::spin();

    return 0;
}
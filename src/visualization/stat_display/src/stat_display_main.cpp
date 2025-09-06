#include <stat_display.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stat_display_node");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    STAT_DISPLAY stat_display;

    ros::waitForShutdown();
    return 0;
}
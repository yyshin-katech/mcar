#include <lidar_diagnostic_pub.h>

LIDAR_DIAGNOSTIC_PUB::LIDAR_DIAGNOSTIC_PUB()
    : center_failed(false), right_failed(false), left_failed(false), ping_thread_stop(false)
{
    lidar = {{"192.168.1.201", 5578},
            {"192.168.1.203", 6688},
            {"192.168.1.202", 4455}};

    pub = nh.advertise<katech_diagnostic_msgs::lidar_diagnostic_msg>("/diagnostic/lidar", 1);
    sub = nh.subscribe("/percept_topic", 5, &LIDAR_DIAGNOSTIC_PUB::percept_callback, this);

    timer_ = nh.createTimer(ros::Duration(0.1), &LIDAR_DIAGNOSTIC_PUB::timer_callback, this);

    ping_thread = std::thread(&LIDAR_DIAGNOSTIC_PUB::pingLoop, this);
}

LIDAR_DIAGNOSTIC_PUB::~LIDAR_DIAGNOSTIC_PUB()
{
    ping_thread_stop = true;
    if (ping_thread.joinable())
    {
        ping_thread.join();
    }
}

void LIDAR_DIAGNOSTIC_PUB::timer_callback(const ros::TimerEvent&)
{
    lidar_msg.LIDAR_Center_StatCode = center_failed.load() ? 1 : 0;
    lidar_msg.LIDAR_Right_StatCode  = right_failed.load()  ? 1 : 0;
    lidar_msg.LIDAR_Left_StatCode   = left_failed.load()   ? 1 : 0;

    lidar_msg.time = ros::Time::now();
    pub.publish(lidar_msg);
}

void LIDAR_DIAGNOSTIC_PUB::percept_callback(const perception_ros_msg::RsPerceptionMsg::ConstPtr& msg)
{
    lidar_msg.LIDAR_AliveCount++;
}

bool LIDAR_DIAGNOSTIC_PUB::pingCheck(const std::string& ip)
{
    std::string cmd = "ping -c 1 -W 1 " + ip + " > /dev/null 2>&1";
    int result = system(cmd.c_str());
    return (result == 0);
}

void LIDAR_DIAGNOSTIC_PUB::pingLoop()
{
    while (!ping_thread_stop.load())
    {
        center_failed.store(!pingCheck(lidar[0].ip));
        if (ping_thread_stop.load()) break;

        right_failed.store(!pingCheck(lidar[1].ip));
        if (ping_thread_stop.load()) break;

        left_failed.store(!pingCheck(lidar[2].ip));

        // ping_thread_stop을 100ms 단위로 체크 (총 1초 sleep)
        for (int i = 0; i < 10 && !ping_thread_stop.load(); ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

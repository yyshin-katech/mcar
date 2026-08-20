#include <v2x_diagnostic_pub.h>

V2X_DIAGNOSTIC_PUB::V2X_DIAGNOSTIC_PUB()
    : ping_failed(false), ping_thread_stop(false)
{
    v2x_obu = {{"192.168.1.2", 60000}};

    pub = nh.advertise<katech_diagnostic_msgs::v2x_diagnostic_msg>("/diagnostic/v2x", 1);
    sub = nh.subscribe("/siheung_spat", 1, &V2X_DIAGNOSTIC_PUB::v2x_callback, this);

    timer_ = nh.createTimer(ros::Duration(0.1), &V2X_DIAGNOSTIC_PUB::timer_callback, this);

    conn_stat = 0;
    v2x_callback_cnt = 0;
    v2x_callback_cnt_old = 0;

    ping_thread = std::thread(&V2X_DIAGNOSTIC_PUB::pingLoop, this);
}

V2X_DIAGNOSTIC_PUB::~V2X_DIAGNOSTIC_PUB()
{
    ping_thread_stop = true;
    if (ping_thread.joinable())
    {
        ping_thread.join();
    }
}

void V2X_DIAGNOSTIC_PUB::timer_callback(const ros::TimerEvent&)
{
    static uint8_t count = 0;

    if(v2x_callback_cnt == v2x_callback_cnt_old)
    {
        count++;
        if(count > 30)
        {
            v2x_msg.V2X_StatCode = 0;
            count = 0;
        }
        else
        {
            v2x_msg.V2X_StatCode = 0;
        }
    }
    else
    {
        v2x_callback_cnt_old = v2x_callback_cnt;
        v2x_msg.V2X_StatCode = 0;
        v2x_msg.V2X_AliveCount++;
    }

    // OBU ping 실패 시 즉시 warning (SPaT 30틱 대기 우회)
    if(ping_failed.load())
    {
        v2x_msg.V2X_StatCode = 1;
    }

    v2x_msg.time = ros::Time::now();

    pub.publish(v2x_msg);
}

void V2X_DIAGNOSTIC_PUB::v2x_callback(const v2x_msgs::intersection_array_msg::ConstPtr& msg)
{
    v2x_callback_cnt++;
}

bool V2X_DIAGNOSTIC_PUB::pingCheck(const std::string& ip)
{
    std::string cmd = "ping -c 1 -W 1 " + ip + " > /dev/null 2>&1";
    int result = system(cmd.c_str());
    return (result == 0);
}

void V2X_DIAGNOSTIC_PUB::pingLoop()
{
    const std::string ip = v2x_obu[0].ip;
    while (!ping_thread_stop.load())
    {
        bool failed = !pingCheck(ip);
        ping_failed.store(failed);

        for (int i = 0; i < 10 && !ping_thread_stop.load(); ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

#include <lidar_diagnostic_pub.h>

LIDAR_DIAGNOSTIC_PUB::LIDAR_DIAGNOSTIC_PUB()
{
    lidar = {{"192.168.1.201", 5578},
            {"192.168.1.203", 6688},
            {"192.168.1.202", 4455}};

    connection_stat = {0, 0, 0};

    pub = nh.advertise<katech_diagnostic_msgs::lidar_diagnostic_msg>("/diagnostic/lidar", 1);
    sub = nh.subscribe("/percept_topic", 5, &LIDAR_DIAGNOSTIC_PUB::percept_callback, this);

    timer_ = nh.createTimer(ros::Duration(1.0), &LIDAR_DIAGNOSTIC_PUB::timer_callback, this);
}

LIDAR_DIAGNOSTIC_PUB::~LIDAR_DIAGNOSTIC_PUB()
{

}

void LIDAR_DIAGNOSTIC_PUB::timer_callback(const ros::TimerEvent&)
{
    static uint8_t count = 0;
    if(count % 3 == 0)
    {
        connection_stat.Center = this->checkCenterLidarConnection();
    }
    else if(count % 3 == 1)
    {
        connection_stat.Right = this->checkRightLidarConnection();
    }
    else if(count % 3 == 2)
    {
        connection_stat.Left = this->checkLeftLidarConnection();
    }
    count++;
    
    if(connection_stat.Center == 1)
    {
        lidar_msg.LIDAR_Center_StatCode = 1;
    }
    else
    {
        lidar_msg.LIDAR_Center_StatCode = 0;
    }

    if(connection_stat.Right == 1)
    {
        lidar_msg.LIDAR_Right_StatCode = 1;
    }
    else
    {
        lidar_msg.LIDAR_Right_StatCode = 0;
    }

    if(connection_stat.Left == 1)
    {
        lidar_msg.LIDAR_Left_StatCode = 1;
    }
    else
    {
        lidar_msg.LIDAR_Left_StatCode = 0;
    }
    lidar_msg.time = ros::Time::now();

    pub.publish(lidar_msg);
}

void LIDAR_DIAGNOSTIC_PUB::percept_callback(const perception_ros_msg::RsPerceptionMsg::ConstPtr& msg)
{
    static uint8_t callback_count = 0;

    if(callback_count++ > 9)
    {
        lidar_msg.LIDAR_AliveCount++;
        callback_count = 0;
    }
}

bool LIDAR_DIAGNOSTIC_PUB::checkConnection(const std::string& ip, uint16_t port)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) return false;

    // 소켓을 논블로킹 모드로 설정
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);

    int result = connect(sock, (struct sockaddr*)&addr, sizeof(addr));
    if (result == 0) {
        close(sock);
        return true; // 즉시 연결 성공
    }

    // EINPROGRESS이면 연결 시도 중
    if (errno != EINPROGRESS) {
        close(sock);
        return false;
    }

    fd_set writefds;
    FD_ZERO(&writefds);
    FD_SET(sock, &writefds);

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    result = select(sock + 1, nullptr, &writefds, nullptr, &tv);
    if (result > 0) {
        int sock_error;
        socklen_t len = sizeof(sock_error);
        getsockopt(sock, SOL_SOCKET, SO_ERROR, &sock_error, &len);
        close(sock);
        return (sock_error == 0); // 0이면 연결 성공
    }

    // 타임아웃 또는 select 실패
    close(sock);
    return false;
    // struct sockaddr_in server;
    // server.sin_family = AF_INET;
    // server.sin_port = htons(port);
    // if (inet_pton(AF_INET, ip.c_str(), &server.sin_addr) <= 0) {
    //     close(sock);
    //     return false;
    // }

    // bool isConnected = (connect(sock, (struct sockaddr *)&server, sizeof(server)) == 0);
    // close(sock);
    // return isConnected;
}

bool LIDAR_DIAGNOSTIC_PUB::checkCenterLidarConnection()
{
    std::string ip = lidar[0].ip;
    uint16_t port = static_cast<uint16_t>(lidar[0].port);

    bool result = this->checkConnection(ip, port);

    return result;
}

bool LIDAR_DIAGNOSTIC_PUB::checkRightLidarConnection()
{
    std::string ip = lidar[1].ip;
    uint16_t port = static_cast<uint16_t>(lidar[1].port);

    bool result = this->checkConnection(ip, port);

    return result;
}

bool LIDAR_DIAGNOSTIC_PUB::checkLeftLidarConnection()
{
    std::string ip = lidar[2].ip;
    uint16_t port = static_cast<uint16_t>(lidar[2].port);

    bool result = this->checkConnection(ip, port);

    return result;
}
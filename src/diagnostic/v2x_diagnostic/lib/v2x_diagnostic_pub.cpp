#include <v2x_diagnostic_pub.h>

V2X_DIAGNOSTIC_PUB::V2X_DIAGNOSTIC_PUB()
{
    v2x_obu = {{"192.168.1.2", 60000}};
    
    pub = nh.advertise<katech_diagnostic_msgs::v2x_diagnostic_msg>("/diagnostic/v2x", 1);
    sub = nh.subscribe("/katri_v2x_node/katri_spat", 1, &V2X_DIAGNOSTIC_PUB::v2x_callback, this);

    timer_ = nh.createTimer(ros::Duration(1.0), &V2X_DIAGNOSTIC_PUB::timer_callback, this);

    conn_stat = 0;
    v2x_callback_cnt = 0;
    v2x_callback_cnt_old = 0;
}

V2X_DIAGNOSTIC_PUB::~V2X_DIAGNOSTIC_PUB()
{

}

void V2X_DIAGNOSTIC_PUB::timer_callback(const ros::TimerEvent&)
{   
    uint8_t ret = 0;
    static uint8_t count = 0;
    std::string ip = v2x_obu[0].ip;
    uint16_t port = v2x_obu[0].port;
    
    count++;

    if(v2x_callback_cnt == v2x_callback_cnt_old)
    {
        v2x_msg.V2X_StatCode = 1;
    }   
    else
    {
        v2x_callback_cnt_old = v2x_callback_cnt;
        v2x_msg.V2X_StatCode = 0;
        v2x_msg.V2X_AliveCount++;
    }

    if(count % 3 == 0)
    {
        ret = this->checkConnection(ip, port);
    }

    if(ret == false) v2x_msg.V2X_StatCode = 2;
    
    v2x_msg.time = ros::Time::now();

    pub.publish(v2x_msg);
}

void V2X_DIAGNOSTIC_PUB::v2x_callback(const v2x_msgs::intersection_array_msg::ConstPtr& msg)
{
    v2x_callback_cnt++;
}

bool V2X_DIAGNOSTIC_PUB::checkConnection(const std::string& ip, uint16_t port)
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
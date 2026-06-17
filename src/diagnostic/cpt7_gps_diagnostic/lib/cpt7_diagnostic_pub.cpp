#include <cpt7_diagnostic_pub.h>


CPT7_DIAGNOSTIC_PUB::CPT7_DIAGNOSTIC_PUB()
    : network_failed(false), ping_thread_stop(false)
{
    pub = nh.advertise<katech_diagnostic_msgs::cpt7_gps_diagnostic_msg>("diagnostic/cpt7_gps", 1);
    sub = nh.subscribe("/ublox/navpvt", 1, &CPT7_DIAGNOSTIC_PUB::navpvt_callback, this);

    timer_ = nh.createTimer(ros::Duration(0.1), &CPT7_DIAGNOSTIC_PUB::timerCallback, this);

    alive_cnt = 0;
    msg_received = false;
    navpvt_miss_cnt = 0;

    cpt7_msg.IMU_StatCode = 0;
    cpt7_msg.GPS_StatCode = 0;
    cpt7_msg.INS_StatCode = 0;
    cpt7_msg.GPSRTK_StatCode = 0;
    cpt7_msg.GPS_INS_SolutionStat = 0x01;
    cpt7_msg.GPS_INS_AliveCnt = 0;
    cpt7_msg.lon_std = 0;
    cpt7_msg.lat_std = 0;
    cpt7_msg.Network_Status = 0;

    // GPS는 /dev/ttyACM0 USB serial이라 인터넷 ping은 무관 — 비활성
    // ping_thread = std::thread(&CPT7_DIAGNOSTIC_PUB::pingLoop, this);
}

CPT7_DIAGNOSTIC_PUB::~CPT7_DIAGNOSTIC_PUB()
{
    ping_thread_stop = true;
    if (ping_thread.joinable())
    {
        ping_thread.join();
    }
}

// NavPVT fixType
// 0: NO_FIX
// 1: DEAD_RECKONING_ONLY
// 2: 2D-Fix
// 3: 3D-Fix
// 4: GNSS + Dead Reckoning combined
// 5: TIME_ONLY
//
// NavPVT flags carrier phase (bits 6-7)
// 0: No carrier phase
// 64: Float solution
// 128: Fixed solution

void CPT7_DIAGNOSTIC_PUB::navpvt_callback(const ublox_msgs::NavPVT::ConstPtr& msg)
{
    msg_received = true;

    // carrSoln (flags bit 6-7) → GPSRTK_StatCode 매핑
    // 0: No carrier phase, 1: Float, 2: Fixed
    cpt7_msg.GPSRTK_StatCode = (msg->flags >> 6) & 0x03;

    // hAcc/vAcc (mm → m)
    cpt7_msg.lon_std = msg->hAcc * 0.001;
    cpt7_msg.lat_std = msg->vAcc * 0.001;

    // fixType 기반 GPS_INS_SolutionStat
    if(msg->fixType >= 2)
    {
        cpt7_msg.GPS_INS_SolutionStat = 0x00;  // SOL_COMPUTED
    }
    else
    {
        cpt7_msg.GPS_INS_SolutionStat = 0x01;  // INSUFFICIENT_OBS
    }
}

void CPT7_DIAGNOSTIC_PUB::timerCallback(const ros::TimerEvent&)
{
    cpt7_msg.Network_Status = network_failed.load() ? 1 : 0;

    cpt7_msg.GPS_INS_AliveCnt = alive_cnt++;
    if(!msg_received)
    {
        cpt7_msg.GPS_INS_SolutionStat = 0x01;  // no data
        if(navpvt_miss_cnt < 255) navpvt_miss_cnt++;
    }
    else
    {
        navpvt_miss_cnt = 0;
    }
    msg_received = false;

    // GPS 전원 분리 등으로 NavPVT가 끊겨도 이 타이머는 계속 돌며 AliveCnt를 올리고
    // GPSRTK_StatCode/std는 navpvt_callback에서만 갱신돼 마지막 정상값으로 freeze된다.
    // 그러면 소비자(stat_display/pyqt_hmi)가 단절을 감지 못 하므로, NavPVT가 일정 시간
    // (3 tick = 300ms) 끊기면 소비자가 동일하게 보는 GPSRTK_StatCode를 No RTK(0)로 내려
    // gps_status=2(고장)로 표출되게 한다. NavPVT는 20Hz라 정상 시 오판 없음.
    if(navpvt_miss_cnt >= 3)
    {
        cpt7_msg.GPSRTK_StatCode = 0;
    }

    pub.publish(cpt7_msg);
}

bool CPT7_DIAGNOSTIC_PUB::pingCheck(const std::string& ip)
{
    std::string cmd = "ping -c 1 -W 1 " + ip + " > /dev/null 2>&1";
    int result = system(cmd.c_str());
    return (result == 0) ? 0 : 1;  // 성공 시 0, 실패 시 1
}

void CPT7_DIAGNOSTIC_PUB::pingLoop()
{
    const std::string ip = "8.8.8.8";
    while (!ping_thread_stop.load())
    {
        bool failed = (pingCheck(ip) == 1);
        network_failed.store(failed);

        for (int i = 0; i < 10 && !ping_thread_stop.load(); ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

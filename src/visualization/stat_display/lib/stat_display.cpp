#include <stat_display.h>

// 1280 * 720


STAT_DISPLAY::STAT_DISPLAY()
{
    popup_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/popup", 1);
    gps_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/gps_stat", 1);
    adcu_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/adcu_stat", 1);
    lidar_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/lidar_stat", 1);
    radar_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/radar_stat", 1);
    v2x_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/v2x_stat", 1);
    hmi_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/hmi_stat", 1);
    vcu_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/vcu_stat", 1);
    cam_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/cam_stat", 1);
    ipc_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/ipc_stat", 1);
    local_text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/local_info_stat", 1);
    mode_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/mode_text", 1);
    odd_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/odd_text", 1);
    speed_limit_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/speed_limit_text", 1);

    sound_pub = nh.advertise<sound_play::SoundRequest>("/robotsound", 1);

    katech_diag_pub = nh.advertise<katech_diagnostic_msgs::katech_diagnostic_msg>("/diagnostic/system", 1);

    // 신호등 Publisher 추가
    traffic_light_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/traffic_light_stat", 1);
    
    gps_sub = nh.subscribe("/diagnostic/cpt7_gps", 1, &STAT_DISPLAY::diagnostic_gps_callback, this);
    adcu_sub = nh.subscribe("/diagnostic/adcu", 1, &STAT_DISPLAY::diagnostic_adcu_callback, this);
    lidar_sub = nh.subscribe("/diagnostic/lidar", 1, &STAT_DISPLAY::diagnostic_lidar_callback, this);
    radar_sub = nh.subscribe("/diagnostic/radar", 1, &STAT_DISPLAY::diagnostic_radar_callback, this);
    v2x_sub = nh.subscribe("/diagnostic/v2x", 1, &STAT_DISPLAY::diagnostic_v2x_callback, this);
    hmi_sub = nh.subscribe("/diagnostic/hmi", 1, &STAT_DISPLAY::diagnostic_hmi_callback, this);
    vcu_sub = nh.subscribe("/diagnostic/vcu", 1, &STAT_DISPLAY::diagnostic_vcu_callback, this);
    cam_sub = nh.subscribe("/diagnostic/cam", 1, &STAT_DISPLAY::diagnostic_cam_callback, this);
    ipc_sub = nh.subscribe("/diagnostic/ipc", 1, &STAT_DISPLAY::diagnostic_ipc_callback, this);

    local_sub = nh.subscribe("/localization/to_control_team", 1, &STAT_DISPLAY::local_callback, this);
    chassis_sub_node = nh.subscribe("/sensors/chassis", 1, &STAT_DISPLAY::chassis_callback_func, this);

    // 신호등 Subscriber 추가 (토픽 이름은 실제 사용하는 것으로 변경)
    traffic_light_sub = nh.subscribe("/katri_v2x_node/katri_spat", 1, &STAT_DISPLAY::traffic_light_callback, this);
    
    timer_ = nh.createTimer(ros::Duration(1.0), &STAT_DISPLAY::timerCallback, this);
    diag_timer_ = nh.createTimer(ros::Duration(0.1), &STAT_DISPLAY::diag_timerCallback, this);

    gps_status = 2; // 0: 정상 1: warning 2:error
    adcu_status = 2;
    lidar_status = 2;
    radar_status = 2;
    v2x_status = 2;
    hmi_status = 2;
    vcu_status = 2;
    cam_status = 2;
    ipc_status = 2;
}

STAT_DISPLAY::~STAT_DISPLAY()
{
    
}

void STAT_DISPLAY::traffic_light_callback(const v2x_msgs::intersection_array_msg::ConstPtr& msg)
{
    uint16_t target_intersection_id = local_msg.look_at_IntersectionID;
    uint8_t target_signal_group_id = local_msg.look_at_signalGroupID;
    intersectionid = local_msg.look_at_IntersectionID;
    if (target_intersection_id == 0)
    {
        return;
    }

    for (const auto& intersection : msg->data)
    {
        // 교차로 ID 매칭
        if (intersection.IntersectionID == target_intersection_id)
        {
            // Movements는 단일 객체이므로 직접 접근
            const auto& movement = intersection.Movements;
            
            // SignalGroupID 체크 (0이 아닐 때만)
            if (target_signal_group_id != 0 && 
                movement.SignalGroupID != target_signal_group_id)
            {
                continue;  // SignalGroupID가 다르면 건너뛰기
            }
            
            // 남은 시간 및 색상 정보 저장
            traffic_light_time = movement.TimeChangeDetails;
            
            switch(movement.MovementPhaseStatus)
            {
                case 3:  traffic_light_color = 1; break;  // 초록
                case 8:  traffic_light_color = 2; break;  // 주황
                case 6:  traffic_light_color = 3; break;  // 빨강
                default: traffic_light_color = 0; break;  // 알 수 없음
            }
            
            // ROS_INFO("🚦 [ID:%d, SG:%d] 색상=%d, 남은시간=%.1f초",
            //          target_intersection_id, 
            //          target_signal_group_id,
            //          traffic_light_color, 
            //          traffic_light_time / 10.0);
            
            return;  // 찾았으면 종료
        }
    }

}

void STAT_DISPLAY::chassis_callback_func(const mmc_msgs::chassis_msg::ConstPtr& msg)
{
    lo_chassis_msg = *msg;
}

void STAT_DISPLAY::POPUP_Text_Gen(const std::string& message)
{
    POPUP_text.text = message;
    std_msgs::ColorRGBA state_color;
    int32_t msg_len = POPUP_text.text.length();
    int32_t width = 20;
    int32_t height = 80;

    POPUP_text.action = POPUP_text.ADD;
    POPUP_text.font = "DejaVu Sans Mono";
    POPUP_text.text_size = 40;
    POPUP_text.width = width*POPUP_text.text.length();
    POPUP_text.height = height;
    if(msg_len > 30)
    {
        POPUP_text.left = 1280 - 23*POPUP_text.text.length();
    }
    else
    {
        POPUP_text.left = 1280 - 35*POPUP_text.text.length();
    }
    
    POPUP_text.top = 300;

    state_color.r = 1;
    state_color.g = 0;
    state_color.b = 0;
    state_color.a = 1;
    POPUP_text.fg_color = state_color;

    state_color.r = 0.4;
    state_color.g = 0.4;
    state_color.b = 0.4;
    state_color.a = 0.7;
    POPUP_text.bg_color = state_color;
    popup_pub.publish(POPUP_text);
}

void STAT_DISPLAY::POPUP_Text_Clear()
{
    POPUP_text.action = POPUP_text.DELETE;
    popup_pub.publish(POPUP_text);
}

void STAT_DISPLAY::diagnostic_gps_callback(const katech_diagnostic_msgs::cpt7_gps_diagnostic_msg::ConstPtr& msg)
{   
    cpt7_msg = *msg;
}

void STAT_DISPLAY::diagnostic_adcu_callback(const katech_diagnostic_msgs::k_adcu_diagnostic_msg::ConstPtr& msg)
{
    adcu_msg = *msg;
}

void STAT_DISPLAY::diagnostic_lidar_callback(const katech_diagnostic_msgs::lidar_diagnostic_msg::ConstPtr& msg)
{
    lidar_msg = *msg;
}

void STAT_DISPLAY::diagnostic_radar_callback(const katech_diagnostic_msgs::radar_diagnostic_msg::ConstPtr& msg)
{
    radar_msg = *msg;
}

void STAT_DISPLAY::diagnostic_v2x_callback(const katech_diagnostic_msgs::v2x_diagnostic_msg::ConstPtr& msg)
{
    v2x_msg = *msg;
}

void STAT_DISPLAY::diagnostic_hmi_callback(const katech_diagnostic_msgs::hmi_diagnostic_msg::ConstPtr& msg)
{
    hmi_msg = *msg;
}

void STAT_DISPLAY::diagnostic_vcu_callback(const katech_diagnostic_msgs::vcu_diagnostic_msg::ConstPtr& msg)
{
    vcu_msg = *msg;
}

void STAT_DISPLAY::diagnostic_cam_callback(const katech_diagnostic_msgs::cam_diagnostic_msg::ConstPtr& msg)
{
    cam_msg = *msg;
}

void STAT_DISPLAY::diagnostic_ipc_callback(const katech_diagnostic_msgs::ipc_diagnostic_msg::ConstPtr& msg)
{
    ipc_msg = *msg;
}

void STAT_DISPLAY::local_callback(const mmc_msgs::to_control_team_from_local_msg::ConstPtr& msg)
{
    local_msg = *msg;
}

void STAT_DISPLAY::diag_timerCallback(const ros::TimerEvent&)
{
    

    this->TRAFFIC_LIGHT_Text_Gen();

    
}

void STAT_DISPLAY::timerCallback(const ros::TimerEvent&)
{
    this->GPS_Text_Gen();
    this->ADCU_Text_Gen();
    this->LIDAR_Text_Gen();
    this->RADAR_Text_Gen();
    this->V2X_Text_Gen();
    this->HMI_Text_Gen();
    this->VCU_Text_Gen();
    this->CAM_Text_Gen();
    this->IPC_Text_Gen();

    this->Local_Text_Gen();

    this->MODE_Text_Gen();

    this->ODD_Text_Gen();

    this->system_status_check();
    katech_diag_pub.publish(katech_diag_msg);
    this->SPEED_LIMIT_Text_Gen();
}

void STAT_DISPLAY::GPS_Text_Gen()
{
    ros::Time now = ros::Time::now();

    GPS_text.text = "GPSRTK";

    std_msgs::ColorRGBA state_color;

    int32_t width = 200;
    int32_t height = 30;

    GPS_text.action = GPS_text.ADD;
    GPS_text.font = "DejaVu Sans Mono";
    GPS_text.text_size = 20;
    GPS_text.width = width;
    GPS_text.height = height;
    GPS_text.left = 20;
    GPS_text.top = 50;

    GPS_AliveCnt_Check(cpt7_msg.GPS_INS_AliveCnt);

    if(gps_status == 0)
    {   //흰색 정상
        state_color.r = 0;
        state_color.g = 0.8;
        state_color.b = 0;
        state_color.a = 1;
        GPS_text.fg_color = state_color;
    }
    else if(gps_status == 1)
    {   // 주황 warning
        state_color.r = 1;
        state_color.g = 0.5;
        state_color.b = 0;
        state_color.a = 1;
        GPS_text.fg_color = state_color;
    }
    else
    {   // 빨강 error
        state_color.r = 1;
        state_color.g = 0;
        state_color.b = 0;
        state_color.a = 1;
        GPS_text.fg_color = state_color;
    }

    if (local_msg.GPS_Over == 1)
    {
        state_color.r = 1;
        state_color.g = 0.5;
        state_color.b = 0;
        state_color.a = 1;
        GPS_text.fg_color = state_color; 
    }

    state_color.r = 0.4;
    state_color.g = 0.4;
    state_color.b = 0.4;
    state_color.a = 0.5;
    GPS_text.bg_color = state_color;
    gps_pub.publish(GPS_text);

    katech_diag_msg.gps_status = gps_status;

}
void STAT_DISPLAY::GPS_AliveCnt_Check(uint8_t current_cnt)
{
    //GPS AliveCount Check
    current_gps_cnt = current_cnt;
    if(current_gps_cnt == last_gps_cnt)
    {
        unchanged_gps_cnt++;
    }
    else
    {
        unchanged_gps_cnt = 0;
        last_gps_cnt = current_gps_cnt;
    }

    if(unchanged_gps_cnt > 2)
    {
        gps_status = 2;
    }
    else
    {
        gps_status = 0;
    }
    
    // if(local_msg.GPS_Over == 1)
    // {
    //     gps_status = 1;
    // }
}

void STAT_DISPLAY::ADCU_Text_Gen()
{
    ros::Time now = ros::Time::now();
    
    ADCU_text.text = "K-ADCU";

    std_msgs::ColorRGBA state_color;

    int32_t width = 200;
    int32_t height = 30;

    ADCU_text.action = ADCU_text.ADD;
    ADCU_text.font = "DejaVu Sans Mono";
    ADCU_text.text_size = 20;
    ADCU_text.width = width;
    ADCU_text.height = height;
    ADCU_text.left = 20;
    ADCU_text.top = 20;

    ADCU_AliveCnt_Check(adcu_msg.ADCU_AliveCount);

    if(adcu_status == 0)
    {   //흰색 정상
        state_color.r = 0;
        state_color.g = 0.8;
        state_color.b = 0;
        state_color.a = 1;
        ADCU_text.fg_color = state_color;
    }
    else if(adcu_status == 1)
    {   // 주황 warning
        state_color.r = 1;
        state_color.g = 0.5;
        state_color.b = 0;
        state_color.a = 1;
        ADCU_text.fg_color = state_color;
    }
    else
    {   // 빨강 error
        state_color.r = 1;
        state_color.g = 0;
        state_color.b = 0;
        state_color.a = 1;
        ADCU_text.fg_color = state_color;
    }


    state_color.r = 0.4;
    state_color.g = 0.4;
    state_color.b = 0.4;
    state_color.a = 0.5;
    ADCU_text.bg_color = state_color;
    adcu_pub.publish(ADCU_text);

    katech_diag_msg.adcu_status = adcu_status;
}

void STAT_DISPLAY::ADCU_AliveCnt_Check(uint8_t current_cnt)
{
    //ADCU AliveCount Check
    current_adcu_cnt = current_cnt;
    if(current_adcu_cnt == last_adcu_cnt)
    {
        unchanged_adcu_cnt++;
    }
    else
    {
        unchanged_adcu_cnt = 0;
        last_adcu_cnt = current_adcu_cnt;
    }

    if(unchanged_adcu_cnt > 2)
    {
        adcu_status = 2;
    }
    else
    {
        adcu_status = 0;
    }
}

void STAT_DISPLAY::LIDAR_Text_Gen()
{
    ros::Time now = ros::Time::now();
    
    LIDAR_text.text = "LIDAR";

    std_msgs::ColorRGBA state_color;

    int32_t width = 200;
    int32_t height = 30;

    LIDAR_text.action = LIDAR_text.ADD;
    LIDAR_text.font = "DejaVu Sans Mono";
    LIDAR_text.text_size = 20;
    LIDAR_text.width = width;
    LIDAR_text.height = height;
    LIDAR_text.left = 20;
    LIDAR_text.top = 50+30;

    LIDAR_AliveCnt_Check(lidar_msg.LIDAR_AliveCount);

    if(lidar_status == 0)
    {
        if(lidar_msg.LIDAR_Center_StatCode == 1) lidar_status = 1;
        else if(lidar_msg.LIDAR_Right_StatCode == 1) lidar_status = 1;
        else if(lidar_msg.LIDAR_Right_StatCode == 1) lidar_status = 1;
    }

    if(lidar_status == 0)
    {   //흰색 정상
        state_color.r = 0;
        state_color.g = 0.8;
        state_color.b = 0;
        state_color.a = 1;
        LIDAR_text.fg_color = state_color;
    }
    else if(lidar_status == 1)
    {   // 주황 warning
        state_color.r = 1;
        state_color.g = 0.5;
        state_color.b = 0;
        state_color.a = 1;
        LIDAR_text.fg_color = state_color;
    }
    else
    {   // 빨강 error
        state_color.r = 1;
        state_color.g = 0;
        state_color.b = 0;
        state_color.a = 1;
        LIDAR_text.fg_color = state_color;
    }


    state_color.r = 0.4;
    state_color.g = 0.4;
    state_color.b = 0.4;
    state_color.a = 0.5;
    LIDAR_text.bg_color = state_color;
    lidar_pub.publish(LIDAR_text);

    katech_diag_msg.lidar_status = lidar_status;
}

void STAT_DISPLAY::LIDAR_AliveCnt_Check(uint8_t current_cnt)
{
    //ADCU AliveCount Check
    current_lidar_cnt = current_cnt;
    if(current_lidar_cnt == last_lidar_cnt)
    {
        unchanged_lidar_cnt++;
    }
    else
    {
        unchanged_lidar_cnt = 0;
        last_lidar_cnt = current_lidar_cnt;
    }

    if(unchanged_lidar_cnt > 5)
    {
        lidar_status = 2;
    }
    else
    {
        lidar_status = 0;
    }
}

void STAT_DISPLAY::RADAR_Text_Gen()
{
    ros::Time now = ros::Time::now();
    
    RADAR_text.text = "Radar";

    std_msgs::ColorRGBA state_color;

    int32_t width = 200;
    int32_t height = 30;

    RADAR_text.action = RADAR_text.ADD;
    RADAR_text.font = "DejaVu Sans Mono";
    RADAR_text.text_size = 20;
    RADAR_text.width = width;
    RADAR_text.height = height;
    RADAR_text.left = 20;
    RADAR_text.top = 50+30+30;

    RADAR_AliveCnt_Check(radar_msg.RADAR_AliveCount);

    if(radar_status == 0)
    {   //흰색 정상
        state_color.r = 0;
        state_color.g = 0.8;
        state_color.b = 0;
        state_color.a = 1;
        RADAR_text.fg_color = state_color;
    }
    else if(radar_status == 1)
    {   // 주황 warning
        state_color.r = 1;
        state_color.g = 0.5;
        state_color.b = 0;
        state_color.a = 1;
        RADAR_text.fg_color = state_color;
    }
    else
    {   // 빨강 error
        state_color.r = 1;
        state_color.g = 0;
        state_color.b = 0;
        state_color.a = 1;
        RADAR_text.fg_color = state_color;
    }


    state_color.r = 0.4;
    state_color.g = 0.4;
    state_color.b = 0.4;
    state_color.a = 0.5;
    RADAR_text.bg_color = state_color;
    radar_pub.publish(RADAR_text);

    katech_diag_msg.radar_status = radar_status;
}

void STAT_DISPLAY::RADAR_AliveCnt_Check(uint8_t current_cnt)
{
    //ADCU AliveCount Check
    current_radar_cnt = current_cnt;
    if(current_radar_cnt == last_radar_cnt)
    {
        unchanged_radar_cnt++;
    }
    else
    {
        unchanged_radar_cnt = 0;
        last_radar_cnt = current_radar_cnt;
    }

    if(unchanged_radar_cnt > 2)
    {
        radar_status = 2;
    }
    else
    {
        radar_status = 0;
    }
}

void STAT_DISPLAY::V2X_Text_Gen()
{
    ros::Time now = ros::Time::now();
    
    V2X_text.text = "V2X";

    std_msgs::ColorRGBA state_color;

    int32_t width = 200;
    int32_t height = 30;

    V2X_text.action = V2X_text.ADD;
    V2X_text.font = "DejaVu Sans Mono";
    V2X_text.text_size = 20;
    V2X_text.width = width;
    V2X_text.height = height;
    V2X_text.left = 20;
    V2X_text.top = 50+30+30+30;

    V2X_AliveCnt_Check(v2x_msg.V2X_AliveCount);
    
    if((v2x_status == 0) || (v2x_msg.V2X_StatCode == 0))
    {   //흰색 정상
        state_color.r = 0;
        state_color.g = 0.8;
        state_color.b = 0;
        state_color.a = 1;
        V2X_text.fg_color = state_color;
    }
    else if((v2x_status == 1) || (v2x_msg.V2X_StatCode == 1))
    {   // 주황 warning
        state_color.r = 1;
        state_color.g = 0.5;
        state_color.b = 0;
        state_color.a = 1;
        V2X_text.fg_color = state_color;
    }
    else
    {   // 빨강 error
        state_color.r = 1;
        state_color.g = 0;
        state_color.b = 0;
        state_color.a = 1;
        V2X_text.fg_color = state_color;
    }


    state_color.r = 0.4;
    state_color.g = 0.4;
    state_color.b = 0.4;
    state_color.a = 0.5;
    V2X_text.bg_color = state_color;
    v2x_pub.publish(V2X_text);

    katech_diag_msg.v2x_status = v2x_status;
}

void STAT_DISPLAY::V2X_AliveCnt_Check(uint8_t current_cnt)
{
    //ADCU AliveCount Check
    current_v2x_cnt = current_cnt;
    if(current_v2x_cnt == last_v2x_cnt)
    {
        unchanged_v2x_cnt++;
    }
    else
    {
        unchanged_v2x_cnt = 0;
        last_v2x_cnt = current_v2x_cnt;
    }

    if(unchanged_v2x_cnt > 2)
    {
        v2x_status = 2;
    }
    else
    {
        v2x_status = 0;
    }
}

void STAT_DISPLAY::HMI_Text_Gen()
{
    ros::Time now = ros::Time::now();
    
    HMI_text.text = "HMI";

    std_msgs::ColorRGBA state_color;

    int32_t width = 200;
    int32_t height = 30;

    HMI_text.action = HMI_text.ADD;
    HMI_text.font = "DejaVu Sans Mono";
    HMI_text.text_size = 20;
    HMI_text.width = width;
    HMI_text.height = height;
    HMI_text.left = 20;
    HMI_text.top = 50+30+30+30+30;

    HMI_AliveCnt_Check(hmi_msg.HMI_AliveCount);

    if(hmi_status == 0)
    {   //흰색 정상
        state_color.r = 0;
        state_color.g = 0.8;
        state_color.b = 0;
        state_color.a = 1;
        HMI_text.fg_color = state_color;
    }
    else if(hmi_status == 1)
    {   // 주황 warning
        state_color.r = 1;
        state_color.g = 0.5;
        state_color.b = 0;
        state_color.a = 1;
        HMI_text.fg_color = state_color;
    }
    else
    {   // 빨강 error
        state_color.r = 1;
        state_color.g = 0;
        state_color.b = 0;
        state_color.a = 1;
        HMI_text.fg_color = state_color;
    }


    state_color.r = 0.4;
    state_color.g = 0.4;
    state_color.b = 0.4;
    state_color.a = 0.5;
    HMI_text.bg_color = state_color;
    hmi_pub.publish(HMI_text);

    katech_diag_msg.hmi_status = hmi_status;
}

void STAT_DISPLAY::HMI_AliveCnt_Check(uint8_t current_cnt)
{
    //ADCU AliveCount Check
    current_hmi_cnt = current_cnt;
    if(current_hmi_cnt == last_hmi_cnt)
    {
        unchanged_hmi_cnt++;
    }
    else
    {
        unchanged_hmi_cnt = 0;
        last_hmi_cnt = current_hmi_cnt;
    }

    if(unchanged_hmi_cnt > 2)
    {
        hmi_status = 2;
    }
    else
    {
        hmi_status = 0;
    }
}

void STAT_DISPLAY::VCU_Text_Gen()
{
    ros::Time now = ros::Time::now();
    
    VCU_text.text = "VCU";

    std_msgs::ColorRGBA state_color;

    int32_t width = 200;
    int32_t height = 30;

    VCU_text.action = VCU_text.ADD;
    VCU_text.font = "DejaVu Sans Mono";
    VCU_text.text_size = 20;
    VCU_text.width = width;
    VCU_text.height = height;
    VCU_text.left = 20;
    VCU_text.top = 50+30+30+30+30+30;

    VCU_AliveCnt_Check(vcu_msg.VCU_AliveCount);

    if(vcu_status == 0)
    {   //흰색 정상
        state_color.r = 0;
        state_color.g = 0.8;
        state_color.b = 0;
        state_color.a = 1;
        VCU_text.fg_color = state_color;
    }
    else if(vcu_status == 1)
    {   // 주황 warning
        state_color.r = 1;
        state_color.g = 0.5;
        state_color.b = 0;
        state_color.a = 1;
        VCU_text.fg_color = state_color;
    }
    else
    {   // 빨강 error
        state_color.r = 1;
        state_color.g = 0;
        state_color.b = 0;
        state_color.a = 1;
        VCU_text.fg_color = state_color;
    }


    state_color.r = 0.4;
    state_color.g = 0.4;
    state_color.b = 0.4;
    state_color.a = 0.5;
    VCU_text.bg_color = state_color;
    vcu_pub.publish(VCU_text);

    katech_diag_msg.vcu_status = vcu_status;
}

void STAT_DISPLAY::VCU_AliveCnt_Check(uint8_t current_cnt)
{
    //ADCU AliveCount Check
    current_vcu_cnt = current_cnt;
    if(current_vcu_cnt == last_vcu_cnt)
    {
        unchanged_vcu_cnt++;
    }
    else
    {
        unchanged_vcu_cnt = 0;
        last_vcu_cnt = current_vcu_cnt;
    }

    if(unchanged_vcu_cnt > 2)
    {
        vcu_status = 2;
    }
    else
    {
        vcu_status = 0;
    }
}

void STAT_DISPLAY::CAM_Text_Gen()
{
    ros::Time now = ros::Time::now();

    CAM_text.text = "CAM";

    std_msgs::ColorRGBA state_color;

    int32_t width = 200;
    int32_t height = 30;

    CAM_text.action = CAM_text.ADD;
    CAM_text.font = "DejaVu Sans Mono";
    CAM_text.text_size = 20;
    CAM_text.width = width;
    CAM_text.height = height;
    CAM_text.left = 20;
    CAM_text.top = 50+30+30+30+30+30+30;

    CAM_AliveCnt_Check(cam_msg.CAM_AliveCount);
    cam_status = 0;
    if(cam_status == 0)
    {   //흰색 정상
        state_color.r = 0;
        state_color.g = 0.8;
        state_color.b = 0;
        state_color.a = 1;
        CAM_text.fg_color = state_color;
    }
    else if(cam_status == 1)
    {   // 주황 warning
        state_color.r = 1;
        state_color.g = 0.5;
        state_color.b = 0;
        state_color.a = 1;
        CAM_text.fg_color = state_color;
    }
    else
    {   // 빨강 error
        state_color.r = 1;
        state_color.g = 0;
        state_color.b = 0;
        state_color.a = 1;
        CAM_text.fg_color = state_color;
    }


    state_color.r = 0.4;
    state_color.g = 0.4;
    state_color.b = 0.4;
    state_color.a = 0.5;
    CAM_text.bg_color = state_color;
    cam_pub.publish(CAM_text);

    katech_diag_msg.cam_status = cam_status;

}
void STAT_DISPLAY::CAM_AliveCnt_Check(uint8_t current_cnt)
{
    //GPS AliveCount Check
    current_cam_cnt = current_cnt;
    if(current_cam_cnt == last_cam_cnt)
    {
        unchanged_cam_cnt++;
    }
    else
    {
        unchanged_cam_cnt = 0;
        last_cam_cnt = current_cam_cnt;
    }

    if(unchanged_cam_cnt > 10)
    {
        cam_status = 2;
    }
    else
    {
        cam_status = 0;
    }
}

void STAT_DISPLAY::IPC_Text_Gen()
{
    ros::Time now = ros::Time::now();

    IPC_text.text = "IPC";

    std_msgs::ColorRGBA state_color;

    int32_t width = 200;
    int32_t height = 30;

    IPC_text.action = IPC_text.ADD;
    IPC_text.font = "DejaVu Sans Mono";
    IPC_text.text_size = 20;
    IPC_text.width = width;
    IPC_text.height = height;
    IPC_text.left = 20;
    IPC_text.top = 50+30+30+30+30+30+30+30;

    IPC_AliveCnt_Check(ipc_msg.IPC_AliveCount);

    if(ipc_status == 0)
    {   //흰색 정상
        state_color.r = 0;
        state_color.g = 0.8;
        state_color.b = 0;
        state_color.a = 1;
        IPC_text.fg_color = state_color;
    }
    else if(ipc_status == 1)
    {   // 주황 warning
        state_color.r = 1;
        state_color.g = 0.5;
        state_color.b = 0;
        state_color.a = 1;
        IPC_text.fg_color = state_color;
    }
    else
    {   // 빨강 error
        state_color.r = 1;
        state_color.g = 0;
        state_color.b = 0;
        state_color.a = 1;
        IPC_text.fg_color = state_color;
    }


    state_color.r = 0.4;
    state_color.g = 0.4;
    state_color.b = 0.4;
    state_color.a = 0.5;
    IPC_text.bg_color = state_color;
    ipc_pub.publish(IPC_text);

    katech_diag_msg.ipc_status = ipc_status;

}
void STAT_DISPLAY::IPC_AliveCnt_Check(uint8_t current_cnt)
{
    //GPS AliveCount Check
    current_ipc_cnt = current_cnt;
    if(current_ipc_cnt == last_ipc_cnt)
    {
        unchanged_ipc_cnt++;
    }
    else
    {
        unchanged_ipc_cnt = 0;
        last_ipc_cnt = current_ipc_cnt;
    }

    if(unchanged_ipc_cnt > 4)
    {
        ipc_status = 2;
    }
    else
    {
        ipc_status = 0;
    }
}

void STAT_DISPLAY::system_status_check()
{
    std::vector<std::pair<std::string, int>> statuses = {
        {"GPS", gps_status},
        {"ADCU", adcu_status},
        {"LiDAR", lidar_status},
        {"Radar", radar_status},
        {"V2X", v2x_status},
        {"HMI", hmi_status},
        {"CAM", cam_status},        
        {"VCU", vcu_status},
        {"IPC", ipc_status}
    };

    int abnormal_count = 0;
    std::string abnormal_sensor = "";
    std::ostringstream oss;

    for (const auto& s : statuses)
    {
        if (s.second != 0)
        {
            abnormal_count++;
            abnormal_sensor = s.first;  // 가장 최근 감지된 비정상 센서
        }
    }

    if (abnormal_count == 1)
    {
        oss << "⚠️  " << abnormal_sensor << " 센서 고장 ";
        std::string str = oss.str();

        this->POPUP_Text_Gen(str);
        this->sound_play(abnormal_sensor);

    }
    else if (abnormal_count >= 2)
    {
        oss << "🚨 시스템 고장 (" << abnormal_count << "개 시스템 오류)";
        std::string str = oss.str();

        this->POPUP_Text_Gen(str);
        this->sound_play("ADS");
    }
    else if (local_msg.Road_State == 1)
    {
        oss << "전방 ODD 이탈 경고";
        std::string str = oss.str();

        this->sound_play("ODD");
        this->POPUP_Text_Gen(str);
    }
    else if (lo_chassis_msg.AEB_flag == 1)
    {
        oss << "🚨 전방 추돌 경고 🚨";
        std::string str = oss.str();

        this->sound_play("AEB");
        this->POPUP_Text_Gen(str);
    }
    else
    {
        // std::cout << "✅ 모든 센서 정상" << std::endl;
        this->POPUP_Text_Clear();
    }
}

void STAT_DISPLAY::sound_play(const std::string& sensor_name)
{
    static std::string base_path = "/home/ads/mcar_v13/src/visualization/stat_display/sound_files/";
    std::string path;
    static uint8_t play_count = 0;

    sound_play::SoundRequest sound_msg;

    sound_msg.sound = sound_play::SoundRequest::PLAY_FILE;  // 6
    sound_msg.command = sound_play::SoundRequest::PLAY_ONCE;  // 1
    sound_msg.volume = 1.0;
    
    sound_msg.arg2 = ""; // 음성 TTS일 경우 사용

    if (sensor_name == "GPS") path = base_path + "gps_warning.mp3";
    else if (sensor_name == "LiDAR") path = base_path + "lidar_warning.mp3";
    else if (sensor_name == "Radar") path = base_path + "radar_warning.mp3";
    else if (sensor_name == "V2X") path = base_path + "v2x_warning.mp3";
    else if (sensor_name == "ADCU") path = base_path + "adcu_warning.mp3";
    else if (sensor_name == "HMI") path = base_path + "hmi_warning.mp3";
    else if (sensor_name == "VCU") path = base_path + "vcu_warning.mp3";
    else if (sensor_name == "CAM") path = base_path + "cam_warning.mp3";
    else if (sensor_name == "ADS") path = base_path + "ad_system_warning.mp3";
    else if (sensor_name == "ODD") path = base_path + "odd_warning.mp3";
    else if (sensor_name == "IPC") path = base_path + "percept_warning.mp3";
    else if (sensor_name == "AEB") path = base_path + "aeb_warning.mp3";
    else path = base_path + "ad_system_warning.mp3";  // fallback

    sound_msg.arg = path;
    sound_msg.arg2 = "";

    if(lo_chassis_msg.vcu_EPS_Status == 2)
    {
        if(local_msg.Road_State == 1)
        {
            if(play_count >= 4)
            {
                sound_pub.publish(sound_msg);
                play_count = 0;
            }
            else
            {
                play_count++;
            }
        }
        else
        {
            if(play_count >= 2)
            {
                sound_pub.publish(sound_msg);
                play_count = 0;
            }
            else
            {
                play_count++;
            }
        }
    }
    
}

void STAT_DISPLAY::Local_Text_Gen()
{
    ros::Time now = ros::Time::now();
    
    LOCAL_text.text = "Curr LANE: " + std::to_string(local_msg.LINK_ID) +
                    "\nGPSRTK: " + std::to_string(cpt7_msg.GPSRTK_StatCode);

    std_msgs::ColorRGBA state_color;

    int32_t width = 400;
    int32_t height = 80;

    LOCAL_text.action = LOCAL_text.ADD;
    LOCAL_text.font = "DejaVu Sans Mono";
    LOCAL_text.text_size = 20;
    LOCAL_text.width = width;
    LOCAL_text.height = height;
    LOCAL_text.left = 500;
    LOCAL_text.top = 20;

    state_color.r = 0;
    state_color.g = 0;
    state_color.b = 0;
    state_color.a = 1;
    LOCAL_text.fg_color = state_color;
    
    state_color.r = 0.4;
    state_color.g = 0.4;
    state_color.b = 0.4;
    state_color.a = 0.5;
    LOCAL_text.bg_color = state_color;

    local_text_pub.publish(LOCAL_text);
}

void STAT_DISPLAY::MODE_Text_Gen()
{
    // vcu_EPS_Status 값에 따라 텍스트 결정
    if(lo_chassis_msg.vcu_EPS_Status == 2)
    {
        MANUAL_text.text = "AUTO";
    }
    else  // 0 또는 1
    {
        MANUAL_text.text = "MANUAL";
    }

    std_msgs::ColorRGBA state_color;

    int32_t width = 300;
    int32_t height = 50;

    MANUAL_text.action = MANUAL_text.ADD;
    MANUAL_text.font = "DejaVu Sans Mono";
    MANUAL_text.text_size = 30;
    MANUAL_text.width = width;
    MANUAL_text.height = height;
    
    // 중앙 하단 위치 설정 (1920x1080 기준)
    MANUAL_text.left = 550;//(1920 - width) / 2;  // 중앙 정렬
    MANUAL_text.top = 550;  // 하단 (화면 해상도에 맞게 조정 필요)

    // 파란색 텍스트
    state_color.r = 0;
    state_color.g = 0;
    state_color.b = 1;
    state_color.a = 1;
    MANUAL_text.fg_color = state_color;

    // 완전 투명 배경
    state_color.r = 0.2;
    state_color.g = 0.2;
    state_color.b = 0.2;
    state_color.a = 0.5;
    MANUAL_text.bg_color = state_color;
    
    mode_pub.publish(MANUAL_text);
}

void STAT_DISPLAY::TRAFFIC_LIGHT_Text_Gen()
{
    // 신호등이 없으면 표시하지 않음
    if(intersectionid == 0)
    {
        TRAFFIC_LIGHT_text.text = "N/A";

        std_msgs::ColorRGBA state_color;

        int32_t width = 150;
        int32_t height = 80;

        TRAFFIC_LIGHT_text.action = TRAFFIC_LIGHT_text.ADD;
        TRAFFIC_LIGHT_text.font = "DejaVu Sans Mono";
        TRAFFIC_LIGHT_text.text_size = 50;  // 큰 글씨로
        TRAFFIC_LIGHT_text.width = width;
        TRAFFIC_LIGHT_text.height = height;
        TRAFFIC_LIGHT_text.left = 1100;  // 화면 오른쪽 상단
        TRAFFIC_LIGHT_text.top = 50;

        state_color.r = 1;
        state_color.g = 0;
        state_color.b = 0;
        state_color.a = 1;
    
        TRAFFIC_LIGHT_text.fg_color = state_color;

        traffic_light_pub.publish(TRAFFIC_LIGHT_text);
        return;
    }

    // 초 단위로 변환 (100 -> 10초)
    int seconds = traffic_light_time / 10;
    
    TRAFFIC_LIGHT_text.text = std::to_string(seconds) + "s";

    std_msgs::ColorRGBA state_color;

    int32_t width = 150;
    int32_t height = 80;

    TRAFFIC_LIGHT_text.action = TRAFFIC_LIGHT_text.ADD;
    TRAFFIC_LIGHT_text.font = "DejaVu Sans Mono";
    TRAFFIC_LIGHT_text.text_size = 50;  // 큰 글씨로
    TRAFFIC_LIGHT_text.width = width;
    TRAFFIC_LIGHT_text.height = height;
    TRAFFIC_LIGHT_text.left = 1100;  // 화면 오른쪽 상단
    TRAFFIC_LIGHT_text.top = 50;

    // 신호등 색상에 따라 색 설정
    if(traffic_light_color == 1)  // 초록
    {
        state_color.r = 0;
        state_color.g = 1;
        state_color.b = 0;
        state_color.a = 1;
    }
    else if(traffic_light_color == 2)  // 주황
    {
        state_color.r = 1;
        state_color.g = 0.6;
        state_color.b = 0;
        state_color.a = 1;
    }
    else if(traffic_light_color == 3)  // 빨강
    {
        state_color.r = 1;
        state_color.g = 0;
        state_color.b = 0;
        state_color.a = 1;
    }
    
    TRAFFIC_LIGHT_text.fg_color = state_color;
    
    traffic_light_pub.publish(TRAFFIC_LIGHT_text);
}

void STAT_DISPLAY::ODD_Text_Gen()
{
    ros::Time now = ros::Time::now();

    ODD_text.text = "ODD";

    std_msgs::ColorRGBA state_color;

    int32_t width = 200;
    int32_t height = 30;

    ODD_text.action = ODD_text.ADD;
    ODD_text.font = "DejaVu Sans Mono";
    ODD_text.text_size = 20;
    ODD_text.width = width;
    ODD_text.height = height;
    ODD_text.left = 20;
    ODD_text.top = 50+30+30+30+30+30+30+30+30;

    if(local_msg.Road_State == 0)
    {   //흰색 정상
        state_color.r = 0;
        state_color.g = 0.8;
        state_color.b = 0;
        state_color.a = 1;
        ODD_text.fg_color = state_color;
    }
    else if(local_msg.Road_State == 1)
    {   // 주황 warning
        state_color.r = 1;
        state_color.g = 0.5;
        state_color.b = 0;
        state_color.a = 1;
        ODD_text.fg_color = state_color;
    }
    else
    {   // 빨강 error
        state_color.r = 1;
        state_color.g = 0;
        state_color.b = 0;
        state_color.a = 1;
        ODD_text.fg_color = state_color;
    }


    state_color.r = 0.4;
    state_color.g = 0.4;
    state_color.b = 0.4;
    state_color.a = 0.5;
    ODD_text.bg_color = state_color;
    odd_pub.publish(ODD_text);


}

void STAT_DISPLAY::SPEED_LIMIT_Text_Gen()
{
    ros::Time now = ros::Time::now();

    SPEED_LIMIT_text.text = std::to_string(local_msg.Speed_Limit);;

    std_msgs::ColorRGBA state_color;

    int32_t width = 200;
    int32_t height = 60;

    SPEED_LIMIT_text.action = SPEED_LIMIT_text.ADD;
    SPEED_LIMIT_text.font = "DejaVu Sans Mono";
    SPEED_LIMIT_text.text_size = 40;
    SPEED_LIMIT_text.width = width;
    SPEED_LIMIT_text.height = height;
    SPEED_LIMIT_text.left = 1100;
    SPEED_LIMIT_text.top = 120;

    state_color.r = 1;
    state_color.g = 0;
    state_color.b = 0;
    state_color.a = 1;
    SPEED_LIMIT_text.fg_color = state_color;
    
    // state_color.r = 0.4;
    // state_color.g = 0.4;
    // state_color.b = 0.4;
    // state_color.a = 0.5;
    // SPEED_LIMIT_text.bg_color = state_color;
    speed_limit_pub.publish(SPEED_LIMIT_text);


}
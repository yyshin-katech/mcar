#ifndef _STAT_DISPLAY_H_
#define _STAT_DISPLAY_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <string.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>

#include <jsk_rviz_plugins/OverlayText.h>
#include <jsk_rviz_plugins/OverlayMenu.h>

#include <katech_diagnostic_msgs/cpt7_gps_diagnostic_msg.h>
#include <katech_diagnostic_msgs/k_adcu_diagnostic_msg.h>
#include <katech_diagnostic_msgs/lidar_diagnostic_msg.h>
#include <katech_diagnostic_msgs/radar_diagnostic_msg.h>
#include <katech_diagnostic_msgs/v2x_diagnostic_msg.h>
#include <katech_diagnostic_msgs/hmi_diagnostic_msg.h>
#include <katech_diagnostic_msgs/vcu_diagnostic_msg.h>
#include <katech_diagnostic_msgs/cam_diagnostic_msg.h>
#include <katech_diagnostic_msgs/ipc_diagnostic_msg.h>
#include <katech_diagnostic_msgs/katech_diagnostic_msg.h>

#include <mmc_msgs/to_control_team_from_local_msg.h>
#include <mmc_msgs/chassis_msg.h>

#include <v2x_msgs/intersection_msg.h>
#include <v2x_msgs/intersection_array_msg.h>

#include <sound_play/SoundRequest.h>

#include <map>

#define GPS_ID 1

class STAT_DISPLAY{

    public:
        STAT_DISPLAY();
        ~STAT_DISPLAY();

    private:
        ros::NodeHandle nh;

        ros::Subscriber gps_sub;
        ros::Subscriber adcu_sub;
        ros::Subscriber lidar_sub;
        ros::Subscriber radar_sub;
        ros::Subscriber v2x_sub;
        ros::Subscriber hmi_sub;
        ros::Subscriber vcu_sub;
        ros::Subscriber cam_sub;
        ros::Subscriber local_sub;
        ros::Subscriber ipc_sub;

        ros::Subscriber chassis_sub_node;

        ros::Publisher popup_pub;
        ros::Publisher gps_pub;
        ros::Publisher adcu_pub;
        ros::Publisher lidar_pub;
        ros::Publisher radar_pub;
        ros::Publisher v2x_pub;
        ros::Publisher hmi_pub;
        ros::Publisher vcu_pub;
        ros::Publisher cam_pub;
        ros::Publisher ipc_pub;
        ros::Publisher local_text_pub;

        ros::Publisher mode_pub;

        ros::Publisher katech_diag_pub;

        ros::Publisher sound_pub;


        // Publisher 추가
        ros::Publisher traffic_light_pub;

        // Subscriber 추가
        ros::Subscriber traffic_light_sub;

        ros::Timer timer_;

        uint8_t current_gps_cnt, last_gps_cnt, unchanged_gps_cnt;
        uint8_t gps_status;
        uint8_t current_adcu_cnt, last_adcu_cnt, unchanged_adcu_cnt;
        uint8_t adcu_status;
        uint8_t current_lidar_cnt, last_lidar_cnt, unchanged_lidar_cnt;
        uint8_t lidar_status;
        uint8_t current_radar_cnt, last_radar_cnt, unchanged_radar_cnt;
        uint8_t radar_status;
        uint8_t current_v2x_cnt, last_v2x_cnt, unchanged_v2x_cnt;
        uint8_t v2x_status;
        uint8_t current_hmi_cnt, last_hmi_cnt, unchanged_hmi_cnt;
        uint8_t hmi_status;
        uint8_t current_vcu_cnt, last_vcu_cnt, unchanged_vcu_cnt;
        uint8_t vcu_status;
        uint8_t current_cam_cnt, last_cam_cnt, unchanged_cam_cnt;
        uint8_t cam_status;
        uint8_t current_ipc_cnt, last_ipc_cnt, unchanged_ipc_cnt;
        uint8_t ipc_status;

        katech_diagnostic_msgs::cpt7_gps_diagnostic_msg cpt7_msg;
        katech_diagnostic_msgs::k_adcu_diagnostic_msg adcu_msg;
        katech_diagnostic_msgs::lidar_diagnostic_msg lidar_msg;
        katech_diagnostic_msgs::radar_diagnostic_msg radar_msg;
        katech_diagnostic_msgs::v2x_diagnostic_msg v2x_msg;
        katech_diagnostic_msgs::hmi_diagnostic_msg hmi_msg;
        katech_diagnostic_msgs::vcu_diagnostic_msg vcu_msg;
        katech_diagnostic_msgs::cam_diagnostic_msg cam_msg;
        katech_diagnostic_msgs::ipc_diagnostic_msg ipc_msg;
        
        katech_diagnostic_msgs::katech_diagnostic_msg katech_diag_msg;

        mmc_msgs::to_control_team_from_local_msg local_msg;
        mmc_msgs::chassis_msg lo_chassis_msg;

        // visualization_msgs::Marker GPS_text;

        jsk_rviz_plugins::OverlayText POPUP_text;
        jsk_rviz_plugins::OverlayText GPS_text;
        jsk_rviz_plugins::OverlayText ADCU_text;
        jsk_rviz_plugins::OverlayText LIDAR_text;
        jsk_rviz_plugins::OverlayText RADAR_text;
        jsk_rviz_plugins::OverlayText V2X_text;
        jsk_rviz_plugins::OverlayText HMI_text;
        jsk_rviz_plugins::OverlayText VCU_text;
        jsk_rviz_plugins::OverlayText CAM_text;
        jsk_rviz_plugins::OverlayText IPC_text;
        jsk_rviz_plugins::OverlayText LOCAL_text;
        jsk_rviz_plugins::OverlayText MANUAL_text;

        void diagnostic_gps_callback(const katech_diagnostic_msgs::cpt7_gps_diagnostic_msg::ConstPtr& msg);
        void diagnostic_adcu_callback(const katech_diagnostic_msgs::k_adcu_diagnostic_msg::ConstPtr& msg);
        void diagnostic_lidar_callback(const katech_diagnostic_msgs::lidar_diagnostic_msg::ConstPtr& msg);
        void diagnostic_radar_callback(const katech_diagnostic_msgs::radar_diagnostic_msg::ConstPtr& msg);
        void diagnostic_v2x_callback(const katech_diagnostic_msgs::v2x_diagnostic_msg::ConstPtr& msg);
        void diagnostic_hmi_callback(const katech_diagnostic_msgs::hmi_diagnostic_msg::ConstPtr& msg);
        void diagnostic_vcu_callback(const katech_diagnostic_msgs::vcu_diagnostic_msg::ConstPtr& msg);
        void diagnostic_cam_callback(const katech_diagnostic_msgs::cam_diagnostic_msg::ConstPtr& msg);
        void diagnostic_ipc_callback(const katech_diagnostic_msgs::ipc_diagnostic_msg::ConstPtr& msg);
        void local_callback(const mmc_msgs::to_control_team_from_local_msg::ConstPtr& msg);
        void chassis_callback_func(const mmc_msgs::chassis_msg::ConstPtr& msg);
        
        void timerCallback(const ros::TimerEvent&);

        void POPUP_Text_Gen(const std::string& message);
        void POPUP_Text_Clear();

        void GPS_Text_Gen();
        void GPS_AliveCnt_Check(uint8_t current_cnt);

        void ADCU_Text_Gen();
        void ADCU_AliveCnt_Check(uint8_t current_cnt);

        void LIDAR_Text_Gen();
        void LIDAR_AliveCnt_Check(uint8_t current_cnt);

        void RADAR_Text_Gen();
        void RADAR_AliveCnt_Check(uint8_t current_cnt);        

        void V2X_Text_Gen();
        void V2X_AliveCnt_Check(uint8_t current_cnt);

        void HMI_Text_Gen();
        void HMI_AliveCnt_Check(uint8_t current_cnt);

        void VCU_Text_Gen();
        void VCU_AliveCnt_Check(uint8_t current_cnt);

        void CAM_Text_Gen();
        void CAM_AliveCnt_Check(uint8_t current_cnt);

        void IPC_Text_Gen();
        void IPC_AliveCnt_Check(uint8_t current_cnt);

        void system_status_check();

        void sound_play(const std::string& sensor_name);

        void Local_Text_Gen();

        void MODE_Text_Gen();

        // 신호등 메시지 변수 (메시지 타입은 실제 사용하는 것으로 변경)
        // 예: your_msgs::traffic_light_msg traffic_light_msg;
        // v2x_msgs::intersection_array_msg traffic_light_msg;      // 남은 시간 (0.1초 단위)
        // int traffic_light_color;     // 0: 없음, 1: 초록, 2: 주황, 3: 빨강
        int traffic_light_time;      // 남은 시간 (0.1초 단위)
        int traffic_light_color;     // 0: 없음, 1: 초록, 2: 주황, 3: 빨강
        // Callback 함수
        void traffic_light_callback(const v2x_msgs::intersection_array_msg::ConstPtr& msg);

        // Text 생성 함수
        void TRAFFIC_LIGHT_Text_Gen();

        // OverlayText 변수
        jsk_rviz_plugins::OverlayText TRAFFIC_LIGHT_text;
};

#endif
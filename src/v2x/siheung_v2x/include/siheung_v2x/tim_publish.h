#pragma once

#include <ros/ros.h>

#include "ffasn1-j2735-2026-KSR1600.h"

// J2735 TravelerInformation(TIM) 디코드 결과를 ROS 메시지로 발행하는 공유 로직.
// j2735_TIM_rx_node(독립 UDP 테스터)와 siheung_v2x_node(통합 OBU 수신) 양쪽에서 사용.
namespace tim_publish {

// 디코드된 TravelerInformation 을 받아 3개 토픽으로 발행한다.
//   tim_pub      : v2x_msgs/v2x_tim_total_msg
//   pedes_pub    : v2x_msgs/v2x_pedes_assist_msg
//   go_ahead_pub : v2x_msgs/v2x_tim_can_go_msg  (packetID 가 "RNST" 로 시작할 때만)
void publish(const j2735TravelerInformation* tim,
             ros::Publisher& tim_pub,
             ros::Publisher& pedes_pub,
             ros::Publisher& go_ahead_pub);

}  // namespace tim_publish

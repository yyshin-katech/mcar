#ifndef SDSM_DECODE_H
#define SDSM_DECODE_H

#include <ros/ros.h>
#include <cstdint>

// SDSM 디코딩 (ffasn1-j2735-2020 사용, 별도 번역 단위)
// isMsgFrame=1 전용: raw UPER로 직접 디코딩
bool decode_sdsm_uper(const uint8_t* data, size_t len, ros::Publisher& pub);

#endif

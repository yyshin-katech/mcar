// SDSM 디코더 - ffasn1-j2735-2020 라이브러리 사용
// KSR1600 헤더와 충돌하므로 별도 번역 단위로 분리

#include "ffasn1-j2735-2020.h"
#include "asn1defs.h"

#include <ros/ros.h>
#include <j3224_msgs/sdsm.h>
#include <sstream>
#include <iomanip>
#include <cstring>

static void publish_sdsm(j2735SensorDataSharingMessage* sdsm, ros::Publisher& pub)
{
    j3224_msgs::sdsm ros_msg;

    ros_msg.msgCnt = sdsm->msgCnt;

    for (size_t i = 0; i < 4 && i < sdsm->sourceID.len; ++i)
        ros_msg.sourceID[i] = sdsm->sourceID.buf[i];

    ros_msg.equipmentType = (uint8_t)sdsm->equipmentType;
    ros_msg.sDSMTimeStamp.year = sdsm->sDSMTimeStamp.year;
    ros_msg.sDSMTimeStamp.month = sdsm->sDSMTimeStamp.month;
    ros_msg.sDSMTimeStamp.day = sdsm->sDSMTimeStamp.day;
    ros_msg.sDSMTimeStamp.hour = sdsm->sDSMTimeStamp.hour;
    ros_msg.sDSMTimeStamp.minute = sdsm->sDSMTimeStamp.minute;
    ros_msg.sDSMTimeStamp.second = sdsm->sDSMTimeStamp.second;

    ros_msg.refPos.latitude = sdsm->refPos.lat;
    ros_msg.refPos.longitude = sdsm->refPos.Long;
    ros_msg.refPos.elevation = sdsm->refPos.elevation;

    ros_msg.refPosXYConf.semiMajor = sdsm->refPosXYConf.semiMajor;
    ros_msg.refPosXYConf.semiMinor = sdsm->refPosXYConf.semiMinor;
    ros_msg.refPosXYConf.orientation = sdsm->refPosXYConf.orientation;

    ros_msg.objects.clear();
    for (size_t i = 0; i < sdsm->objects.count; ++i)
    {
        j3224_msgs::DetectedObjectData obj_msg;
        auto& det = sdsm->objects.tab[i].detObjCommon;

        obj_msg.detObjCommon.objType = det.objType;
        obj_msg.detObjCommon.objTypeCfd = det.objTypeCfd;
        obj_msg.detObjCommon.objectID = det.objectID;
        obj_msg.detObjCommon.measurementTime = det.measurementTime;
        obj_msg.detObjCommon.timeConfidence = det.timeConfidence;

        obj_msg.detObjCommon.offsetX = det.pos.offsetX;
        obj_msg.detObjCommon.offsetY = det.pos.offsetY;
        obj_msg.detObjCommon.offsetZ = det.pos.offsetZ;

        obj_msg.detObjCommon.posConfidence = det.posConfidence.pos;
        obj_msg.detObjCommon.elevationConfidence = det.posConfidence.elevation;

        obj_msg.detObjCommon.speed = det.speed;
        obj_msg.detObjCommon.speedConfidence = det.speedConfidence;

        obj_msg.detObjCommon.heading = det.heading;
        obj_msg.detObjCommon.headingConfidence = det.headingConf;

        ros_msg.objects.push_back(obj_msg);
    }

    ROS_INFO("[SDSM] Decoded: msgCnt=%d, objects=%zu", sdsm->msgCnt, sdsm->objects.count);
    pub.publish(ros_msg);
}

// isMsgFrame=1: MessageFrame 아님, UPER 직접 디코딩
bool decode_sdsm_uper(const uint8_t* data, size_t len, ros::Publisher& pub)
{
    ASN1Error err;
    memset(&err, 0, sizeof(err));
    void* msg = nullptr;

    asn1_ssize_t ret = asn1_uper_decode(&msg, asn1_type_j2735SensorDataSharingMessage,
                                         data, len, &err);
    if (ret > 0 && msg)
    {
        j2735SensorDataSharingMessage* sdsm = (j2735SensorDataSharingMessage*)msg;
        publish_sdsm(sdsm, pub);
        asn1_free_value(asn1_type_j2735SensorDataSharingMessage, msg);
        return true;
    }

    ROS_DEBUG("[SDSM] UPER decode failed: %s", err.msg ? err.msg : "unknown");
    return false;
}


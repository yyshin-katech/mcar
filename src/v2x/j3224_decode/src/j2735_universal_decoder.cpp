#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>

#include "ffasn1-j2735-2020.h"
#include "asn1defs.h"
#include <j3224_msgs/sdsm.h>

#define UDP_PORT 9999
#define BUF_SIZE 2048

// J2735 메시지 ID 정의
enum J2735MessageId {
    MSG_ID_BSM  = 0x14,  // 20
    MSG_ID_MAP  = 0x12,  // 18
    MSG_ID_SPAT = 0x13,  // 19
    MSG_ID_TIM  = 0x1F,  // 31
    MSG_ID_PVD  = 0x0D,  // 13
    MSG_ID_RSA  = 0x1B,  // 27
    MSG_ID_PSM  = 0x20,  // 32
    MSG_ID_SDSM = 0x21,  // 33
};

struct ReceivedMsg {
    std::vector<uint8_t> data;
    size_t len;
};

class MessageQueue {
public:
    void push(const ReceivedMsg& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        queue_.push(msg);
        cv_.notify_one();
    }

    bool pop(ReceivedMsg& msg) {
        std::unique_lock<std::mutex> lock(mtx_);
        if(cv_.wait_for(lock, std::chrono::milliseconds(10), [&]{ return !queue_.empty();})) {
            msg = queue_.front();
            queue_.pop();
            return true;
        }
        return false;
    }

private:
    std::mutex mtx_;
    std::condition_variable cv_;
    std::queue<ReceivedMsg> queue_;
};

class J2735_DECODE {
public:
    const char* get_message_name(int msgId) {
        switch(msgId) {
            case MSG_ID_BSM:  return "BSM";
            case MSG_ID_MAP:  return "MAP";
            case MSG_ID_SPAT: return "SPAT";
            case MSG_ID_TIM:  return "TIM";
            case MSG_ID_PVD:  return "PVD";
            case MSG_ID_RSA:  return "RSA";
            case MSG_ID_PSM:  return "PSM";
            case MSG_ID_SDSM: return "SDSM";
            default:          return "Unknown";
        }
    }

    // SDSM 디코딩 및 ROS 메시지 발행
    void decode_sdsm_and_publish(j2735SensorDataSharingMessage* sdsm, ros::Publisher& pub) {
        j3224_msgs::sdsm ros_msg;

        std::stringstream id_ss;
        for (size_t i = 0; i < sdsm->sourceID.len; ++i) {
            id_ss << std::hex << std::setw(2) << std::setfill('0') << (int)sdsm->sourceID.buf[i];
        }
        
        ROS_INFO("[SDSM] Decoded");
        ROS_INFO("  msgCnt: %d", sdsm->msgCnt);
        ROS_INFO("  sourceID: 0x%s", id_ss.str().c_str());
        ROS_INFO("  refPos: (%d, %d)", sdsm->refPos.lat, sdsm->refPos.Long);
        ROS_INFO("  objects: %zu", sdsm->objects.count);

        // ROS 메시지 생성
        ros_msg.msgCnt = sdsm->msgCnt;

        for(size_t i = 0 ; i < 4 && i < sdsm->sourceID.len ; ++i) {
            ros_msg.sourceID[i] = sdsm->sourceID.buf[i];
        }

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
        for (size_t i = 0; i < sdsm->objects.count; ++i) {
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

        pub.publish(ros_msg);
    }

    // TIM 디코딩 및 출력
    void decode_tim_and_display(j2735TravelerInformation* tim) {
        ROS_INFO("[TIM] Decoded");
        ROS_INFO("  msgCnt: %d", tim->msgCnt);
        ROS_INFO("  dataFrames: %zu", tim->dataFrames.count);

        for (size_t i = 0; i < std::min(tim->dataFrames.count, size_t(3)); ++i) {
            auto& frame = tim->dataFrames.tab[i];
            ROS_INFO("    [Frame %zu] frameType: %d, regions: %zu", 
                     i, frame.frameType, frame.regions.count);
        }
    }

    // MessageFrame 디코딩 (BER)
    bool decode_messageframe(const uint8_t* data, size_t len, ros::Publisher& sdsm_pub) {
        ASN1Error err;
        memset(&err, 0, sizeof(err));
        void* mf_msg = nullptr;

        asn1_ssize_t ret = asn1_ber_decode(&mf_msg, asn1_type_j2735MessageFrame,
                                           data, len, &err);

        if (ret > 0 && mf_msg != nullptr) {
            j2735MessageFrame* mf = (j2735MessageFrame*)mf_msg;
            
            ROS_INFO("[MessageFrame] messageId: %d (0x%02X) - %s", 
                     mf->messageId, mf->messageId, get_message_name(mf->messageId));

            // messageId에 따라 처리
            switch(mf->messageId) {
                case MSG_ID_TIM: {
                    // TIM 처리
                    j2735TravelerInformation* tim = nullptr;
                    
                    if (mf->value.type != nullptr) {
                        // 이미 디코딩된 경우
                        tim = (j2735TravelerInformation*)mf->value.u.data;
                        if (tim != nullptr) {
                            ROS_INFO("Decoded");
                            decode_tim_and_display(tim);
                        }
                    } else if (mf->value.u.octet_string.len > 0) {
                        // raw octet_string - 재디코딩
                        ASN1Error err2;
                        void* tim_msg = nullptr;
                        
                        asn1_ber_decode(&tim_msg, asn1_type_j2735TravelerInformation,
                                       (uint8_t*)mf->value.u.octet_string.buf,
                                       mf->value.u.octet_string.len, &err2);
                        
                        if (tim_msg != nullptr) {
                            tim = (j2735TravelerInformation*)tim_msg;
                            decode_tim_and_display(tim);
                            asn1_free_value(asn1_type_j2735TravelerInformation, tim_msg);
                        }
                    }
                    break;
                }

                case MSG_ID_SDSM: {
                    // SDSM 처리
                    j2735SensorDataSharingMessage* sdsm = nullptr;
                    
                    if (mf->value.type != nullptr) {
                        sdsm = (j2735SensorDataSharingMessage*)mf->value.u.data;
                        if (sdsm != nullptr) {
                            decode_sdsm_and_publish(sdsm, sdsm_pub);
                        }
                    } else if (mf->value.u.octet_string.len > 0) {
                        ASN1Error err2;
                        void* sdsm_msg = nullptr;
                        
                        asn1_ber_decode(&sdsm_msg, asn1_type_j2735SensorDataSharingMessage,
                                       (uint8_t*)mf->value.u.octet_string.buf,
                                       mf->value.u.octet_string.len, &err2);
                        
                        if (sdsm_msg != nullptr) {
                            sdsm = (j2735SensorDataSharingMessage*)sdsm_msg;
                            decode_sdsm_and_publish(sdsm, sdsm_pub);
                            asn1_free_value(asn1_type_j2735SensorDataSharingMessage, sdsm_msg);
                        }
                    }
                    break;
                }

                case MSG_ID_BSM:
                case MSG_ID_MAP:
                case MSG_ID_SPAT:
                    ROS_INFO("  [%s] decoder not implemented yet", get_message_name(mf->messageId));
                    break;

                default:
                    ROS_WARN("  Unsupported message type: %d", mf->messageId);
                    break;
            }

            asn1_free_value(asn1_type_j2735MessageFrame, mf_msg);
            return true;
        } else {
            ROS_DEBUG("MessageFrame decode failed: %s", err.msg ? err.msg : "unknown");
            return false;
        }
    }

    // UPER SDSM 디코딩 (기존 방식)
    bool decode_uper_sdsm(const uint8_t* data, size_t len, ros::Publisher& sdsm_pub) {
        ASN1Error err;
        memset(&err, 0, sizeof(err));
        void* msg = nullptr;

        asn1_ssize_t ret = asn1_uper_decode(&msg, asn1_type_j2735SensorDataSharingMessage,
                                            data, len, &err);

        if (ret > 0 && msg != nullptr) {
            j2735SensorDataSharingMessage* sdsm = (j2735SensorDataSharingMessage*)msg;
            decode_sdsm_and_publish(sdsm, sdsm_pub);
            asn1_free_value(asn1_type_j2735SensorDataSharingMessage, msg);
            return true;
        } else {
            ROS_DEBUG("UPER SDSM decode failed: %s", err.msg ? err.msg : "unknown");
            return false;
        }
    }

    // 메인 디코딩 함수 - 자동 형식 감지
    void decode(const ReceivedMsg* rmsg, ros::Publisher& sdsm_pub) {
        const uint8_t* payload = rmsg->data.data();
        size_t payload_len = rmsg->len;

        if (payload_len == 0) {
            return;
        }

        ROS_DEBUG("Received %zu bytes, first byte: 0x%02X", payload_len, payload[0]);

        // 1. BER/DER 인코딩 감지 (MessageFrame)
        if (payload[0] == 0x30) {
            ROS_DEBUG("Detected BER/DER encoding (MessageFrame)");
            
            // MessageFrame 디코딩 시도
            if (decode_messageframe(payload, payload_len, sdsm_pub)) {
                return;  // 성공
            }

            // MessageFrame 실패 시 직접 TIM 디코딩 시도 (offset 11)
            if (payload_len > 11) {
                ROS_DEBUG("Trying direct TIM decode from offset 11");
                
                ASN1Error err;
                void* tim_msg = nullptr;
                
                asn1_ssize_t ret = asn1_ber_decode(&tim_msg, asn1_type_j2735TravelerInformation,
                                                   &payload[11], payload_len - 11, &err);
                
                if (ret > 0 && tim_msg != nullptr) {
                    j2735TravelerInformation* tim = (j2735TravelerInformation*)tim_msg;
                    decode_tim_and_display(tim);
                    asn1_free_value(asn1_type_j2735TravelerInformation, tim_msg);
                    return;
                }
            }

            ROS_WARN("Failed to decode BER data");
            return;
        }

        // 2. UPER 인코딩 감지 (기존 SDSM)
        // 첫 바이트가 0x00으로 시작하고 두 번째 바이트가 messageId
        if (payload[0] == 0x03 && payload_len > 10) {
            ROS_DEBUG("Detected UPER SDSM (starts with 0x00)");
            
            // payload[1]이 0x21 (SDSM)인지 확인
            // if (payload[1] == MSG_ID_SDSM) 
            {
                ROS_INFO("MessageId: 0x%02X - %s", payload[1], get_message_name(payload[1]));
                
                // 기존 방식: offset 10부터 디코딩
                if (decode_uper_sdsm(&payload[10], payload_len - 10, sdsm_pub)) {
                    return;
                }
            }
        }

        // 3. 다른 UPER 메시지 시도
        if (payload[0] <= 0x30) {  // UPER 메시지는 보통 작은 값으로 시작
            ROS_DEBUG("Trying UPER decode from start");
            
            if (decode_uper_sdsm(payload, payload_len, sdsm_pub)) {
                return;
            }
        }

        ROS_WARN("Unable to decode message (len=%zu, first_byte=0x%02X)", 
                 payload_len, payload[0]);
    }
};

int set_nonblocking(int sockfd) {
    int flags = fcntl(sockfd, F_GETFL, 0);
    return (flags < 0) ? -1 : fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
}

void udpReceiverThread(int sockfd, MessageQueue& queue) {
    uint8_t buffer[BUF_SIZE];
    struct sockaddr_in sender_addr;
    socklen_t addrlen = sizeof(sender_addr);

    while (ros::ok()) {
        ssize_t len = recvfrom(sockfd, buffer, BUF_SIZE, 0, 
                               (struct sockaddr*)&sender_addr, &addrlen);
        
        if (len > 0) {
            queue.push({std::vector<uint8_t>(buffer, buffer + len), (size_t)len});
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "j2735_universal_decoder");
    ros::NodeHandle nh;
    
    ROS_INFO("=== J2735 Universal Decoder Started ===");
    ROS_INFO("Supports:");
    ROS_INFO("  - SDSM (UPER encoding)");
    ROS_INFO("  - TIM/SDSM/BSM/MAP/SPAT/etc (BER encoding via MessageFrame)");
    
    ros::Publisher sdsm_pub = nh.advertise<j3224_msgs::sdsm>("/obu/sdsm", 10);

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        ROS_ERROR("Socket creation failed");
        return 1;
    }

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ROS_ERROR("Bind failed: %s", strerror(errno));
        close(sockfd);
        return 1;
    }

    if (set_nonblocking(sockfd) < 0) {
        ROS_ERROR("Failed to set non-blocking");
        close(sockfd);
        return 1;
    }

    MessageQueue queue;
    J2735_DECODE decoder;

    std::thread recv_thread(udpReceiverThread, sockfd, std::ref(queue));
    ros::Rate loop_rate(1000);

    ROS_INFO("Listening on UDP port %d", UDP_PORT);
    ROS_INFO("Ready to receive J2735 messages...\n");

    while (ros::ok()) {
        ReceivedMsg msg;
        if (queue.pop(msg)) {
            decoder.decode(&msg, sdsm_pub);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    recv_thread.join();
    close(sockfd);
    return 0;
}
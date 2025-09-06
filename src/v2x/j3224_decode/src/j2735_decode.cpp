#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/select.h>

#include "ffasn1-j2735-2020.h"
#include "asn1defs.h"

#include <j3224_msgs/sdsm.h>

#define OUB_IP_ADDR "192.168.1.174"
#define UDP_PORT 9999
#define BUF_SIZE 2048

struct ReceivedMsg{
    std::vector<uint8_t> data;
    size_t len;
};

class MessageQueue {
    public:
        void push(const ReceivedMsg& msg)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            queue_.push(msg);
            cv_.notify_one();
        }

        bool pop(ReceivedMsg& msg)
        {
            std::unique_lock<std::mutex> lock(mtx_);
            if(cv_.wait_for(lock, std::chrono::milliseconds(10), [&]{ return !queue_.empty();}))
            {
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

class J2735_DECODE{
    public:
        void decode(const ReceivedMsg* rmsg, ros::Publisher& pub)
        {
            const uint8_t* payload = rmsg->data.data();
            size_t payload_len = rmsg->len;

            // if(payload[0] == 0x03 && payload[1] == 0x81)
            // {
            //     payload += 4;
            //     payload_len -= 4;
            // }
            payload_len = rmsg->len - 3;
            uint8_t msgId = payload[1];
            ASN1Error err;
            void* msg = nullptr;

            const char* name = "Unknown";
            const ASN1CType* type = nullptr;

            switch (msgId)
            {
                case 0x21: name = "SDSM"; type = asn1_type_j2735SensorDataSharingMessage; break;
                case 0x0D: name = "PVD";  type = asn1_type_j2735ProbeVehicleData; break;
                case 0x13: name = "BSM";  type = asn1_type_j2735BasicSafetyMessage; break;
                case 0x1D: name = "SPAT"; type = asn1_type_j2735SPAT; break;
                case 0x1E: name = "TIM";  type = asn1_type_j2735TravelerInformation; break;
                case 0x1F: name = "RSA";  type = asn1_type_j2735RoadSideAlert; break;
                case 0x20: name = "MAP";  type = asn1_type_j2735MapData; break;
            }

            if(type)
            {
                asn1_ssize_t ret = asn1_uper_decode(&msg, type, &payload[3], payload_len, &err);
                // asn1_ssize_t ret = asn1_ber_decode(&msg, type, payload, payload_len, &err);
                if(ret > 0 && msg)
                {
                    j3224_msgs::sdsm ros_msg;

                    j2735SensorDataSharingMessage *sdsm = (j2735SensorDataSharingMessage *)msg;
                    
                    // std::stringstream id_ss;
                    // for (size_t i = 0; i < sdsm->sourceID.len; ++i) {
                    //     id_ss << std::hex << std::setw(2) << std::setfill('0') << (int)sdsm->sourceID.buf[i];

                    // }
                    // ROS_INFO("[decoder] Decoded OK");
                    // ROS_INFO("SDSM Decoded:");
                    // ROS_INFO("  msgCnt: %d", sdsm->msgCnt);
                    // ROS_INFO("  sourceID: 0x%s", id_ss.str().c_str());
                    // ROS_INFO("  year: %d", sdsm->sDSMTimeStamp.year);
                    // ROS_INFO("  mon: %d", sdsm->sDSMTimeStamp.month);
                    // ROS_INFO("  day: %d", sdsm->sDSMTimeStamp.day);
                    // ROS_INFO("  hour: %d", sdsm->sDSMTimeStamp.hour);
                    // ROS_INFO("  min: %d", sdsm->sDSMTimeStamp.minute);
                    // ROS_INFO("  sec: %d", sdsm->sDSMTimeStamp.second);    
                    // ROS_INFO("  refPos.lat: %d", sdsm->refPos.lat);
                    // ROS_INFO("  refPos.long: %d", sdsm->refPos.Long);
                    // ROS_INFO("  refPos.ele_option: %d", sdsm->refPos.elevation_option);
                    // ROS_INFO("  refPos.regional_option: %d", sdsm->refPos.regional_option);
                    // ROS_INFO("  refPos.regional.count: %ld", sdsm->refPos.regional.count);
                    // // 객체 수 확인 (필드 이름은 사용 중인 ffasn1 구조체에 따라 다름)
                    // if (sdsm->objects.count > 0) {
                    //     ROS_INFO("  objects.count: %ld", sdsm->objects.count);
                    //     auto &obj = sdsm->objects.tab[0].detObjCommon;
                    //     ROS_INFO("    measerTime: %d", obj.measurementTime);
                    //     ROS_INFO("    timeConfidence: %d", obj.timeConfidence);
                    //     ROS_INFO("    objectID: %d", obj.objectID);
                    //     ROS_INFO("    objType: %d", obj.objType);
                    //     ROS_INFO("    objTypeCfd: %d", obj.objTypeCfd);

                    //     ROS_INFO("    pos: %d   %d", obj.pos.offsetX, obj.pos.offsetY);
                    //     ROS_INFO("    posConfidence: %d", obj.posConfidence.pos);
                        
                    //     ROS_INFO("    speed: %d (0.02m/s unit)", obj.speed);
                    //     ROS_INFO("    speedConfidence: %d", obj.speedConfidence);
                        
                    //     ROS_INFO("    heading: %d (0.0125deg unit)", obj.heading);
                    //     ROS_INFO("    headingConf: %d", obj.headingConf);
                    // }

                    ros_msg.msgCnt = sdsm->msgCnt;

                    for(size_t i = 0 ; i < 4 && i < sdsm->sourceID.len ; ++i)
                    {
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

                        obj_msg.detObjCommon.speed   = det.speed;
                        obj_msg.detObjCommon.speedConfidence = det.speedConfidence;

                        obj_msg.detObjCommon.heading = det.heading;
                        obj_msg.detObjCommon.headingConfidence = det.headingConf;

                        ros_msg.objects.push_back(obj_msg);
                    }
                    
                    pub.publish(ros_msg);

                    free(msg);
                    // free(sdsm);
                }
                else
                {
                    ROS_WARN("[decoder] Failed to decode %s (msgId=0x%02X): %s", name, msgId, err.msg);
                }
            }
            else
            {
                ROS_WARN("[deocder] Unknown msgId= 0x%02X", msgId);
            }
        }

        uint32_t swap_uint32(uint32_t val) 
        {
            return ((val >> 24) & 0x000000FF) |
                ((val >> 8)  & 0x0000FF00) |
                ((val << 8)  & 0x00FF0000) |
                ((val << 24) & 0xFF000000);
        }
    
};

int set_nonblocking(int sockfd)
{
    int flags = fcntl(sockfd, F_GETFL, 0);
    return (flags < 0) ? -1 : fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
}

void udpReceiverThread(int sockfd, MessageQueue& queue) 
{
    uint8_t buffer[BUF_SIZE];
    struct sockaddr_in sender_addr;
    socklen_t addrlen = sizeof(sender_addr);

    while (ros::ok()) 
    {
        ssize_t len = recvfrom(sockfd, buffer, BUF_SIZE, 0, (struct sockaddr*)&sender_addr, &addrlen);
        if (len > 0) 
        {
            queue.push({std::vector<uint8_t>(buffer, buffer + len), (size_t)len});
        } else 
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "j2735_udp_decoder_mt");
    ros::NodeHandle nh;
    
    ros::Publisher sdsm_pub = nh.advertise<j3224_msgs::sdsm>("/obu/sdsm", 1);;

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) 
    {
        ROS_ERROR("Socket creation failed");
        return 1;
    }

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_PORT);
    inet_pton(AF_INET, OUB_IP_ADDR, &addr.sin_addr);

    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) 
    {
        ROS_ERROR("Bind failed: %s", strerror(errno));
        close(sockfd);
        return 1;
    }

    if (set_nonblocking(sockfd) < 0) 
    {
        ROS_ERROR("Failed to set non-blocking");
        close(sockfd);
        return 1;
    }

    MessageQueue queue;
    J2735_DECODE decoder;

    std::thread recv_thread(udpReceiverThread, sockfd, std::ref(queue));
    ros::Rate loop_rate(1000);

    while (ros::ok()) 
    {
        ReceivedMsg msg;
        if (queue.pop(msg)) 
        {
            decoder.decode(&msg, sdsm_pub);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    recv_thread.join();
    close(sockfd);
    return 0;
}

// J2735_DECODE::J2735_DECODE()
// {
//     memset(&obu_addr, 0, sizeof(obu_addr));
//     memset(buffer, 0, BUF_SIZE);

//     sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
//     if(sock_fd < 0)
//     {
//         ROS_ERROR("socket");
//         exit(0);
//     }

//     obu_addr.sin_family = AF_INET;
//     obu_addr.sin_port = htons(UDP_PORT);
//     if (inet_pton(AF_INET, OUB_IP_ADDR, &obu_addr.sin_addr) <= 0) {
//         ROS_ERROR("Invalid IP address");
//         exit(0);
//     }

//     setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&optVal, optLen);
//     if(sock_fd < 0)
//     {
//         ROS_ERROR("socket init error");
//         exit(0);
//     }

//     if(bind(sock_fd, (struct sockaddr *) &obu_addr, sizeof(obu_addr)) < 0)
//     {
//         ROS_ERROR("bind error");
//         exit(0);
//     }

//     sdsm_pub = nh.advertise<j3224_msgs::sdsm>("/v2x_obu/sdsm", 1);
// }

// J2735_DECODE::~J2735_DECODE()
// {

// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "j2735_decoder");
//     ros::NodeHandle nh;

//     ROS_INFO("j2735_decoder start");

//     uint8_t buffer[] = {
//     0x03, 0x6a, 0xb8, 0xff, 0xeb,
//     0x3f, 0x9f, 0xa3, 0x15, 0x04, 0x80, 0x0e, 0x94,
//     0xc5, 0xcb, 0x44, 0x7b, 0x70, 0x95, 0xcc, 0x32,
//     0xe6, 0x11, 0x31, 0x30, 0x00, 0x00, 0x00, 0x00,
//     0x64, 0x26, 0x6e, 0xbb, 0x80, 0x40, 0x00, 0x00,
//     0x00, 0x1f, 0x2a, 0x00, 0x00, 0x0a, 0x00, 0x84, 0x80
//     };
//     size_t buffer_len = sizeof(buffer);

//     // 3. 디코딩 실행
//     ASN1Error err;
//     void *msg = NULL;
//     asn1_ssize_t dec_ret = asn1_uper_decode(&msg, asn1_type_j2735SensorDataSharingMessage,
//                                         buffer, buffer_len, &err);
//     if (dec_ret < 0 || msg == nullptr) {
//     ROS_ERROR("asn1_uper_decode failed: %s", err.msg);  // ← 이거 출력 OK
//     return 1;  // ← 여기서 무조건 종료해야 segfault 방지됨!
//     }
    
//     // 4. 필드 출력 (예: msgCnt, sourceID, lat, long)
//     j2735SensorDataSharingMessage *sdsm = (j2735SensorDataSharingMessage *)msg;
    
//     std::stringstream id_ss;
//     for (size_t i = 0; i < sdsm->sourceID.len; ++i) {
//         id_ss << std::hex << std::setw(2) << std::setfill('0') << (int)sdsm->sourceID.buf[i];

//     }
    
//     ROS_INFO("SDSM Decoded:");
//     ROS_INFO("  msgCnt: %d", sdsm->msgCnt);
//     ROS_INFO("  sourceID: 0x%s", id_ss.str().c_str());
//     ROS_INFO("  year: %d", sdsm->sDSMTimeStamp.year);
//     ROS_INFO("  mon: %d", sdsm->sDSMTimeStamp.month);
//     ROS_INFO("  day: %d", sdsm->sDSMTimeStamp.day);
//     ROS_INFO("  hour: %d", sdsm->sDSMTimeStamp.hour);
//     ROS_INFO("  min: %d", sdsm->sDSMTimeStamp.minute);
//     ROS_INFO("  sec: %d", sdsm->sDSMTimeStamp.second);    
//     ROS_INFO("  refPos.lat: %d", sdsm->refPos.lat);
//     ROS_INFO("  refPos.long: %d", sdsm->refPos.Long);

//     // 객체 수 확인 (필드 이름은 사용 중인 ffasn1 구조체에 따라 다름)
//     if (sdsm->objects.count > 0) {
//         ROS_INFO("  objects.count: %ld", sdsm->objects.count);
//         auto &obj = sdsm->objects.tab[0].detObjCommon;
//         ROS_INFO("    measerTime: %d", obj.measurementTime);
//         ROS_INFO("    objectID: %d", obj.objectID);
//         ROS_INFO("    objType: %d", obj.objType);
//         ROS_INFO("    pos: %d   %d", obj.pos.offsetX, obj.pos.offsetY);
//         ROS_INFO("    speed: %d (0.02m/s unit)", obj.speed);
//         ROS_INFO("    heading: %d (0.0125deg unit)", obj.heading);
//     }

//     // 5. 메모리 정리
//     asn1_free_value(asn1_type_j2735SensorDataSharingMessage, msg);
//     // free(msg);

//     return 0;
// }
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>

#include "ffasn1-j2735-2026-KSR1600.h"
#include "asn1defs.h"

#include <v2x_msgs/intersection_msg.h>
#include <v2x_msgs/intersection_array_msg.h>
#include <j3224_msgs/sdsm.h>
#include <mmc_msgs/to_control_team_from_local_msg.h>

#include "siheung_v2x/sdsm_decode.h"

#define UDP_PORT 9999
#define BUF_SIZE 4096

// OBU 헤더 (5 bytes)
// [0] Frame Type   : 0=OBU→PC, 1=PC→OBU
// [1] Seq No       : 0~255
// [2] msg source   : 0=from RSU, 1=from Uu
// [3] isMsgFrame   : 0=MessageFrame, 1=not MessageFrame
// [4] Reserved
#define OBU_HEADER_SIZE 5

// J2735 SPaT messageId = 19 (0x13)
#define J2735_MSG_ID_SPAT 19

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

// 현재 차선 링크 정보 (to_control_team 구독으로 갱신)
struct CurrentLinkInfo {
    std::mutex mtx;
    int intersection_id = 0;
    int signal_group_id = 0;
    int manuaver = 0;  // -1=left, 0=straight, 1=right
};

CurrentLinkInfo g_link_info;

void toControlTeamCallback(const mmc_msgs::to_control_team_from_local_msg::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(g_link_info.mtx);
    g_link_info.intersection_id = msg->look_at_IntersectionID;
    g_link_info.signal_group_id = msg->look_at_signalGroupID;
    g_link_info.manuaver = msg->MANUAVER;
}

class SpatDecoder{
    public:
        // WSMP 서브헤더 건너뛰기 (03 80 + BER 길이)
        // 실제 OBU 패킷: [OBU 5B] + [WSMP subheader] + [J2735 UPER]
        // 테스트 패킷:   [OBU 5B] + [J2735 UPER] (WSMP 없음)
        static size_t skipWsmpHeader(const uint8_t* body, size_t body_len)
        {
            if (body_len < 4)
                return 0;

            if (body[0] != 0x03 || body[1] != 0x80)
                return 0;  // WSMP 헤더 없음 → 스킵 불필요

            size_t offset = 2;
            uint8_t ber_byte = body[offset];
            if (ber_byte < 0x80)
                offset += 1;        // short form: 1바이트 길이
            else if (ber_byte == 0x81)
                offset += 2;        // long form: 1+1 바이트
            else if (ber_byte == 0x82)
                offset += 3;        // long form: 1+2 바이트
            else
                offset += 1;        // fallback

            return offset;
        }

        // SPaT 구조체 → ROS 메시지 변환 및 발행
        void publishSpat(j2735SPAT* spat, ros::Publisher& spat_pub)
        {
            // 현재 링크 정보
            int cur_intersection_id, cur_signal_group, cur_manuaver;
            {
                std::lock_guard<std::mutex> lock(g_link_info.mtx);
                cur_intersection_id = g_link_info.intersection_id;
                cur_signal_group = g_link_info.signal_group_id;
                cur_manuaver = g_link_info.manuaver;
            }

            v2x_msgs::intersection_array_msg spat_msg;
            spat_msg.time = ros::Time::now();

            for (size_t i = 0; i < spat->intersections.count; ++i)
            {
                auto& intersection = spat->intersections.tab[i];
                int iid = intersection.id.id;

                for (size_t j = 0; j < intersection.states.count; ++j)
                {
                    auto& movement = intersection.states.tab[j];

                    v2x_msgs::intersection_msg int_msg;
                    int_msg.IntersectionID = iid;
                    int_msg.RoadRegulatorID = intersection.id.region_option ? intersection.id.region : 0;
                    int_msg.MsgCount = intersection.revision;
                    int_msg.NumOfMovementState = intersection.states.count;

                    // movementName (STR / LEFT / PED 등)
                    if (movement.movementName_option && movement.movementName.buf)
                    {
                        int_msg.Movements.MovementStateName = std::string(
                            (char*)movement.movementName.buf, movement.movementName.len);
                    }

                    int_msg.Movements.SignalGroupID = movement.signalGroup;

                    if (movement.state_time_speed.count > 0)
                    {
                        auto& evt = movement.state_time_speed.tab[0];
                        int_msg.Movements.MovementPhaseStatus = (uint8_t)evt.eventState;

                        if (evt.timing_option)
                            int_msg.Movements.TimeChangeDetails = evt.timing.minEndTime;
                    }

                    if (intersection.moy_option)
                        int_msg.MinuteOfTheYear = intersection.moy;
                    if (intersection.timeStamp_option)
                        int_msg.DSecond = intersection.timeStamp;

                    spat_msg.data.push_back(int_msg);
                }
            }

            spat_pub.publish(spat_msg);

            // 현재 링크 매칭 신호 로그
            // MANUAVER: -1→LEFT, 0→STR, 1→RIGHT(없으면 STR 사용)
            if (cur_intersection_id != 0)
            {
                const char* target_name = (cur_manuaver == -1) ? "LEFT" :
                                           (cur_manuaver ==  1) ? "RIGHT" : "STR";

                for (auto& d : spat_msg.data)
                {
                    if ((int)d.IntersectionID == cur_intersection_id &&
                        (int)d.Movements.SignalGroupID == cur_signal_group &&
                        d.Movements.MovementStateName == target_name)
                    {
                        const char* phase = "UNKNOWN";
                        switch (d.Movements.MovementPhaseStatus)
                        {
                            case 0: phase = "unavailable"; break;
                            case 1: phase = "dark"; break;
                            case 2: phase = "stop-Then-Proceed"; break;
                            case 3: phase = "STOP(red)"; break;
                            case 4: phase = "pre-Movement"; break;
                            case 5: phase = "GO(green-perm)"; break;
                            case 6: phase = "GO(green-prot)"; break;
                            case 7: phase = "clearance(perm)"; break;
                            case 8: phase = "clearance(prot)"; break;
                            case 9: phase = "caution"; break;
                        }
                        ROS_INFO("[SPaT] IntID=%d SigGrp=%d Move=%s Phase=%s minEnd=%d",
                                 cur_intersection_id, cur_signal_group, target_name, phase,
                                 d.Movements.TimeChangeDetails);
                        break;
                    }
                }
            }
        }

        // 메인 디코드 함수
        void decode(const ReceivedMsg* rmsg, ros::Publisher& spat_pub, ros::Publisher& sdsm_pub)
        {
            const uint8_t* raw = rmsg->data.data();
            size_t raw_len = rmsg->len;

            if (raw_len <= OBU_HEADER_SIZE)
            {
                ROS_WARN("[V2X] packet too short: %zu bytes", raw_len);
                return;
            }

            // OBU 헤더 파싱
            uint8_t is_msg_frame = raw[3];  // 0=MessageFrame, 1=not

            const uint8_t* body = raw + OBU_HEADER_SIZE;
            size_t body_len = raw_len - OBU_HEADER_SIZE;

            // WSMP 서브헤더 건너뛰기 (실제 OBU 패킷에 포함됨)
            size_t wsmp_off = skipWsmpHeader(body, body_len);
            if (wsmp_off > 0)
            {
                ROS_DEBUG("[V2X] WSMP subheader detected (%zu bytes), skipping", wsmp_off);
                body += wsmp_off;
                body_len -= wsmp_off;
            }

            if (is_msg_frame == 0)
            {
                // MessageFrame → SPaT(UPER) 또는 SDSM(BER) 시도
                ASN1Error err;
                void* frame_msg = nullptr;

                // SPaT: UPER MessageFrame 디코딩 시도
                asn1_ssize_t ret = asn1_uper_decode(&frame_msg, asn1_type_j2735MessageFrame,
                                                     body, body_len, &err);
                if (ret > 0 && frame_msg)
                {
                    j2735MessageFrame* frame = (j2735MessageFrame*)frame_msg;

                    if (frame->messageId == J2735_MSG_ID_SPAT)
                    {
                        // SPaT 처리
                        j2735SPAT* spat = nullptr;
                        void* spat_standalone = nullptr;

                        if (frame->value.type != nullptr)
                        {
                            spat = (j2735SPAT*)frame->value.u.data;
                        }
                        else
                        {
                            ASN1String* raw_bytes = &frame->value.u.octet_string;
                            asn1_ssize_t spat_ret = asn1_uper_decode(&spat_standalone, asn1_type_j2735SPAT,
                                                                      raw_bytes->buf, raw_bytes->len, &err);
                            if (spat_ret > 0 && spat_standalone)
                                spat = (j2735SPAT*)spat_standalone;
                        }

                        if (spat)
                            publishSpat(spat, spat_pub);

                        if (spat_standalone)
                            asn1_free_value(asn1_type_j2735SPAT, spat_standalone);
                        asn1_free_value(asn1_type_j2735MessageFrame, frame_msg);
                        return;
                    }

                    asn1_free_value(asn1_type_j2735MessageFrame, frame_msg);
                }

                ROS_DEBUG("[V2X] isMsgFrame=0 but not SPaT MessageFrame (len=%zu)", body_len);
            }
            else
            {
                // isMsgFrame=1: MessageFrame이 아님
                // SDSM UPER 직접 디코딩 시도 (경2 SDSM)
                if (decode_sdsm_uper(body, body_len, sdsm_pub))
                    return;

                // SDSM 실패 → SPaT UPER 직접 디코딩 시도
                ASN1Error err;
                void* spat_msg = nullptr;
                asn1_ssize_t ret = asn1_uper_decode(&spat_msg, asn1_type_j2735SPAT,
                                                     body, body_len, &err);
                if (ret > 0 && spat_msg)
                {
                    j2735SPAT* spat = (j2735SPAT*)spat_msg;
                    publishSpat(spat, spat_pub);
                    asn1_free_value(asn1_type_j2735SPAT, spat_msg);
                    return;
                }

                ROS_DEBUG("[V2X] unable to decode non-MessageFrame packet (len=%zu)", body_len);
            }
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
    ros::init(argc, argv, "siheung_v2x_node");
    ros::NodeHandle nh;

    ros::Publisher spat_pub = nh.advertise<v2x_msgs::intersection_array_msg>("/siheung_spat", 1);
    ros::Publisher sdsm_pub = nh.advertise<j3224_msgs::sdsm>("/obu/sdsm", 1);
    ros::Subscriber ctrl_sub = nh.subscribe("/localization/to_control_team", 1, toControlTeamCallback);

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        ROS_ERROR("Socket creation failed");
        return 1;
    }

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

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
    SpatDecoder decoder;

    std::thread recv_thread(udpReceiverThread, sockfd, std::ref(queue));
    ros::Rate loop_rate(1000);

    ROS_INFO("[siheung_v2x] started - SPaT(KSR1600) + SDSM(2020), publishing /siheung_spat, /obu/sdsm");
    ROS_INFO("[siheung_v2x] UDP port=%d, OBU header=%d bytes", UDP_PORT, OBU_HEADER_SIZE);

    while (ros::ok())
    {
        ReceivedMsg msg;
        if (queue.pop(msg))
        {
            decoder.decode(&msg, spat_pub, sdsm_pub);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    recv_thread.join();
    close(sockfd);
    return 0;
}

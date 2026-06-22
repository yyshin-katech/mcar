#include <ros/ros.h>
#include <v2x_msgs/v2x_pedes_assist_msg.h>
#include <v2x_msgs/v2x_tim_can_go_msg.h>
#include <v2x_msgs/v2x_tim_total_msg.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <condition_variable>
#include <cstring>
#include <iomanip>
#include <mutex>
#include <queue>
#include <cstdlib>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "asn1defs.h"
#include "ffasn1-j2735-2026-KSR1600.h"
#include "siheung_v2x/tim_publish.h"

#define DEFAULT_TIM_UDP_PORT 9999
#define BUF_SIZE 8192
#define OBU_HEADER_SIZE 5
#define J2735_MSG_ID_TIM 31
#define J2735_MSG_ID_SPAT 19
#define J2735_SCAN_LIMIT 64

enum class DecodeResult {
    Failed,
    Ignored,
    Published,
};

struct ReceivedMsg {
    std::vector<uint8_t> data;
    size_t len;
};

struct PayloadCandidate {
    size_t offset;
    std::string reason;
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
        if (cv_.wait_for(lock, std::chrono::milliseconds(10), [&] { return !queue_.empty(); }))
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

static size_t skipWsmpHeader(const uint8_t* body, size_t body_len)
{
    if (body_len < 4 || body[0] != 0x03 || body[1] != 0x80)
        return 0;

    size_t offset = 2;
    uint8_t ber_byte = body[offset];
    if (ber_byte < 0x80)
        offset += 1;
    else if (ber_byte == 0x81)
        offset += 2;
    else if (ber_byte == 0x82)
        offset += 3;
    else
        offset += 1;

    return offset;
}

static std::string hexPrefix(const uint8_t* data, size_t len, size_t limit = 24)
{
    std::ostringstream ss;
    const size_t n = std::min(len, limit);
    for (size_t i = 0; i < n; ++i)
    {
        if (i > 0)
            ss << " ";
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]);
    }
    return ss.str();
}

static bool containsAscii(const uint8_t* data, size_t len, const char* text)
{
    const size_t text_len = std::strlen(text);
    if (text_len == 0 || len < text_len)
        return false;

    for (size_t i = 0; i + text_len <= len; ++i)
    {
        if (std::memcmp(data + i, text, text_len) == 0)
            return true;
    }
    return false;
}

static bool isServiceAdvertisementPacket(const uint8_t* data, size_t len)
{
    return containsAscii(data, len, "WSM_TIM") ||
           containsAscii(data, len, "WSM_MAP") ||
           containsAscii(data, len, "WSM_SPAT") ||
           containsAscii(data, len, "WSM_PVD") ||
           containsAscii(data, len, "WSM_RSA");
}

static const char* j2735MessageName(int msg_id)
{
    switch (msg_id)
    {
        case 18: return "MAP";
        case 19: return "SPaT";
        case 20: return "BSM";
        case 31: return "TIM";
        case 41: return "SDSM";
        default: return "unknown";
    }
}

static bool isKnownJ2735MessageId(int msg_id)
{
    switch (msg_id)
    {
        case 18:
        case 19:
        case 20:
        case 31:
        case 41:
            return true;
        default:
            return false;
    }
}

class TimDecoder {
public:
    explicit TimDecoder(bool allow_raw_tim, int target_msg_id)
        : allow_raw_tim_(allow_raw_tim), target_msg_id_(target_msg_id) {}

    bool decodePacket(const ReceivedMsg& rmsg,
                      ros::Publisher& tim_pub,
                      ros::Publisher& pedes_pub,
                      ros::Publisher& go_ahead_pub)
    {
        if (rmsg.len <= OBU_HEADER_SIZE)
        {
            ROS_WARN("[TIM] packet too short: %zu bytes", rmsg.len);
            return false;
        }

        const uint8_t* raw = rmsg.data.data();
        ROS_INFO_THROTTLE(2.0, "[TIM] rx packet len=%zu prefix=%s",
                          rmsg.len, hexPrefix(raw, rmsg.len).c_str());

        if (isServiceAdvertisementPacket(raw, rmsg.len))
        {
            ROS_INFO_THROTTLE(5.0, "[TIM] ignored service advertisement packet");
            return true;
        }

        DecodeResult best_result = DecodeResult::Failed;
        int first_non_tim_msg_id = -1;
        size_t first_non_tim_offset = 0;

        for (const auto& candidate : buildPayloadCandidates(raw, rmsg.len))
        {
            const uint8_t* body = raw + candidate.offset;
            const size_t body_len = rmsg.len - candidate.offset;
            int decoded_msg_id = -1;
            DecodeResult frame_result = decodeMessageFrame(body, body_len, tim_pub, pedes_pub, go_ahead_pub,
                                                           candidate.offset, candidate.reason,
                                                           &decoded_msg_id);
            if (frame_result == DecodeResult::Published)
                return true;

            if (frame_result == DecodeResult::Ignored && isKnownJ2735MessageId(decoded_msg_id))
            {
                best_result = DecodeResult::Ignored;
                if (first_non_tim_msg_id < 0)
                {
                    first_non_tim_msg_id = decoded_msg_id;
                    first_non_tim_offset = candidate.offset;
                }
            }
        }

        if (allow_raw_tim_)
        {
            for (const auto& candidate : buildPayloadCandidates(raw, rmsg.len))
            {
                const uint8_t* body = raw + candidate.offset;
                const size_t body_len = rmsg.len - candidate.offset;
                if (decodeRawTim(body, body_len, tim_pub, pedes_pub, go_ahead_pub,
                                 candidate.offset, candidate.reason))
                    return true;
            }
        }

        if (best_result == DecodeResult::Ignored)
        {
            ROS_INFO_THROTTLE(2.0,
                              "[TIM] valid J2735 received but not target msgId=%d(%s): msgId=%d(%s), offset=%zu. "
                              "If /v2x/tim_message is empty, OBU/RSU may be forwarding another service payload.",
                              target_msg_id_,
                              j2735MessageName(target_msg_id_),
                              first_non_tim_msg_id,
                              j2735MessageName(first_non_tim_msg_id),
                              first_non_tim_offset);
            return true;
        }

        ROS_WARN_THROTTLE(2.0, "[TIM] decode failed len=%zu prefix=%s",
                          rmsg.len, hexPrefix(raw, rmsg.len).c_str());
        return false;
    }

private:
    static void addCandidate(std::vector<PayloadCandidate>& candidates, size_t offset,
                             const std::string& reason, size_t packet_len)
    {
        if (offset >= packet_len)
            return;

        for (const auto& candidate : candidates)
        {
            if (candidate.offset == offset)
                return;
        }

        candidates.push_back({offset, reason});
    }

    static std::vector<PayloadCandidate> buildPayloadCandidates(const uint8_t* raw, size_t packet_len)
    {
        std::vector<PayloadCandidate> candidates;

        addCandidate(candidates, 0, "udp-payload", packet_len);

        if (packet_len > OBU_HEADER_SIZE)
        {
            const uint8_t* obu_body = raw + OBU_HEADER_SIZE;
            const size_t obu_body_len = packet_len - OBU_HEADER_SIZE;

            addCandidate(candidates, OBU_HEADER_SIZE, "obu-body", packet_len);

            const size_t wsmp_off = skipWsmpHeader(obu_body, obu_body_len);
            if (wsmp_off > 0)
                addCandidate(candidates, OBU_HEADER_SIZE + wsmp_off, "obu-wsmp", packet_len);
        }

        const size_t scan_limit = std::min(packet_len, static_cast<size_t>(J2735_SCAN_LIMIT));
        for (size_t offset = 0; offset < scan_limit; ++offset)
        {
            if (offset + 1 >= packet_len)
                break;

            if (raw[offset] == 0x00 &&
                (raw[offset + 1] == 0x12 || raw[offset + 1] == 0x13 ||
                 raw[offset + 1] == 0x14 || raw[offset + 1] == 0x1f ||
                 raw[offset + 1] == 0x29))
            {
                addCandidate(candidates, offset, "message-id-pattern", packet_len);
            }
        }

        return candidates;
    }

    DecodeResult decodeMessageFrame(const uint8_t* body, size_t body_len,
                                    ros::Publisher& tim_pub,
                                    ros::Publisher& pedes_pub,
                                    ros::Publisher& go_ahead_pub,
                                    size_t offset, const std::string& reason, int* decoded_msg_id)
    {
        if (decoded_msg_id)
            *decoded_msg_id = -1;

        ASN1Error err;
        void* frame_msg = nullptr;
        asn1_ssize_t ret = asn1_uper_decode(&frame_msg, asn1_type_j2735MessageFrame,
                                             body, body_len, &err);
        if (ret <= 0 || !frame_msg)
            return DecodeResult::Failed;

        j2735MessageFrame* frame = (j2735MessageFrame*)frame_msg;
        if (decoded_msg_id)
            *decoded_msg_id = static_cast<int>(frame->messageId);

        if (!isKnownJ2735MessageId(static_cast<int>(frame->messageId)))
        {
            asn1_free_value(asn1_type_j2735MessageFrame, frame_msg);
            return DecodeResult::Failed;
        }

        DecodeResult result = DecodeResult::Ignored;

        if (frame->messageId == target_msg_id_)
        {
            if (frame->messageId == J2735_MSG_ID_TIM)
            {
                j2735TravelerInformation* tim = nullptr;
                void* standalone = nullptr;

                if (frame->value.type != nullptr)
                {
                    tim = (j2735TravelerInformation*)frame->value.u.data;
                }
                else
                {
                    ASN1String* raw_bytes = &frame->value.u.octet_string;
                    asn1_ssize_t tim_ret = asn1_uper_decode(&standalone, asn1_type_j2735TravelerInformation,
                                                             raw_bytes->buf, raw_bytes->len, &err);
                    if (tim_ret > 0 && standalone)
                        tim = (j2735TravelerInformation*)standalone;
                }

                if (tim)
                {
                    ROS_INFO("[TIM] decoded MessageFrame offset=%zu body_len=%zu reason=%s",
                             offset, body_len, reason.c_str());
                    tim_publish::publish(tim, tim_pub, pedes_pub, go_ahead_pub);
                    result = DecodeResult::Published;
                }
                else
                {
                    ROS_WARN_THROTTLE(2.0,
                                      "[TIM] msgId=TIM(31) but TravelerInformation payload decode failed, offset=%zu body_len=%zu reason=%s",
                                      offset, body_len, reason.c_str());
                }

                if (standalone)
                    asn1_free_value(asn1_type_j2735TravelerInformation, standalone);
            }
            else if (frame->messageId == J2735_MSG_ID_SPAT)
            {
                j2735SPAT* spat = nullptr;
                void* standalone = nullptr;

                if (frame->value.type != nullptr)
                {
                    spat = (j2735SPAT*)frame->value.u.data;
                }
                else
                {
                    ASN1String* raw_bytes = &frame->value.u.octet_string;
                    asn1_ssize_t spat_ret = asn1_uper_decode(&standalone, asn1_type_j2735SPAT,
                                                              raw_bytes->buf, raw_bytes->len, &err);
                    if (spat_ret > 0 && standalone)
                        spat = (j2735SPAT*)standalone;
                }

                if (spat)
                {
                    ROS_INFO("[TIM] target_msg_id=19 decoded SPaT probe offset=%zu body_len=%zu reason=%s",
                             offset, body_len, reason.c_str());
                    publishSpatProbe(spat, tim_pub);
                    result = DecodeResult::Published;
                }
                else
                {
                    ROS_WARN_THROTTLE(2.0,
                                      "[TIM] msgId=SPaT(19) but SPaT payload decode failed, offset=%zu body_len=%zu reason=%s",
                                      offset, body_len, reason.c_str());
                }

                if (standalone)
                    asn1_free_value(asn1_type_j2735SPAT, standalone);
            }
        }
        else
        {
            ROS_INFO_THROTTLE(2.0, "[TIM] ignored MessageFrame msgId=%d(%s), target=%d(%s), offset=%zu reason=%s",
                              static_cast<int>(frame->messageId),
                              j2735MessageName(static_cast<int>(frame->messageId)),
                              target_msg_id_,
                              j2735MessageName(target_msg_id_),
                              offset,
                              reason.c_str());
        }

        asn1_free_value(asn1_type_j2735MessageFrame, frame_msg);
        return result;
    }

    bool decodeRawTim(const uint8_t* body, size_t body_len,
                      ros::Publisher& tim_pub,
                      ros::Publisher& pedes_pub,
                      ros::Publisher& go_ahead_pub,
                      size_t offset, const std::string& reason)
    {
        ASN1Error err;
        void* tim_msg = nullptr;
        asn1_ssize_t ret = asn1_uper_decode(&tim_msg, asn1_type_j2735TravelerInformation,
                                             body, body_len, &err);
        if (ret <= 0 || !tim_msg)
            return false;

        ROS_INFO("[TIM] decoded raw TravelerInformation offset=%zu body_len=%zu reason=%s",
                 offset, body_len, reason.c_str());
        tim_publish::publish((j2735TravelerInformation*)tim_msg, tim_pub, pedes_pub, go_ahead_pub);
        asn1_free_value(asn1_type_j2735TravelerInformation, tim_msg);
        return true;
    }

    void publishSpatProbe(j2735SPAT* spat, ros::Publisher& tim_pub)
    {
        (void)tim_pub;

        ROS_INFO_THROTTLE(1.0,
                          "[TIM-OBU] decoded SPaT probe intersections=%zu target_msg_id=%d; /v2x/tim_message publishes TIM total messages only",
                          spat->intersections.count,
                          target_msg_id_);
    }

    bool allow_raw_tim_;
    int target_msg_id_;
};

static int setNonblocking(int sockfd)
{
    int flags = fcntl(sockfd, F_GETFL, 0);
    return (flags < 0) ? -1 : fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
}

static bool setReuseOptions(int sockfd)
{
    int opt = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
    {
        ROS_ERROR("[TIM] failed to set SO_REUSEADDR: %s", strerror(errno));
        return false;
    }

#ifdef SO_REUSEPORT
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt)) < 0)
    {
        ROS_WARN("[TIM] failed to set SO_REUSEPORT: %s", strerror(errno));
    }
#endif

    return true;
}

static void udpReceiverThread(int sockfd, MessageQueue& queue)
{
    uint8_t buffer[BUF_SIZE];
    struct sockaddr_in sender_addr;
    socklen_t addrlen = sizeof(sender_addr);

    while (ros::ok())
    {
        ssize_t len = recvfrom(sockfd, buffer, BUF_SIZE, 0,
                               (struct sockaddr*)&sender_addr, &addrlen);
        if (len > 0)
            queue.push({std::vector<uint8_t>(buffer, buffer + len), static_cast<size_t>(len)});
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "j2735_TIM_rx_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string bind_ip;
    int bind_port = 0;
    std::string topic;
    std::string pedes_topic;
    std::string go_ahead_topic;
    bool allow_raw_tim = false;
    int target_msg_id = J2735_MSG_ID_TIM;
    pnh.param<std::string>("bind_ip", bind_ip, "0.0.0.0");
    pnh.param<int>("bind_port", bind_port, DEFAULT_TIM_UDP_PORT);
    pnh.param<std::string>("topic", topic, "/v2x/tim_message");
    pnh.param<std::string>("pedes_topic", pedes_topic, "/obu/v2x_pedes_assistance");
    pnh.param<std::string>("go_ahead_topic", go_ahead_topic, "/v2x/tim_message/can_go_status");
    pnh.param<bool>("allow_raw_tim", allow_raw_tim, false);
    pnh.param<int>("target_msg_id", target_msg_id, J2735_MSG_ID_TIM);

    ros::Publisher tim_pub = nh.advertise<v2x_msgs::v2x_tim_total_msg>(topic, 1);
    ros::Publisher pedes_pub = nh.advertise<v2x_msgs::v2x_pedes_assist_msg>(pedes_topic, 1);
    ros::Publisher go_ahead_pub = nh.advertise<v2x_msgs::v2x_tim_can_go_msg>(go_ahead_topic, 1);

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        ROS_ERROR("[TIM] socket creation failed");
        return 1;
    }

    if (!setReuseOptions(sockfd))
    {
        close(sockfd);
        return 1;
    }

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(bind_port));
    if (inet_pton(AF_INET, bind_ip.c_str(), &addr.sin_addr) != 1)
    {
        ROS_ERROR("[TIM] invalid bind_ip '%s'", bind_ip.c_str());
        close(sockfd);
        return 1;
    }

    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        ROS_ERROR("[TIM] bind failed on %s:%d : %s", bind_ip.c_str(), bind_port, strerror(errno));
        close(sockfd);
        return 1;
    }

    if (setNonblocking(sockfd) < 0)
    {
        ROS_ERROR("[TIM] failed to set non-blocking");
        close(sockfd);
        return 1;
    }

    MessageQueue queue;
    TimDecoder decoder(allow_raw_tim, target_msg_id);
    std::thread recv_thread(udpReceiverThread, sockfd, std::ref(queue));
    ros::Rate loop_rate(1000);

    ROS_INFO("[TIM] UDP bound to %s:%d, publishing %s, %s and %s, allow_raw_tim=%s, target_msg_id=%d(%s)",
             bind_ip.c_str(),
             bind_port,
             topic.c_str(),
             pedes_topic.c_str(),
             go_ahead_topic.c_str(),
             allow_raw_tim ? "true" : "false",
             target_msg_id,
             j2735MessageName(target_msg_id));

    while (ros::ok())
    {
        ReceivedMsg msg;
        if (queue.pop(msg) && !decoder.decodePacket(msg, tim_pub, pedes_pub, go_ahead_pub))
            ROS_DEBUG("[TIM] packet ignored or decode failed (len=%zu)", msg.len);

        ros::spinOnce();
        loop_rate.sleep();
    }

    recv_thread.join();
    close(sockfd);
    return 0;
}

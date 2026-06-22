#include <ros/ros.h>
#include <v2x_msgs/v2x_pedes_assist_msg.h>
#include <v2x_msgs/v2x_tim_can_go_msg.h>
#include <v2x_msgs/v2x_tim_content_msg.h>
#include <v2x_msgs/v2x_tim_dataframe_msg.h>
#include <v2x_msgs/v2x_tim_region_msg.h>
#include <v2x_msgs/v2x_tim_regional_msg.h>
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

static std::string asn1StringToString(const ASN1String& s, bool present)
{
    if (!present || !s.buf || s.len == 0)
        return "";
    return std::string(reinterpret_cast<const char*>(s.buf), s.len);
}

static std::string asn1StringToString(const ASN1String& s)
{
    return asn1StringToString(s, true);
}

static std::string bytesToHex(const uint8_t* data, size_t len)
{
    std::ostringstream ss;
    for (size_t i = 0; i < len; ++i)
    {
        if (i > 0)
            ss << " ";
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]);
    }
    return ss.str();
}

static std::string bitStringToHex(const ASN1BitString& value)
{
    const size_t byte_len = (value.len + 7) / 8;
    if (!value.buf || byte_len == 0)
        return "";
    return bytesToHex(value.buf, byte_len);
}

static std::string jsonEscape(const std::string& input)
{
    std::ostringstream out;
    for (unsigned char c : input)
    {
        switch (c)
        {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default:
                if (c < 0x20)
                {
                    out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(c) << std::dec << std::setfill(' ');
                }
                else
                {
                    out << c;
                }
                break;
        }
    }
    return out.str();
}

static void appendAsn1Integer(std::ostringstream& ss, const ASN1Integer& value);

static std::string openTypeOctetString(const ASN1OpenType& value)
{
    if (value.type != nullptr || !value.u.octet_string.buf || value.u.octet_string.len == 0)
        return "";

    return std::string(reinterpret_cast<const char*>(value.u.octet_string.buf),
                       value.u.octet_string.len);
}

static std::string extractJsonObject(const std::string& raw)
{
    const size_t begin = raw.find('{');
    const size_t end = raw.rfind('}');
    if (begin == std::string::npos || end == std::string::npos || begin > end)
        return "";
    return raw.substr(begin, end - begin + 1);
}

static std::string unescapeJsonText(const std::string& input)
{
    std::ostringstream out;
    bool escape = false;

    for (char c : input)
    {
        if (escape)
        {
            switch (c)
            {
                case '"': out << '"'; break;
                case '\\': out << '\\'; break;
                case '/': out << '/'; break;
                case 'b': out << '\b'; break;
                case 'f': out << '\f'; break;
                case 'n': out << '\n'; break;
                case 'r': out << '\r'; break;
                case 't': out << '\t'; break;
                default: out << c; break;
            }
            escape = false;
            continue;
        }

        if (c == '\\')
        {
            escape = true;
            continue;
        }

        out << c;
    }

    if (escape)
        out << '\\';

    return out.str();
}

static std::vector<std::string> jsonCandidatesFromOpenType(const ASN1OpenType& value)
{
    std::vector<std::string> candidates;
    const std::string raw = openTypeOctetString(value);
    const std::string direct = extractJsonObject(raw);
    if (!direct.empty())
        candidates.push_back(direct);

    const std::string unescaped = unescapeJsonText(raw);
    const std::string unescaped_object = extractJsonObject(unescaped);
    if (!unescaped_object.empty() &&
        std::find(candidates.begin(), candidates.end(), unescaped_object) == candidates.end())
    {
        candidates.push_back(unescaped_object);
    }

    if (raw.size() >= 2 && raw.front() == '"' && raw.back() == '"')
    {
        const std::string inner = raw.substr(1, raw.size() - 2);
        const std::string inner_unescaped = unescapeJsonText(inner);
        const std::string inner_object = extractJsonObject(inner_unescaped);
        if (!inner_object.empty() &&
            std::find(candidates.begin(), candidates.end(), inner_object) == candidates.end())
        {
            candidates.push_back(inner_object);
        }
    }

    return candidates;
}

static bool findJsonStringValue(const std::string& json, const std::string& key, std::string& value)
{
    const std::string pattern = "\"" + key + "\"";
    size_t pos = json.find(pattern);
    if (pos == std::string::npos)
        return false;

    pos = json.find(':', pos + pattern.size());
    if (pos == std::string::npos)
        return false;

    pos = json.find('"', pos + 1);
    if (pos == std::string::npos)
        return false;

    std::ostringstream out;
    bool escape = false;
    for (size_t i = pos + 1; i < json.size(); ++i)
    {
        const char c = json[i];
        if (escape)
        {
            switch (c)
            {
                case '"': out << '"'; break;
                case '\\': out << '\\'; break;
                case '/': out << '/'; break;
                case 'b': out << '\b'; break;
                case 'f': out << '\f'; break;
                case 'n': out << '\n'; break;
                case 'r': out << '\r'; break;
                case 't': out << '\t'; break;
                default: out << c; break;
            }
            escape = false;
            continue;
        }

        if (c == '\\')
        {
            escape = true;
            continue;
        }

        if (c == '"')
        {
            value = out.str();
            return true;
        }

        out << c;
    }

    return false;
}

static bool findJsonNumberValue(const std::string& json, const std::string& key, double& value)
{
    const std::string pattern = "\"" + key + "\"";
    size_t pos = json.find(pattern);
    if (pos == std::string::npos)
        return false;

    pos = json.find(':', pos + pattern.size());
    if (pos == std::string::npos)
        return false;

    ++pos;
    while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos])))
        ++pos;

    char* end = nullptr;
    value = std::strtod(json.c_str() + pos, &end);
    return end && end != json.c_str() + pos;
}

static bool findJsonBoolValue(const std::string& json, const std::string& key, bool& value)
{
    const std::string pattern = "\"" + key + "\"";
    size_t pos = json.find(pattern);
    if (pos == std::string::npos)
        return false;

    pos = json.find(':', pos + pattern.size());
    if (pos == std::string::npos)
        return false;

    ++pos;
    while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos])))
        ++pos;

    if (json.compare(pos, 4, "true") == 0)
    {
        value = true;
        return true;
    }

    if (json.compare(pos, 5, "false") == 0)
    {
        value = false;
        return true;
    }

    if (json.compare(pos, 6, "\"true\"") == 0 || json.compare(pos, 1, "1") == 0)
    {
        value = true;
        return true;
    }

    if (json.compare(pos, 7, "\"false\"") == 0 || json.compare(pos, 1, "0") == 0)
    {
        value = false;
        return true;
    }

    return false;
}

static bool findJsonObjectValue(const std::string& json, const std::string& key, std::string& value)
{
    const std::string pattern = "\"" + key + "\"";
    size_t pos = json.find(pattern);
    if (pos == std::string::npos)
        return false;

    pos = json.find(':', pos + pattern.size());
    if (pos == std::string::npos)
        return false;

    pos = json.find('{', pos + 1);
    if (pos == std::string::npos)
        return false;

    size_t depth = 0;
    bool in_string = false;
    bool escape = false;
    for (size_t i = pos; i < json.size(); ++i)
    {
        const char c = json[i];

        if (escape)
        {
            escape = false;
            continue;
        }

        if (in_string && c == '\\')
        {
            escape = true;
            continue;
        }

        if (c == '"')
        {
            in_string = !in_string;
            continue;
        }

        if (in_string)
            continue;

        if (c == '{')
        {
            ++depth;
        }
        else if (c == '}')
        {
            if (depth == 0)
                return false;

            --depth;
            if (depth == 0)
            {
                value = json.substr(pos, i - pos + 1);
                return true;
            }
        }
    }

    return false;
}

struct PedesAssistanceData {
    bool standard_time_valid = false;
    std::string standard_time;

    bool rsu_latitude_valid = false;
    double rsu_latitude = 0.0;

    bool rsu_longitude_valid = false;
    double rsu_longitude = 0.0;

    bool north_pedes_valid = false;
    bool north_pedes = false;

    bool east_pedes_valid = false;
    bool east_pedes = false;

    bool south_pedes_valid = false;
    bool south_pedes = false;

    bool west_pedes_valid = false;
    bool west_pedes = false;

    bool hasAny() const
    {
        return standard_time_valid || rsu_latitude_valid || rsu_longitude_valid ||
               north_pedes_valid || east_pedes_valid || south_pedes_valid || west_pedes_valid;
    }

    bool complete() const
    {
        return standard_time_valid && rsu_latitude_valid && rsu_longitude_valid &&
               north_pedes_valid && east_pedes_valid && south_pedes_valid && west_pedes_valid;
    }
};

static void updatePedesAssistanceFromRegional(const j2735TravelerInformation& tim,
                                              PedesAssistanceData& data)
{
    if (!tim.regional_option)
        return;

    for (size_t i = 0; i < tim.regional.count; ++i)
    {
        const auto& regional = tim.regional.tab[i];
        const std::vector<std::string> candidates = jsonCandidatesFromOpenType(regional.regExtValue);
        if (candidates.empty())
        {
            const std::string raw = openTypeOctetString(regional.regExtValue);
            ROS_DEBUG("[TIM-OBU] regional[%zu] regionId=%d has no JSON object, raw_prefix=%s",
                      i,
                      regional.regionId,
                      jsonEscape(raw.substr(0, std::min(raw.size(), static_cast<size_t>(80)))).c_str());
            continue;
        }

        for (const std::string& json : candidates)
        {
            std::string standard_time;
            if (findJsonStringValue(json, "standard_time", standard_time))
            {
                data.standard_time = standard_time;
                data.standard_time_valid = true;
            }

            double numeric = 0.0;
            if (findJsonNumberValue(json, "rsu_latitude", numeric))
            {
                data.rsu_latitude = numeric;
                data.rsu_latitude_valid = true;
            }

            if (findJsonNumberValue(json, "rsu_longitude", numeric))
            {
                data.rsu_longitude = numeric;
                data.rsu_longitude_valid = true;
            }

            std::string pedestrian_json;
            if (findJsonObjectValue(json, "pedestrian", pedestrian_json))
            {
                bool flag = false;
                if (findJsonBoolValue(pedestrian_json, "10", flag))
                {
                    data.north_pedes = flag;
                    data.north_pedes_valid = true;
                }

                if (findJsonBoolValue(pedestrian_json, "20", flag))
                {
                    data.east_pedes = flag;
                    data.east_pedes_valid = true;
                }

                if (findJsonBoolValue(pedestrian_json, "30", flag))
                {
                    data.south_pedes = flag;
                    data.south_pedes_valid = true;
                }

                if (findJsonBoolValue(pedestrian_json, "40", flag))
                {
                    data.west_pedes = flag;
                    data.west_pedes_valid = true;
                }
            }
        }
    }
}

static void appendOpenTypeJson(std::ostringstream& ss, const ASN1OpenType& value)
{
    ss << "{";

    if (value.type == nullptr)
    {
        const ASN1String& raw = value.u.octet_string;
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(raw.buf);
        const size_t len = raw.buf ? raw.len : 0;

        ss << "\"octet_string\":\"" << jsonEscape(openTypeOctetString(value)) << "\"";
        ss << ",\"hex\":\"" << (len > 0 ? bytesToHex(bytes, len) : "") << "\"";
        ss << ",\"length\":" << len;
    }
    else if (value.type == asn1_type_j2735TravelerInformation_KOR)
    {
        const j2735TravelerInformation_KOR* kor =
            static_cast<const j2735TravelerInformation_KOR*>(value.u.data);

        ss << "\"type\":\"TravelerInformation_KOR\"";
        if (kor)
        {
            ss << ",\"msgID\":";
            if (kor->msgID_option)
                appendAsn1Integer(ss, kor->msgID);
            else
                ss << "null";
            ss << ",\"additionalInfoCount\":"
               << (kor->additionalInfo_option ? kor->additionalInfo.count : 0);
        }
    }
    else
    {
        ss << "\"type\":\"typed-regional-extension\"";
    }

    ss << "}";
}

static void appendTimRegional(std::ostringstream& ss, const j2735TravelerInformation& tim)
{
    ss << ",\"regional\":[";

    if (tim.regional_option)
    {
        for (size_t i = 0; i < tim.regional.count; ++i)
        {
            const auto& regional = tim.regional.tab[i];
            if (i > 0)
                ss << ",";

            ss << "{";
            ss << "\"regionId\":" << regional.regionId << ",";
            ss << "\"regExtValue\":";
            appendOpenTypeJson(ss, regional.regExtValue);
            ss << "}";
        }
    }

    ss << "]";
}

static void appendOptionalInt(std::ostringstream& ss, bool present, int value)
{
    if (present)
        ss << value;
    else
        ss << "null";
}

static void appendAsn1Integer(std::ostringstream& ss, const ASN1Integer& value)
{
    if (value.len == 0)
    {
        ss << 0;
        return;
    }

    if (value.len > 2)
    {
        ss << "\"large\"";
        return;
    }

    uint64_t result = 0;
    for (uint32_t i = 0; i < value.len; ++i)
        result |= static_cast<uint64_t>(value.data[i]) << (i * 32);

    if (value.negative)
        ss << "-" << result;
    else
        ss << result;
}

static void appendAdvisory(std::ostringstream& ss, const j2735ITIScodesAndText& advisory)
{
    ss << ",\"advisory\":[";
    const size_t limit = std::min(advisory.count, static_cast<size_t>(16));
    for (size_t i = 0; i < limit; ++i)
    {
        const auto& item = advisory.tab[i].item;
        if (i > 0)
            ss << ",";

        if (item.choice == j2735ITIScodesAndText_1_itis)
            ss << "{\"itis\":" << item.u.itis << "}";
        else
            ss << "{\"text\":\"" << jsonEscape(asn1StringToString(item.u.text, true)) << "\"}";
    }
    ss << "]";

    if (advisory.count > limit)
        ss << ",\"truncatedAdvisory\":" << (advisory.count - limit);
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
                    publishTim(tim, tim_pub, pedes_pub, go_ahead_pub);
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
        publishTim((j2735TravelerInformation*)tim_msg, tim_pub, pedes_pub, go_ahead_pub);
        asn1_free_value(asn1_type_j2735TravelerInformation, tim_msg);
        return true;
    }

    void publishGoAhead(const j2735TravelerInformation& tim,
                        ros::Publisher& go_ahead_pub,
                        const std::string& packet_id)
    {
        if (packet_id.rfind("RNST", 0) != 0)
            return;

        PedesAssistanceData data;
        updatePedesAssistanceFromRegional(tim, data);

        v2x_msgs::v2x_tim_can_go_msg msg;
        msg.standard_time = data.standard_time_valid ? data.standard_time : "";
        msg.rsu_latitude = data.rsu_latitude_valid ? data.rsu_latitude : 0.0;
        msg.rsu_longitude = data.rsu_longitude_valid ? data.rsu_longitude : 0.0;
        msg.do_not_go_forward = true;

        go_ahead_pub.publish(msg);

        ROS_INFO_THROTTLE(1.0,
                          "[TIM-OBU] published do_not_go_forward=true packetID=%s standard_time=%s rsu=(%.9f, %.9f)",
                          packet_id.c_str(),
                          msg.standard_time.c_str(),
                          msg.rsu_latitude,
                          msg.rsu_longitude);
    }

    template <typename ContentList>
    void appendContentItems(v2x_msgs::v2x_tim_dataframe_msg& frame_msg,
                            const ContentList& list,
                            int32_t content_choice)
    {
        for (size_t i = 0; i < list.count; ++i)
        {
            const auto& item = list.tab[i].item;
            v2x_msgs::v2x_tim_content_msg content_msg;
            content_msg.content_index = static_cast<uint32_t>(i);
            content_msg.content_choice = content_choice;
            content_msg.item_choice = static_cast<int32_t>(item.choice);

            if (static_cast<int32_t>(item.choice) == 0)
            {
                content_msg.itis = static_cast<int32_t>(item.u.itis);
                content_msg.text = "";
            }
            else
            {
                content_msg.itis = 0;
                content_msg.text = asn1StringToString(item.u.text, true);
            }

            frame_msg.contents.push_back(content_msg);
        }
    }

    void appendFrameContent(v2x_msgs::v2x_tim_dataframe_msg& frame_msg,
                            const j2735TravelerDataFrame& frame)
    {
        const int32_t content_choice = static_cast<int32_t>(frame.content.choice);
        switch (frame.content.choice)
        {
            case j2735TravelerDataFrame_3_advisory:
                appendContentItems(frame_msg, frame.content.u.advisory, content_choice);
                break;
            case j2735TravelerDataFrame_3_workZone:
                appendContentItems(frame_msg, frame.content.u.workZone, content_choice);
                break;
            case j2735TravelerDataFrame_3_genericSign:
                appendContentItems(frame_msg, frame.content.u.genericSign, content_choice);
                break;
            case j2735TravelerDataFrame_3_speedLimit:
                appendContentItems(frame_msg, frame.content.u.speedLimit, content_choice);
                break;
            case j2735TravelerDataFrame_3_exitService:
                appendContentItems(frame_msg, frame.content.u.exitService, content_choice);
                break;
        }
    }

    v2x_msgs::v2x_tim_region_msg buildRegionMsg(const j2735GeographicalPath& region,
                                                uint32_t region_index)
    {
        v2x_msgs::v2x_tim_region_msg region_msg;
        region_msg.region_index = region_index;

        region_msg.name_present = region.name_option;
        region_msg.name = asn1StringToString(region.name, region.name_option);

        region_msg.id_present = region.id_option;
        region_msg.id_region_present = region.id_option && region.id.region_option;
        region_msg.id_region = (region.id_option && region.id.region_option) ? region.id.region : 0;
        region_msg.id = region.id_option ? region.id.id : 0;

        region_msg.anchor_present = region.anchor_option;
        region_msg.anchor_lat = region.anchor_option ? region.anchor.lat : 0;
        region_msg.anchor_long = region.anchor_option ? region.anchor.Long : 0;
        region_msg.anchor_elevation_present = region.anchor_option && region.anchor.elevation_option;
        region_msg.anchor_elevation = (region.anchor_option && region.anchor.elevation_option) ? region.anchor.elevation : 0;

        region_msg.lane_width_present = region.laneWidth_option;
        region_msg.lane_width = region.laneWidth_option ? region.laneWidth : 0;
        region_msg.directionality_present = region.directionality_option;
        region_msg.directionality = region.directionality_option ? static_cast<int32_t>(region.directionality) : 0;
        region_msg.closed_path_present = region.closedPath_option;
        region_msg.closed_path = region.closedPath_option ? region.closedPath : false;
        region_msg.direction_present = region.direction_option;
        region_msg.direction_hex = region.direction_option ? bitStringToHex(region.direction) : "";
        region_msg.direction_bits = region.direction_option ? static_cast<uint32_t>(region.direction.len) : 0;
        region_msg.description_present = region.description_option;
        region_msg.description_choice = region.description_option ? static_cast<int32_t>(region.description.choice) : -1;
        region_msg.regional_count = region.regional_option ? static_cast<uint32_t>(region.regional.count) : 0;

        return region_msg;
    }

    v2x_msgs::v2x_tim_dataframe_msg buildFrameMsg(const j2735TravelerDataFrame& frame,
                                                  uint32_t frame_index)
    {
        v2x_msgs::v2x_tim_dataframe_msg frame_msg;
        frame_msg.frame_index = frame_index;
        frame_msg.not_used = static_cast<uint8_t>(frame.notUsed);
        frame_msg.frame_type = static_cast<int32_t>(frame.frameType);
        frame_msg.msg_id_choice = static_cast<uint8_t>(frame.msgId.choice);

        const bool has_road_sign = frame.msgId.choice == j2735TravelerDataFrame_1_roadSignID;
        frame_msg.further_info_id = frame.msgId.choice == j2735TravelerDataFrame_1_furtherInfoID
                                        ? asn1StringToString(frame.msgId.u.furtherInfoID, true)
                                        : "";
        frame_msg.road_sign_id_present = has_road_sign;
        frame_msg.road_sign_lat = has_road_sign ? frame.msgId.u.roadSignID.position.lat : 0;
        frame_msg.road_sign_long = has_road_sign ? frame.msgId.u.roadSignID.position.Long : 0;
        frame_msg.road_sign_elevation_present = has_road_sign && frame.msgId.u.roadSignID.position.elevation_option;
        frame_msg.road_sign_elevation = (has_road_sign && frame.msgId.u.roadSignID.position.elevation_option)
                                            ? frame.msgId.u.roadSignID.position.elevation
                                            : 0;
        frame_msg.road_sign_view_angle_hex = has_road_sign ? bitStringToHex(frame.msgId.u.roadSignID.viewAngle) : "";
        frame_msg.road_sign_view_angle_bits = has_road_sign ? static_cast<uint32_t>(frame.msgId.u.roadSignID.viewAngle.len) : 0;
        frame_msg.road_sign_mutcd_code_present = has_road_sign && frame.msgId.u.roadSignID.mutcdCode_option;
        frame_msg.road_sign_mutcd_code = (has_road_sign && frame.msgId.u.roadSignID.mutcdCode_option)
                                             ? static_cast<int32_t>(frame.msgId.u.roadSignID.mutcdCode)
                                             : 0;

        frame_msg.start_year_present = frame.startYear_option;
        frame_msg.start_year = frame.startYear_option ? static_cast<uint16_t>(frame.startYear) : 0;
        frame_msg.start_time = static_cast<uint32_t>(frame.startTime);
        frame_msg.duration_time = static_cast<uint16_t>(frame.durationTime);
        frame_msg.priority = static_cast<int32_t>(frame.priority);
        frame_msg.not_used1 = static_cast<uint8_t>(frame.notUsed1);
        frame_msg.not_used2 = static_cast<uint8_t>(frame.notUsed2);
        frame_msg.not_used3 = static_cast<uint8_t>(frame.notUsed3);
        frame_msg.content_choice = static_cast<int32_t>(frame.content.choice);
        frame_msg.url_present = frame.url_option;
        frame_msg.url = asn1StringToString(frame.url, frame.url_option);

        for (size_t i = 0; i < frame.regions.count; ++i)
            frame_msg.regions.push_back(buildRegionMsg(frame.regions.tab[i], static_cast<uint32_t>(i)));

        appendFrameContent(frame_msg, frame);
        return frame_msg;
    }

    void appendRegionalExtensions(v2x_msgs::v2x_tim_total_msg& msg,
                                  const j2735TravelerInformation& tim)
    {
        if (!tim.regional_option)
            return;

        for (size_t i = 0; i < tim.regional.count; ++i)
        {
            const auto& regional = tim.regional.tab[i];
            const ASN1OpenType& value = regional.regExtValue;
            const ASN1String& raw = value.u.octet_string;
            const bool has_raw_octets = value.type == nullptr && raw.buf && raw.len > 0;

            v2x_msgs::v2x_tim_regional_msg regional_msg;
            regional_msg.regional_index = static_cast<uint32_t>(i);
            regional_msg.region_id = static_cast<int32_t>(regional.regionId);
            regional_msg.octet_string = has_raw_octets ? openTypeOctetString(value) : "";
            regional_msg.hex = has_raw_octets ? bytesToHex(reinterpret_cast<const uint8_t*>(raw.buf), raw.len) : "";
            regional_msg.length = has_raw_octets ? static_cast<uint32_t>(raw.len) : 0;
            msg.regional.push_back(regional_msg);
        }
    }

    v2x_msgs::v2x_tim_total_msg buildTimTotalMsg(const j2735TravelerInformation& tim,
                                                 const std::string& packet_id)
    {
        v2x_msgs::v2x_tim_total_msg msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "j2735_tim";
        msg.message_id = J2735_MSG_ID_TIM;
        msg.msg_cnt = static_cast<uint8_t>(tim.msgCnt);
        msg.time_stamp_present = tim.timeStamp_option;
        msg.time_stamp = tim.timeStamp_option ? static_cast<uint32_t>(tim.timeStamp) : 0;
        msg.packet_id_present = tim.packetID_option;
        msg.packet_id = packet_id;
        msg.url_b_present = tim.urlB_option;
        msg.url_b = asn1StringToString(tim.urlB, tim.urlB_option);

        for (size_t i = 0; i < tim.dataFrames.count; ++i)
            msg.data_frames.push_back(buildFrameMsg(tim.dataFrames.tab[i], static_cast<uint32_t>(i)));

        appendRegionalExtensions(msg, tim);

        PedesAssistanceData pedes_data;
        updatePedesAssistanceFromRegional(tim, pedes_data);
        msg.standard_time_present = pedes_data.standard_time_valid;
        msg.standard_time = pedes_data.standard_time_valid ? pedes_data.standard_time : "";
        msg.rsu_latitude_present = pedes_data.rsu_latitude_valid;
        msg.rsu_latitude = pedes_data.rsu_latitude_valid ? pedes_data.rsu_latitude : 0.0;
        msg.rsu_longitude_present = pedes_data.rsu_longitude_valid;
        msg.rsu_longitude = pedes_data.rsu_longitude_valid ? pedes_data.rsu_longitude : 0.0;
        msg.north_pedes_present = pedes_data.north_pedes_valid;
        msg.north_pedes = pedes_data.north_pedes_valid ? pedes_data.north_pedes : false;
        msg.east_pedes_present = pedes_data.east_pedes_valid;
        msg.east_pedes = pedes_data.east_pedes_valid ? pedes_data.east_pedes : false;
        msg.south_pedes_present = pedes_data.south_pedes_valid;
        msg.south_pedes = pedes_data.south_pedes_valid ? pedes_data.south_pedes : false;
        msg.west_pedes_present = pedes_data.west_pedes_valid;
        msg.west_pedes = pedes_data.west_pedes_valid ? pedes_data.west_pedes : false;

        return msg;
    }

    void publishTim(j2735TravelerInformation* tim,
                    ros::Publisher& tim_pub,
                    ros::Publisher& pedes_pub,
                    ros::Publisher& go_ahead_pub)
    {
        const std::string packet_id = asn1StringToString(tim->packetID, tim->packetID_option);

        tim_pub.publish(buildTimTotalMsg(*tim, packet_id));
        publishPedesAssistance(*tim, pedes_pub);
        publishGoAhead(*tim, go_ahead_pub, packet_id);

        ROS_INFO_THROTTLE(1.0, "[TIM-OBU] published TIM dataFrames=%zu regional=%zu msgCnt=%d",
                          tim->dataFrames.count,
                          tim->regional_option ? tim->regional.count : 0,
                          tim->msgCnt);
    }

    void publishPedesAssistance(const j2735TravelerInformation& tim, ros::Publisher& pedes_pub)
    {
        PedesAssistanceData data;
        updatePedesAssistanceFromRegional(tim, data);

        if (!data.hasAny())
            ROS_WARN_THROTTLE(2.0,
                              "[TIM-OBU] TIM decoded, but regional pedestrian assistance fields were not found; "
                              "publishing default false pedestrian state");
        else if (!data.north_pedes_valid || !data.east_pedes_valid ||
                 !data.south_pedes_valid || !data.west_pedes_valid)
            ROS_INFO_THROTTLE(2.0,
                              "[TIM-OBU] TIM pedestrian fields are partially missing: "
                              "N=%s E=%s S=%s W=%s; missing directions are published as false",
                              data.north_pedes_valid ? "ok" : "missing",
                              data.east_pedes_valid ? "ok" : "missing",
                              data.south_pedes_valid ? "ok" : "missing",
                              data.west_pedes_valid ? "ok" : "missing");

        v2x_msgs::v2x_pedes_assist_msg msg;
        msg.standard_time = data.standard_time_valid ? data.standard_time : "";
        msg.rsu_latitude = data.rsu_latitude_valid ? data.rsu_latitude : 0.0;
        msg.rsu_longitude = data.rsu_longitude_valid ? data.rsu_longitude : 0.0;
        msg.north_pedes = data.north_pedes_valid ? data.north_pedes : false;
        msg.east_pedes = data.east_pedes_valid ? data.east_pedes : false;
        msg.south_pedes = data.south_pedes_valid ? data.south_pedes : false;
        msg.west_pedes = data.west_pedes_valid ? data.west_pedes : false;
        pedes_pub.publish(msg);

        ROS_INFO_THROTTLE(1.0, "[TIM-OBU] published pedestrian assistance from TIM regional extensions");
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

#include "siheung_v2x/tim_publish.h"

#include "asn1defs.h"

#include <v2x_msgs/v2x_pedes_assist_msg.h>
#include <v2x_msgs/v2x_tim_can_go_msg.h>
#include <v2x_msgs/v2x_tim_content_msg.h>
#include <v2x_msgs/v2x_tim_dataframe_msg.h>
#include <v2x_msgs/v2x_tim_region_msg.h>
#include <v2x_msgs/v2x_tim_regional_msg.h>
#include <v2x_msgs/v2x_tim_total_msg.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#define J2735_MSG_ID_TIM 31

namespace tim_publish {
namespace {

// ── ASN.1 / 문자열 헬퍼 ───────────────────────────────────────────────
std::string asn1StringToString(const ASN1String& s, bool present)
{
    if (!present || !s.buf || s.len == 0)
        return "";
    return std::string(reinterpret_cast<const char*>(s.buf), s.len);
}

std::string bytesToHex(const uint8_t* data, size_t len)
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

std::string bitStringToHex(const ASN1BitString& value)
{
    const size_t byte_len = (value.len + 7) / 8;
    if (!value.buf || byte_len == 0)
        return "";
    return bytesToHex(value.buf, byte_len);
}

std::string jsonEscape(const std::string& input)
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

std::string openTypeOctetString(const ASN1OpenType& value)
{
    if (value.type != nullptr || !value.u.octet_string.buf || value.u.octet_string.len == 0)
        return "";

    return std::string(reinterpret_cast<const char*>(value.u.octet_string.buf),
                       value.u.octet_string.len);
}

std::string extractJsonObject(const std::string& raw)
{
    const size_t begin = raw.find('{');
    const size_t end = raw.rfind('}');
    if (begin == std::string::npos || end == std::string::npos || begin > end)
        return "";
    return raw.substr(begin, end - begin + 1);
}

std::string unescapeJsonText(const std::string& input)
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

std::vector<std::string> jsonCandidatesFromOpenType(const ASN1OpenType& value)
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

bool findJsonStringValue(const std::string& json, const std::string& key, std::string& value)
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

bool findJsonNumberValue(const std::string& json, const std::string& key, double& value)
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

bool findJsonBoolValue(const std::string& json, const std::string& key, bool& value)
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

bool findJsonObjectValue(const std::string& json, const std::string& key, std::string& value)
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

// ── regional 확장 JSON 에서 보행자/표준시각/RSU 위치 파싱 ──────────────
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
};

void updatePedesAssistanceFromRegional(const j2735TravelerInformation& tim,
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
            ROS_DEBUG("[TIM] regional[%zu] regionId=%d has no JSON object, raw_prefix=%s",
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

// ── TIM 구조체 → v2x_tim_total_msg 빌더 ───────────────────────────────
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
                      "[TIM] published do_not_go_forward=true packetID=%s standard_time=%s rsu=(%.9f, %.9f)",
                      packet_id.c_str(),
                      msg.standard_time.c_str(),
                      msg.rsu_latitude,
                      msg.rsu_longitude);
}

void publishPedesAssistance(const j2735TravelerInformation& tim, ros::Publisher& pedes_pub)
{
    PedesAssistanceData data;
    updatePedesAssistanceFromRegional(tim, data);

    if (!data.hasAny())
        ROS_WARN_THROTTLE(2.0,
                          "[TIM] TIM decoded, but regional pedestrian assistance fields were not found; "
                          "publishing default false pedestrian state");
    else if (!data.north_pedes_valid || !data.east_pedes_valid ||
             !data.south_pedes_valid || !data.west_pedes_valid)
        ROS_INFO_THROTTLE(2.0,
                          "[TIM] TIM pedestrian fields are partially missing: "
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

    // ── /obu/v2x_pedes_assistance 발행 전 수신 데이터 로그 저장 ──────────
    {
        const char* home = std::getenv("HOME");
        std::string log_path =
            std::string(home ? home : ".") + "/v2x_pedes_assistance.log";
        std::ofstream ofs(log_path, std::ios::app);
        if (ofs)
        {
            ros::WallTime now = ros::WallTime::now();
            ofs << std::fixed << std::setprecision(3) << now.toSec()
                << " has_any=" << (data.hasAny() ? 1 : 0)
                << " std_time=\"" << msg.standard_time << "\""
                << " rsu_lat=" << std::setprecision(7) << msg.rsu_latitude
                << " rsu_lon=" << msg.rsu_longitude
                << " N=" << (msg.north_pedes ? 1 : 0)
                << "(" << (data.north_pedes_valid ? "ok" : "miss") << ")"
                << " E=" << (msg.east_pedes ? 1 : 0)
                << "(" << (data.east_pedes_valid ? "ok" : "miss") << ")"
                << " S=" << (msg.south_pedes ? 1 : 0)
                << "(" << (data.south_pedes_valid ? "ok" : "miss") << ")"
                << " W=" << (msg.west_pedes ? 1 : 0)
                << "(" << (data.west_pedes_valid ? "ok" : "miss") << ")"
                << "\n";
        }
    }

    pedes_pub.publish(msg);

    ROS_INFO_THROTTLE(1.0, "[TIM] published pedestrian assistance from TIM regional extensions");
}

}  // namespace

void publish(const j2735TravelerInformation* tim,
             ros::Publisher& tim_pub,
             ros::Publisher& pedes_pub,
             ros::Publisher& go_ahead_pub)
{
    const std::string packet_id = asn1StringToString(tim->packetID, tim->packetID_option);

    tim_pub.publish(buildTimTotalMsg(*tim, packet_id));
    publishPedesAssistance(*tim, pedes_pub);
    publishGoAhead(*tim, go_ahead_pub, packet_id);

    ROS_INFO_THROTTLE(1.0, "[TIM] published TIM dataFrames=%zu regional=%zu msgCnt=%d",
                      tim->dataFrames.count,
                      tim->regional_option ? tim->regional.count : 0,
                      tim->msgCnt);
}

}  // namespace tim_publish

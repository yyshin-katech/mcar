// mqtt_spat_rx_node.cpp
// MQTT broker (V2N) 의 SPaT 토픽을 구독해서 J2735 SPaT 을 디코드하고
// v2x_msgs/intersection_array_msg 로 발행한다.
//
// 사양서: claude_work_list/mqtt_spat_rx.md (2026-05-22, siheung_dev)
// 방안 A: src/v2x/siheung_v2x/src/j2735_decode.cpp 의 publishSpat / skipWsmpHeader /
//         g_link_info / toControlTeamCallback 를 이 파일 안에 복제(file-local namespace).
//         j2735_decode.cpp 와 bsm_tx_node.cpp 는 일절 수정하지 않는다.

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <vector>
#include <string>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <map>

extern "C" {
#include <mosquitto.h>
}

#include "ffasn1-j2735-2026-KSR1600.h"
#include "asn1defs.h"

#include <v2x_msgs/intersection_msg.h>
#include <v2x_msgs/intersection_array_msg.h>
#include <mmc_msgs/to_control_team_from_local_msg.h>

// ──────────────────────────────────────────────────────────────────────────────
// 상수
// ──────────────────────────────────────────────────────────────────────────────
namespace {
// J2735 SPaT messageId = 19 (0x13)
constexpr int J2735_MSG_ID_SPAT = 19;
}

// ──────────────────────────────────────────────────────────────────────────────
// 사양서 §1.2 방안 A: j2735_decode.cpp 에서 복제
//   - struct ReceivedMsg
//   - class MessageQueue
//   - struct CurrentLinkInfo / g_link_info
//   - void toControlTeamCallback(...)
//   - class SpatDecoder { skipWsmpHeader, publishSpat }
//
// 이 노드는 별도 process 이므로 j2735_decode.cpp 의 전역과 공유되지 않는다.
// 따라서 동일 이름·동일 동작을 file-local 익명 namespace 에 두고 사용한다.
// ──────────────────────────────────────────────────────────────────────────────
namespace {

struct ReceivedMsg {
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
        if (cv_.wait_for(lock, std::chrono::milliseconds(10),
                         [&] { return !queue_.empty(); }))
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

// ── OBU(WAVE) SPaT 중복 제거 캐시 ──────────────────────────────────────────
// siheung_v2x j2735_decode 가 발행하는 /siheung_spat (OBU/WAVE 경유) 의
// intersection_id 별 최근 수신 시각을 기록. MQTT 로 들어온 같은 intersection_id
// 가 timeout 안에 OBU 캐시에도 존재하면 본 노드의 publish 는 skip — 동일
// 신호등 정보는 OBU 경로(/siheung_spat) 만 to_control_team 으로 전달되도록 함.
struct ObuSpatCache {
    std::mutex mtx;
    std::map<int, ros::Time> last_seen;  // intersection_id → 마지막 OBU 수신 시각
};
ObuSpatCache g_obu_cache;
double g_obu_dedup_timeout = 2.0;  // 초; MqttSpatRxNode 생성자에서 파라미터로 갱신

void obuSpatCallback(const v2x_msgs::intersection_array_msg::ConstPtr& msg)
{
    ros::Time now = ros::Time::now();
    std::lock_guard<std::mutex> lock(g_obu_cache.mtx);
    for (const auto& d : msg->data)
    {
        if (d.IntersectionID != 0)
            g_obu_cache.last_seen[(int)d.IntersectionID] = now;
    }
}

bool obuHasRecent(int iid)
{
    if (iid == 0) return false;
    std::lock_guard<std::mutex> lock(g_obu_cache.mtx);
    auto it = g_obu_cache.last_seen.find(iid);
    if (it == g_obu_cache.last_seen.end()) return false;
    return (ros::Time::now() - it->second).toSec() < g_obu_dedup_timeout;
}

class SpatDecoder {
public:
    // WSMP 서브헤더 건너뛰기 (03 80 + BER 길이)
    // MQTT 페이로드에는 보통 없지만, 안전망으로 그대로 사용.
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

    // V2N MQTT payload 앞단의 V2N container 16-byte fixed header 스킵.
    // 경찰청 V2N 정보연계 규격 ITSK-00150-2 <표 4-11> "단일 메시지 전송을
    // 위한 V2N Container" (container_type=0x04, sem_length=0) 정합 layout:
    //   offset 0    : container_type (8 bit)  = 0x04 (단일 메시지 전송, <표 4-3>)
    //   offset 1    : version        (8 bit)  = 0x00
    //   offset 2-3  : fid            (16 bit BE) = 0xFF11 (<표 4-4> 외부 표)
    //   offset 4    : standard_type  (8 bit)  = 0x01 (KS 표준, <표 4-5>)
    //   offset 5    : sem_length     (8 bit)  = 0x00 (SEM 없음)
    //   offset 6    : flags          (8 bit)  = 0x00
    //   offset 7    : message_id     (8 bit)  = container-level 메시지 식별자(시퀀스)
    //   offset 8-11 : psid           (32 bit BE) = 0x00014085 (<표 4-6> 외부 표)
    //     ※ ~/protocol/경4 V2N FID PSID 정의안 (1).xlsx PSID 시트는 SPaT 행에서
    //        decimal=82055 / hex=0x014085 로 두 값이 내부 모순됨.
    //        0x014085 = decimal 82053 이며 BSM 등 다른 행은 정합. 운영 브로커가
    //        실제 발행하는 raw 헤더가 0x00014085 (라이브 캡처 2026-05-23) 이므로
    //        본 노드는 hex 값을 단일 출처로 사용 (BSM TX psid 와 동일 정책).
    //   offset 12-15: message_length (32 bit BE) = 이후 UPER 메시지 길이
    //   offset 16.. : message               = J2735 UPER MessageFrame (00 13 ... = SPaT)
    static size_t skipMqttV2nHeader(const uint8_t* body, size_t body_len)
    {
        if (body_len < 16)
            return 0;
        // container_type=0x04, version=0x00, fid=0xFF11 prefix check
        if (body[0] != 0x04 || body[1] != 0x00 ||
            body[2] != 0xff || body[3] != 0x11)
            return 0;

        // message_length (32 bit BE) at offset 12..15
        uint32_t inner_len = (uint32_t)body[12] << 24 |
                             (uint32_t)body[13] << 16 |
                             (uint32_t)body[14] <<  8 |
                             (uint32_t)body[15];
        if (inner_len == 0 || inner_len > body_len - 16)
            return 0;  // 길이 필드가 비정상이면 안전하게 skip 안 함

        return 16;
    }

    // SPaT 구조체 → ROS 메시지 변환 및 발행
    // to_control_team의 intersection_id, signalGroupID, MANUAVER 기반으로
    // 현재 링크에 필요한 신호만 필터링하여 발행
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

        // MANUAVER → 타겟 movementName 매핑 (실제 브로커 데이터 기준)
        // -1=LEFT, 0=STR(직진), 1=RIGHT
        // 시흥 운영 브로커가 발행하는 SPaT movementName 은 "STR"/"LEFT"/"RIGHT"
        // ("STRAIGHT" 가 아님 — 2026-05-23 라이브 캡처로 확인)
        std::string target_name;
        if (cur_manuaver == -1)
            target_name = "LEFT";
        else if (cur_manuaver == 1)
            target_name = "RIGHT";
        else
            target_name = "STR";

        // (verify) 첫 디코딩 성공 시 SPaT 안 intersection 목록을 1회 dump
        if (spat && spat->intersections.count > 0)
        {
            std::string ids;
            for (size_t i = 0; i < spat->intersections.count && i < 8; ++i)
            {
                char buf[16];
                std::snprintf(buf, sizeof(buf), "%s%d",
                              i == 0 ? "" : ",", spat->intersections.tab[i].id.id);
                ids += buf;
            }
            ROS_INFO_ONCE("[mqtt_spat_rx] decoded SPaT: %zu intersection(s) iid=[%s]",
                          (size_t)spat->intersections.count, ids.c_str());
        }

        v2x_msgs::intersection_array_msg spat_msg;
        spat_msg.time = ros::Time::now();

        // 신호 불필요 링크: 모든 필드 0인 빈 메시지 발행
        if (cur_intersection_id == 0)
        {
            v2x_msgs::intersection_msg zero_msg;
            spat_msg.data.push_back(zero_msg);
            spat_pub.publish(spat_msg);
            return;
        }

        // OBU(WAVE) 가 동일 intersection_id 를 최근에 발행했으면 본 MQTT 노드는 skip.
        // 동일 신호등 정보가 OBU/MQTT 양쪽에서 동시에 들어오는 상황을 가정하고,
        // OBU 경로(/siheung_spat) 만 to_control_team 흐름으로 전달되도록 한다.
        if (obuHasRecent(cur_intersection_id))
        {
            ROS_DEBUG_THROTTLE(2.0,
                "[mqtt_spat_rx] iid=%d covered by OBU SPaT — skipping MQTT publish",
                cur_intersection_id);
            return;
        }

        // SPaT-level timeStamp (intersection.moy 가 없는 경우의 fallback).
        // j2735SPAT.timeStamp 는 MinuteOfTheYear (optional). 표준 정합.
        const bool   spat_has_moy = (spat && spat->timeStamp_option);
        const int32_t spat_moy    = spat_has_moy ? (int32_t)spat->timeStamp : 0;

        for (size_t i = 0; i < spat->intersections.count; ++i)
        {
            auto& intersection = spat->intersections.tab[i];
            int iid = intersection.id.id;

            // intersection_id 필터
            if (iid != cur_intersection_id)
                continue;

            for (size_t j = 0; j < intersection.states.count; ++j)
            {
                auto& movement = intersection.states.tab[j];

                // signalGroupID 필터
                if (cur_signal_group != 0 &&
                    (int)movement.signalGroup != cur_signal_group)
                    continue;

                // movementName 추출
                std::string move_name;
                if (movement.movementName_option && movement.movementName.buf)
                    move_name = std::string((char*)movement.movementName.buf,
                                            movement.movementName.len);

                // MANUAVER 방향 필터
                if (!move_name.empty() && move_name != target_name)
                    continue;

                v2x_msgs::intersection_msg int_msg;
                int_msg.IntersectionID = iid;
                int_msg.RoadRegulatorID = intersection.id.region_option ? intersection.id.region : 0;
                int_msg.MsgCount = intersection.revision;
                int_msg.NumOfMovementState = intersection.states.count;
                int_msg.Movements.MovementStateName = move_name;
                int_msg.Movements.SignalGroupID = movement.signalGroup;

                // IntersectionStatusObject (J2735 BIT STRING SIZE(16), MSB first) → bool[16]
                // ASN1BitString.len 은 bit 단위. buf 는 ceil(len/8) byte.
                {
                    const ASN1BitString& bs = intersection.status;
                    size_t bits = bs.len;
                    if (bits > 16) bits = 16;
                    for (size_t k = 0; k < bits && bs.buf; ++k)
                    {
                        uint8_t byte = bs.buf[k / 8];
                        int_msg.IntersectionStatusObject[k] =
                            ((byte >> (7 - (k % 8))) & 0x01) ? true : false;
                    }
                }

                if (movement.state_time_speed.count > 0)
                {
                    auto& evt = movement.state_time_speed.tab[0];
                    int_msg.Movements.MovementPhaseStatus = (uint8_t)evt.eventState;

                    if (evt.timing_option)
                        int_msg.Movements.TimeChangeDetails = evt.timing.minEndTime;
                }

                // MinuteOfTheYear : intersection.moy 우선, 없으면 SPaT-level timeStamp fallback
                if (intersection.moy_option)
                    int_msg.MinuteOfTheYear = intersection.moy;
                else if (spat_has_moy)
                    int_msg.MinuteOfTheYear = spat_moy;
                if (intersection.timeStamp_option)
                    int_msg.DSecond = intersection.timeStamp;

                spat_msg.data.push_back(int_msg);
            }
        }

        spat_pub.publish(spat_msg);

        // 매칭 결과 로그
        if (!spat_msg.data.empty())
        {
            auto& d = spat_msg.data[0];
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
            ROS_INFO("[SPaT] IntID=%d SigGrp=%d Move=%s Phase=%s minEnd=%.1fs",
                     cur_intersection_id, cur_signal_group,
                     target_name.c_str(), phase,
                     d.Movements.TimeChangeDetails / 10.0);
        }
        else
        {
            ROS_DEBUG("[SPaT] IntID=%d SigGrp=%d Move=%s -> 매칭 없음",
                      cur_intersection_id, cur_signal_group, target_name.c_str());
        }
    }
};

}  // namespace

// ──────────────────────────────────────────────────────────────────────────────
// MqttSpatRxNode (사양서 §3)
// ──────────────────────────────────────────────────────────────────────────────
class MqttSpatRxNode {
public:
    MqttSpatRxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~MqttSpatRxNode();

    // queue 소비 + ros::spinOnce 루프
    void spin();

private:
    // ROS
    ros::Publisher  spat_pub_;
    ros::Subscriber ctrl_sub_;
    ros::Subscriber obu_sub_;   // /siheung_spat (OBU SPaT) — dedup 용

    // MQTT (mosquitto C handle)
    struct mosquitto* mosq_;
    std::string broker_host_;
    std::string username_;
    std::string password_;
    std::string topic_;
    std::string client_id_;
    std::string spat_topic_out_;
    int broker_port_;
    int keepalive_;
    int qos_;
    int reconnect_min_;
    int reconnect_max_;
    bool verbose_first_msg_;

    // 페이로드 큐
    MessageQueue queue_;

    // SPaT decoder
    SpatDecoder decoder_;

    bool mosq_started_;
    bool first_msg_;

    // mosquitto static callbacks (C API → C++ instance)
    static void onConnect   (struct mosquitto*, void* userdata, int rc);
    static void onMessage   (struct mosquitto*, void* userdata,
                             const struct mosquitto_message* msg);
    static void onDisconnect(struct mosquitto*, void* userdata, int rc);

    // 인스턴스 메서드
    void handleConnect   (int rc);
    void handleMessage   (const struct mosquitto_message* msg);
    void handleDisconnect(int rc);
    void decodePayload   (const std::vector<uint8_t>& buf);
};

// ──────────────────────────────────────────────────────────────────────────────
// 생성자/소멸자 (사양서 §3.2)
// ──────────────────────────────────────────────────────────────────────────────
MqttSpatRxNode::MqttSpatRxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : mosq_(nullptr),
      broker_port_(10044),
      keepalive_(60),
      qos_(0),
      reconnect_min_(1),
      reconnect_max_(30),
      verbose_first_msg_(true),
      mosq_started_(false),
      first_msg_(true)
{
    // 1. 파라미터 로드
    pnh.param<std::string>("broker_host", broker_host_, "192.168.255.173");
    pnh.param<int>        ("broker_port", broker_port_, 10044);
    pnh.param<std::string>("username",    username_,    "xcms-mtqq");
    pnh.param<std::string>("password",    password_,    "xcms123!");
    pnh.param<std::string>("topic",       topic_,       "V2N/1321103202/trf_drct/spat");
    pnh.param<std::string>("client_id",   client_id_,   "");
    pnh.param<int>        ("keepalive",   keepalive_,   60);
    pnh.param<int>        ("qos",         qos_,         0);
    pnh.param<std::string>("spat_topic",  spat_topic_out_, "/siheung_v2x/mqtt_spat");
    pnh.param<int>        ("reconnect_min", reconnect_min_, 1);
    pnh.param<int>        ("reconnect_max", reconnect_max_, 30);
    pnh.param<bool>       ("verbose_first_msg", verbose_first_msg_, true);

    // OBU SPaT (siheung_v2x j2735_decode 발행) 와 중복 신호등 제거용 파라미터
    std::string obu_spat_topic;
    pnh.param<std::string>("obu_spat_topic",   obu_spat_topic,        "/siheung_spat");
    pnh.param<double>     ("obu_dedup_timeout", g_obu_dedup_timeout,  2.0);

    // 2. ROS publisher / subscriber
    spat_pub_ = nh.advertise<v2x_msgs::intersection_array_msg>(spat_topic_out_, 1);
    ctrl_sub_ = nh.subscribe("/localization/to_control_team", 1, toControlTeamCallback);
    obu_sub_  = nh.subscribe(obu_spat_topic, 10, obuSpatCallback);

    ROS_INFO("[mqtt_spat_rx] started - broker=%s:%d topic=%s out=%s",
             broker_host_.c_str(), broker_port_, topic_.c_str(), spat_topic_out_.c_str());
    ROS_INFO("[mqtt_spat_rx] OBU dedup: subscribed to %s (timeout=%.1fs)",
             obu_spat_topic.c_str(), g_obu_dedup_timeout);

    // 4. mosquitto 초기화
    mosquitto_lib_init();

    // 5. mosq_ 생성. client_id 가 빈 문자열이면 nullptr 로 (broker 가 random id 할당)
    const char* client_id_arg = client_id_.empty() ? nullptr : client_id_.c_str();
    mosq_ = mosquitto_new(client_id_arg, /*clean_session=*/true, this);
    if (!mosq_)
    {
        ROS_ERROR("[mqtt_spat_rx] mosquitto_new failed: %s", strerror(errno));
        return;
    }

    // 6. 인증
    if (!username_.empty())
    {
        const char* pw_arg = password_.empty() ? nullptr : password_.c_str();
        int u_rc = mosquitto_username_pw_set(mosq_, username_.c_str(), pw_arg);
        if (u_rc != MOSQ_ERR_SUCCESS)
            ROS_WARN("[mqtt_spat_rx] mosquitto_username_pw_set rc=%d (%s)",
                     u_rc, mosquitto_strerror(u_rc));
    }

    // 7-9. 콜백 등록
    mosquitto_connect_callback_set(mosq_,    &MqttSpatRxNode::onConnect);
    mosquitto_message_callback_set(mosq_,    &MqttSpatRxNode::onMessage);
    mosquitto_disconnect_callback_set(mosq_, &MqttSpatRxNode::onDisconnect);

    // 사양서 §3.7: 재접속 지연 (1~30s, exponential backoff)
    mosquitto_reconnect_delay_set(mosq_, reconnect_min_, reconnect_max_, true);

    // 10. broker 접속
    int rc = mosquitto_connect(mosq_, broker_host_.c_str(), broker_port_, keepalive_);
    if (rc != MOSQ_ERR_SUCCESS)
    {
        ROS_ERROR("[mqtt_spat_rx] mosquitto_connect rc=%d (%s) - will retry via loop",
                  rc, mosquitto_strerror(rc));
        // 즉시 shutdown 하지 않음. loop_start 가 reconnect 처리.
    }

    // 11. 백그라운드 네트워크 스레드 시작
    int ls_rc = mosquitto_loop_start(mosq_);
    if (ls_rc != MOSQ_ERR_SUCCESS)
    {
        ROS_ERROR("[mqtt_spat_rx] mosquitto_loop_start rc=%d (%s)",
                  ls_rc, mosquitto_strerror(ls_rc));
    }
    else
    {
        mosq_started_ = true;
    }
}

MqttSpatRxNode::~MqttSpatRxNode()
{
    if (mosq_)
    {
        mosquitto_disconnect(mosq_);
        if (mosq_started_)
            mosquitto_loop_stop(mosq_, /*force=*/false);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
    }
    mosquitto_lib_cleanup();
}

// ──────────────────────────────────────────────────────────────────────────────
// mosquitto 정적 콜백 → 인스턴스 dispatch
// ──────────────────────────────────────────────────────────────────────────────
void MqttSpatRxNode::onConnect(struct mosquitto*, void* userdata, int rc)
{
    auto* self = static_cast<MqttSpatRxNode*>(userdata);
    if (self) self->handleConnect(rc);
}

void MqttSpatRxNode::onMessage(struct mosquitto*, void* userdata,
                               const struct mosquitto_message* msg)
{
    auto* self = static_cast<MqttSpatRxNode*>(userdata);
    if (self) self->handleMessage(msg);
}

void MqttSpatRxNode::onDisconnect(struct mosquitto*, void* userdata, int rc)
{
    auto* self = static_cast<MqttSpatRxNode*>(userdata);
    if (self) self->handleDisconnect(rc);
}

// ──────────────────────────────────────────────────────────────────────────────
// 콜백 흐름 (사양서 §3.3)
// ──────────────────────────────────────────────────────────────────────────────
void MqttSpatRxNode::handleConnect(int rc)
{
    ROS_INFO("[mqtt_spat_rx] connected to broker (rc=%d)", rc);
    if (rc == 0)
    {
        int s_rc = mosquitto_subscribe(mosq_, nullptr, topic_.c_str(), qos_);
        if (s_rc == MOSQ_ERR_SUCCESS)
        {
            ROS_INFO("[mqtt_spat_rx] subscribed to %s qos=%d", topic_.c_str(), qos_);
        }
        else
        {
            ROS_ERROR("[mqtt_spat_rx] mosquitto_subscribe rc=%d (%s)",
                      s_rc, mosquitto_strerror(s_rc));
        }
    }
    else
    {
        ROS_ERROR("[mqtt_spat_rx] connect failed rc=%d (%s)",
                  rc, mosquitto_connack_string(rc));
    }
}

void MqttSpatRxNode::handleMessage(const struct mosquitto_message* msg)
{
    if (!msg || !msg->payload || msg->payloadlen <= 0)
        return;

    // mosquitto 스레드에서 ROS publisher 를 직접 호출하지 않는다 (사양서 §3.3, §3.6)
    // → 큐에 넣고 main 스레드의 decodePayload 가 처리.
    const uint8_t* p = static_cast<const uint8_t*>(msg->payload);
    ReceivedMsg rmsg;
    rmsg.data.assign(p, p + msg->payloadlen);
    rmsg.len = static_cast<size_t>(msg->payloadlen);
    queue_.push(rmsg);
}

void MqttSpatRxNode::handleDisconnect(int rc)
{
    if (rc != 0)
        ROS_WARN_THROTTLE(10.0, "[mqtt_spat_rx] disconnected rc=%d (%s) - auto-reconnect",
                          rc, mosquitto_strerror(rc));
    else
        ROS_INFO("[mqtt_spat_rx] disconnected cleanly");
}

// ──────────────────────────────────────────────────────────────────────────────
// 메인 루프 (사양서 §3.4)
// ──────────────────────────────────────────────────────────────────────────────
void MqttSpatRxNode::spin()
{
    ros::Rate loop_rate(1000);
    ReceivedMsg msg;
    while (ros::ok())
    {
        if (queue_.pop(msg))
            decodePayload(msg.data);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// decodePayload (사양서 §3.5)
//   - WSMP 헤더 스킵 (있다면)
//   - MessageFrame UPER 시도 → messageId==19 면 SPaT publishSpat()
//   - 실패하면 SPaT UPER 직접 디코드 fallback
//   - 첫 메시지 hex dump (사양서 §2.2 / §3.9)
// ──────────────────────────────────────────────────────────────────────────────
void MqttSpatRxNode::decodePayload(const std::vector<uint8_t>& buf)
{
    if (buf.size() < 4)
    {
        ROS_WARN_THROTTLE(5.0, "[mqtt_spat_rx] payload too short (len=%zu)", buf.size());
        return;
    }

    // 첫 메시지 sanity check (사양서 §2.2, §3.9)
    if (first_msg_)
    {
        first_msg_ = false;
        if (verbose_first_msg_)
        {
            size_t n = buf.size();
            uint8_t b0 = n > 0 ? buf[0] : 0;
            uint8_t b1 = n > 1 ? buf[1] : 0;
            uint8_t b2 = n > 2 ? buf[2] : 0;
            uint8_t b3 = n > 3 ? buf[3] : 0;
            ROS_INFO("[mqtt_spat_rx] first payload len=%d, head=%02x %02x %02x %02x ...",
                     static_cast<int>(n), b0, b1, b2, b3);

            // head 16 bytes
            char hex[3 * 16 + 1] = {0};
            size_t dump_n = std::min<size_t>(16, n);
            for (size_t i = 0; i < dump_n; ++i)
                std::snprintf(hex + i * 3, sizeof(hex) - i * 3, "%02x ", buf[i]);
            ROS_INFO("[mqtt_spat_rx] first payload hex[0..%zu]: %s",
                     dump_n, hex);
        }
    }

    const uint8_t* body = buf.data();
    size_t body_len = buf.size();

    // (선택) V2N MQTT 커스텀 16-byte 헤더 스킵 (magic 04 00 ff 11)
    size_t v2n_off = SpatDecoder::skipMqttV2nHeader(body, body_len);
    if (v2n_off > 0)
    {
        ROS_DEBUG("[mqtt_spat_rx] V2N hdr skipped (%zu B)", v2n_off);
        body     += v2n_off;
        body_len -= v2n_off;
    }

    // (선택) WSMP 헤더 스킵
    size_t wsmp_off = SpatDecoder::skipWsmpHeader(body, body_len);
    if (wsmp_off > 0)
    {
        ROS_DEBUG("[mqtt_spat_rx] WSMP skipped (%zu B)", wsmp_off);
        body     += wsmp_off;
        body_len -= wsmp_off;
    }

    // 1차: MessageFrame UPER 디코드 시도
    ASN1Error err;
    void* frame_msg = nullptr;
    asn1_ssize_t ret = asn1_uper_decode(&frame_msg, asn1_type_j2735MessageFrame,
                                        body, body_len, &err);
    if (ret > 0 && frame_msg)
    {
        j2735MessageFrame* frame = (j2735MessageFrame*)frame_msg;

        if (frame->messageId == J2735_MSG_ID_SPAT)
        {
            j2735SPAT* spat = nullptr;
            void* spat_standalone = nullptr;

            if (frame->value.type != nullptr)
            {
                spat = (j2735SPAT*)frame->value.u.data;
            }
            else
            {
                ASN1String* raw = &frame->value.u.octet_string;
                asn1_ssize_t r2 = asn1_uper_decode(&spat_standalone, asn1_type_j2735SPAT,
                                                   raw->buf, raw->len, &err);
                if (r2 > 0 && spat_standalone)
                    spat = (j2735SPAT*)spat_standalone;
            }

            if (spat)
                decoder_.publishSpat(spat, spat_pub_);

            if (spat_standalone)
                asn1_free_value(asn1_type_j2735SPAT, spat_standalone);
            asn1_free_value(asn1_type_j2735MessageFrame, frame_msg);
            return;
        }

        ROS_DEBUG_THROTTLE(5.0,
                           "[mqtt_spat_rx] MessageFrame but not SPaT (msgId=%d)",
                           (int)frame->messageId);
        asn1_free_value(asn1_type_j2735MessageFrame, frame_msg);
    }

    // 2차 fallback: SPaT UPER 직접 디코드
    void* spat_msg = nullptr;
    asn1_ssize_t r3 = asn1_uper_decode(&spat_msg, asn1_type_j2735SPAT,
                                       body, body_len, &err);
    if (r3 > 0 && spat_msg)
    {
        decoder_.publishSpat((j2735SPAT*)spat_msg, spat_pub_);
        asn1_free_value(asn1_type_j2735SPAT, spat_msg);
        return;
    }

    ROS_WARN_THROTTLE(5.0, "[mqtt_spat_rx] failed to decode payload (len=%zu)", body_len);
}

// ──────────────────────────────────────────────────────────────────────────────
// main
// ──────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mqtt_spat_rx_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    MqttSpatRxNode node(nh, pnh);
    node.spin();
    return 0;
}

// =============================================================================
// mqtt_bsm_tx_node.cpp
//
// J2735 BasicSafetyMessage (BSM) MQTT 송신 노드.
// - Inspva (/sensors/gps/inspva, novatel_gps_msgs) + v_can (/sensors/v_can) 를
//   캐시하여 10Hz 로 BSM 을 채우고 MessageFrame 으로 감싸 UPER 인코딩.
// - 경찰청 V2N 정보연계 규격 ITSK-00150-2 <표 4-11> "단일 메시지 전송을
//   위한 V2N Container" (container_type=0x04, sem_length=0) 의 16-byte fixed
//   header prepend 후 MQTT broker 의 BSM 토픽 (V2N/1321103202/bsm) 으로 publish.
//
// 사양서: claude_work_list/mqtt_bsm_tx.md (2026-05-23, siheung_dev)
//   - BSM 인코딩 로직은 bsm_tx_node.cpp 에서 file-local 복제.
//   - mosquitto 패턴은 mqtt_spat_rx_node.cpp 에서 file-local 복제.
//   - 두 원본 파일은 일절 수정하지 않는다.
// =============================================================================

#include <ros/ros.h>
#include <novatel_gps_msgs/Inspva.h>
#include <katech_custom_msgs/v_can_msg.h>

#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <mutex>
#include <string>
#include <vector>

extern "C" {
#include <mosquitto.h>
}

// 헤더 자체가 extern "C" 블록을 포함하므로 직접 include 가능.
#include "ffasn1-j2735-2026-KSR1600.h"
#include "asn1defs.h"

// J2735 BSM messageId
#define J2735_MSG_ID_BSM 20  // 0x14

namespace {

// ---------------------------------------------------------------------------
// 유틸: C++14 호환 clamp (bsm_tx_node.cpp 복제)
// ---------------------------------------------------------------------------
inline int clampi(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ---------------------------------------------------------------------------
// vehicle_id 파라미터 파싱 ("AABBCCDD" 또는 "0xAABBCCDD")
// 성공 시 true, 실패 시 false 반환 (호출자가 default 유지).
// (bsm_tx_node.cpp 복제)
// ---------------------------------------------------------------------------
bool parseVehicleIdHex(const std::string& in, uint8_t out[4]) {
  std::string s = in;
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.erase(s.begin());
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) s.pop_back();
  if (s.size() >= 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
    s = s.substr(2);
  }
  if (s.size() != 8) return false;
  for (char c : s) {
    bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
    if (!ok) return false;
  }
  for (int i = 0; i < 4; ++i) {
    unsigned int byte_val = 0;
    if (std::sscanf(s.substr(i * 2, 2).c_str(), "%x", &byte_val) != 1) return false;
    out[i] = static_cast<uint8_t>(byte_val & 0xFF);
  }
  return true;
}

// ASCII fallback: 첫 4바이트만 사용, 짧으면 0x00 padding (bsm_tx_node.cpp 복제)
void parseVehicleIdAscii(const std::string& in, uint8_t out[4]) {
  std::memset(out, 0, 4);
  const size_t n = std::min(in.size(), static_cast<size_t>(4));
  std::memcpy(out, in.data(), n);
}

}  // namespace

// ---------------------------------------------------------------------------
// MqttBsmTxNode
// ---------------------------------------------------------------------------
class MqttBsmTxNode {
 public:
  MqttBsmTxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~MqttBsmTxNode();

 private:
  // ── ROS ─────────────────────────────────────────────
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_inspva_;
  ros::Subscriber sub_vcan_;
  ros::Timer      timer_;

  // ── 최신 입력 캐시 ──────────────────────────────────
  std::mutex                       mtx_input_;
  novatel_gps_msgs::Inspva         latest_inspva_;
  bool                             has_inspva_ = false;
  katech_custom_msgs::v_can_msg    latest_vcan_;
  bool                             has_vcan_ = false;

  // ── BSM 카운터 (UPER 안의 msgCnt, 0..127 wrap) ─────
  uint8_t bsm_msg_cnt_ = 0;        // mtx_input_ 보호

  // ── V2N 헤더 시퀀스 (offset 7, 0..255 wrap) ────────
  std::atomic<uint8_t> seq_{0};

  // ── 차량 파라미터 ───────────────────────────────────
  uint8_t  vehicle_id_[4]    = {0x00, 0x00, 0x00, 0x01};
  int      vehicle_width_cm_ = 190;
  int      vehicle_length_cm_= 464;

  // ── 입력 토픽 / publish 주기 ────────────────────────
  std::string inspva_topic_ = "/sensors/gps/inspva";
  std::string vcan_topic_   = "/sensors/v_can";
  double      publish_rate_ = 10.0;

  // ── MQTT 파라미터 + handle ─────────────────────────
  struct mosquitto* mosq_ = nullptr;
  std::string broker_host_;
  std::string username_;
  std::string password_;
  std::string topic_;
  std::string client_id_;
  int  broker_port_   = 23312;
  int  keepalive_     = 60;
  int  qos_           = 0;
  bool retain_        = false;
  int  reconnect_min_ = 1;
  int  reconnect_max_ = 30;
  bool verbose_first_msg_ = true;
  bool mosq_started_  = false;
  std::atomic<bool> first_publish_{true};

  // ── 콜백 ───────────────────────────────────────────
  void onInspva(const novatel_gps_msgs::Inspva::ConstPtr& msg);
  void onVCan  (const katech_custom_msgs::v_can_msg::ConstPtr& msg);
  void onTimer (const ros::TimerEvent&);

  // mosquitto static → 인스턴스 dispatch
  static void onConnect   (struct mosquitto*, void* userdata, int rc);
  static void onDisconnect(struct mosquitto*, void* userdata, int rc);
  void handleConnect   (int rc);
  void handleDisconnect(int rc);

  // 핵심 로직
  bool fillBsm(j2735BasicSafetyMessage& bsm,
               uint8_t id_storage[4],
               uint8_t brake_bits[1]);
  void buildAndPublish(const uint8_t* uper_buf, size_t uper_len);
  static int gearToTransmission(uint8_t g);
};

// ---------------------------------------------------------------------------
// 생성자
// ---------------------------------------------------------------------------
MqttBsmTxNode::MqttBsmTxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {
  // 1. MQTT 파라미터 (defaults: test 서버)
  pnh_.param<std::string>("broker_host",   broker_host_, std::string("121.137.106.141"));
  pnh_.param<int>        ("broker_port",   broker_port_, 23312);
  pnh_.param<std::string>("username",      username_,    std::string("ut_adcp"));
  pnh_.param<std::string>("password",      password_,    std::string("ut_adcp123!@#"));
  pnh_.param<std::string>("topic",         topic_,       std::string("V2N/1321103202/bsm"));
  pnh_.param<std::string>("client_id",     client_id_,   std::string(""));
  pnh_.param<int>        ("keepalive",     keepalive_,   60);
  pnh_.param<int>        ("qos",           qos_,         0);
  pnh_.param<bool>       ("retain",        retain_,      false);
  pnh_.param<int>        ("reconnect_min", reconnect_min_, 1);
  pnh_.param<int>        ("reconnect_max", reconnect_max_, 30);
  pnh_.param<bool>       ("verbose_first_msg", verbose_first_msg_, true);

  // 2. 차량 / 토픽 파라미터
  std::string vid_str;
  pnh_.param<std::string>("vehicle_id", vid_str, std::string(""));
  if (!vid_str.empty()) {
    uint8_t parsed[4];
    if (parseVehicleIdHex(vid_str, parsed)) {
      std::memcpy(vehicle_id_, parsed, 4);
      ROS_INFO("[mqtt_bsm_tx] vehicle_id (hex) = %02X %02X %02X %02X",
               vehicle_id_[0], vehicle_id_[1], vehicle_id_[2], vehicle_id_[3]);
    } else {
      parseVehicleIdAscii(vid_str, parsed);
      std::memcpy(vehicle_id_, parsed, 4);
      ROS_INFO("[mqtt_bsm_tx] vehicle_id ASCII first-4 of '%s' = "
               "%02X %02X %02X %02X",
               vid_str.c_str(),
               vehicle_id_[0], vehicle_id_[1], vehicle_id_[2], vehicle_id_[3]);
    }
  }

  pnh_.param("vehicle_width_cm",  vehicle_width_cm_,  vehicle_width_cm_);
  pnh_.param("vehicle_length_cm", vehicle_length_cm_, vehicle_length_cm_);
  pnh_.param<std::string>("inspva_topic", inspva_topic_, inspva_topic_);
  pnh_.param<std::string>("v_can_topic",  vcan_topic_,   vcan_topic_);
  pnh_.param("publish_rate", publish_rate_, publish_rate_);
  if (publish_rate_ <= 0.0) publish_rate_ = 10.0;

  // 3. Subscribe
  sub_inspva_ = nh_.subscribe(inspva_topic_, 10, &MqttBsmTxNode::onInspva, this);
  sub_vcan_   = nh_.subscribe(vcan_topic_,   10, &MqttBsmTxNode::onVCan,   this);

  // 4. Timer (publish_rate Hz)
  const double period = 1.0 / publish_rate_;
  timer_ = nh_.createTimer(ros::Duration(period), &MqttBsmTxNode::onTimer, this);

  ROS_INFO("[mqtt_bsm_tx] started - broker=%s:%d topic=%s vehicle_id=%02X%02X%02X%02X",
           broker_host_.c_str(), broker_port_, topic_.c_str(),
           vehicle_id_[0], vehicle_id_[1], vehicle_id_[2], vehicle_id_[3]);

  // 5. mosquitto 초기화
  mosquitto_lib_init();

  const char* client_id_arg = client_id_.empty() ? nullptr : client_id_.c_str();
  mosq_ = mosquitto_new(client_id_arg, /*clean_session=*/true, this);
  if (!mosq_) {
    ROS_ERROR("[mqtt_bsm_tx] mosquitto_new failed: %s", strerror(errno));
    return;
  }

  // 6. 인증
  if (!username_.empty()) {
    const char* pw_arg = password_.empty() ? nullptr : password_.c_str();
    int u_rc = mosquitto_username_pw_set(mosq_, username_.c_str(), pw_arg);
    if (u_rc != MOSQ_ERR_SUCCESS) {
      ROS_WARN("[mqtt_bsm_tx] mosquitto_username_pw_set rc=%d (%s)",
               u_rc, mosquitto_strerror(u_rc));
    }
  }

  // 7. 콜백 등록
  mosquitto_connect_callback_set(mosq_,    &MqttBsmTxNode::onConnect);
  mosquitto_disconnect_callback_set(mosq_, &MqttBsmTxNode::onDisconnect);

  // 8. 재접속 지연 (1~30s, exponential backoff)
  mosquitto_reconnect_delay_set(mosq_, reconnect_min_, reconnect_max_, true);

  // 9. broker 접속 시도 (실패해도 loop_start 가 재시도)
  int rc = mosquitto_connect(mosq_, broker_host_.c_str(), broker_port_, keepalive_);
  if (rc != MOSQ_ERR_SUCCESS) {
    ROS_ERROR("[mqtt_bsm_tx] mosquitto_connect rc=%d (%s) - will retry via loop",
              rc, mosquitto_strerror(rc));
  }

  // 10. 백그라운드 네트워크 스레드 시작
  int ls_rc = mosquitto_loop_start(mosq_);
  if (ls_rc != MOSQ_ERR_SUCCESS) {
    ROS_ERROR("[mqtt_bsm_tx] mosquitto_loop_start rc=%d (%s)",
              ls_rc, mosquitto_strerror(ls_rc));
  } else {
    mosq_started_ = true;
  }
}

MqttBsmTxNode::~MqttBsmTxNode() {
  if (mosq_) {
    mosquitto_disconnect(mosq_);
    if (mosq_started_) {
      mosquitto_loop_stop(mosq_, /*force=*/false);
    }
    mosquitto_destroy(mosq_);
    mosq_ = nullptr;
  }
  mosquitto_lib_cleanup();
}

// ---------------------------------------------------------------------------
// mosquitto static → 인스턴스 dispatch
// ---------------------------------------------------------------------------
void MqttBsmTxNode::onConnect(struct mosquitto*, void* userdata, int rc) {
  auto* self = static_cast<MqttBsmTxNode*>(userdata);
  if (self) self->handleConnect(rc);
}

void MqttBsmTxNode::onDisconnect(struct mosquitto*, void* userdata, int rc) {
  auto* self = static_cast<MqttBsmTxNode*>(userdata);
  if (self) self->handleDisconnect(rc);
}

void MqttBsmTxNode::handleConnect(int rc) {
  ROS_INFO("[mqtt_bsm_tx] connected to broker (rc=%d)", rc);
  if (rc != 0) {
    ROS_ERROR("[mqtt_bsm_tx] connect failed rc=%d (%s)",
              rc, mosquitto_connack_string(rc));
  }
}

void MqttBsmTxNode::handleDisconnect(int rc) {
  if (rc != 0) {
    ROS_WARN_THROTTLE(10.0, "[mqtt_bsm_tx] disconnected rc=%d, reconnecting", rc);
  } else {
    ROS_INFO("[mqtt_bsm_tx] disconnected cleanly");
  }
}

// ---------------------------------------------------------------------------
// 입력 콜백
// ---------------------------------------------------------------------------
void MqttBsmTxNode::onInspva(const novatel_gps_msgs::Inspva::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_input_);
  latest_inspva_ = *msg;
  has_inspva_    = true;
}

void MqttBsmTxNode::onVCan(const katech_custom_msgs::v_can_msg::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_input_);
  latest_vcan_ = *msg;
  has_vcan_    = true;
}

// ---------------------------------------------------------------------------
// Timer 콜백: 10Hz fillBsm → UPER 인코딩 → V2N 헤더 prepend → publish
// ---------------------------------------------------------------------------
void MqttBsmTxNode::onTimer(const ros::TimerEvent&) {
  if (!has_inspva_) {
    ROS_WARN_THROTTLE(2.0, "[mqtt_bsm_tx] waiting for Inspva");
    return;
  }

  if (!mosq_) {
    ROS_WARN_THROTTLE(2.0, "[mqtt_bsm_tx] mosquitto handle null, skip publish");
    return;
  }

  // BSM 채움
  j2735BasicSafetyMessage bsm{};
  uint8_t id_storage[4]   = {0};
  uint8_t brake_bits[1]   = {0};
  if (!fillBsm(bsm, id_storage, brake_bits)) {
    return;
  }

  // MessageFrame 으로 감쌈 (type-pointer 방식, bsm_tx_node 와 동일)
  j2735MessageFrame mf{};
  mf.messageId      = J2735_MSG_ID_BSM;
  mf.value.type     = const_cast<ASN1CType*>(asn1_type_j2735BasicSafetyMessage);
  mf.value.u.data   = &bsm;

  // UPER 인코딩
  uint8_t*    buf = nullptr;
  ASN1Error   err{};
  asn1_ssize_t len = asn1_uper_encode2(&buf, asn1_type_j2735MessageFrame, &mf, &err);
  if (len <= 0 || buf == nullptr) {
    ROS_ERROR_THROTTLE(2.0,
        "[mqtt_bsm_tx] UPER encode failed: len=%ld bit_pos=%d msg='%s'",
        static_cast<long>(len), err.bit_pos, err.msg);
    if (buf) {
      asn1_free(buf);
    }
    return;
  }

  // V2N 헤더 prepend + MQTT publish
  buildAndPublish(buf, static_cast<size_t>(len));

  asn1_free(buf);
  // bsm 내부의 id.buf / brakes.wheelBrakes.buf 는 local storage 이므로
  // asn1_free_value 호출하지 않는다.
}

// ---------------------------------------------------------------------------
// V2N 16-byte 헤더 prepend + mosquitto_publish
// ---------------------------------------------------------------------------
void MqttBsmTxNode::buildAndPublish(const uint8_t* uper_buf, size_t uper_len) {
  std::vector<uint8_t> payload;
  payload.reserve(16 + uper_len);

  // 경찰청 V2N 정보연계 규격 ITSK-00150-2 <표 4-11> "단일 메시지 전송을
  // 위한 V2N Container" (container_type=0x04, sem_length=0) 의 fixed 16-byte
  // header. 시흥 V2N 브로커 SPaT 수신 페이로드와 동일 layout 을 따른다.

  // offset 0 : container_type (8 bit) = 0x04 (단일 메시지 전송, <표 4-3>)
  payload.push_back(0x04);
  // offset 1 : version (8 bit) = 0x00
  payload.push_back(0x00);
  // offset 2-3 : fid (16 bit BE) = 0xFF10 (BSM, 차량→센터)
  //   출처: ~/protocol/경4 V2N FID PSID 정의안 (1).xlsx FID 시트
  //   (참고: SPaT 는 0xFF11, TLSM 은 0xFF12, 통행지시 결과 수집은 0xFF09)
  payload.push_back(0xff);
  payload.push_back(0x10);

  // offset 4 : standard_type (8 bit) = 0x01 (KS 표준, <표 4-5>)
  payload.push_back(0x01);
  // offset 5 : sem_length (8 bit) = 0x00 (Service Enhancement Metadata 없음)
  payload.push_back(0x00);
  // offset 6 : flags (8 bit) = 0x00
  payload.push_back(0x00);

  // offset 7 : message_id (8 bit) = container-level 메시지 식별자(시퀀스)
  //   <표 4-11> 주석: 실제 V2X 메시지 디코딩 이후의 id 와는 별개의
  //   V2N container 레벨 고유 식별자
  const uint8_t cur_seq = seq_.fetch_add(1, std::memory_order_relaxed);
  payload.push_back(cur_seq);

  // offset 8-11 : psid (32 bit BE) = 0x00014082 (BSM, PSID=82050)
  //   출처: ~/protocol/경4 V2N FID PSID 정의안 (1).xlsx PSID 시트
  //   (참고: SPaT 는 0x00014085=82055, TLSM 은 0x00014100=82176)
  payload.push_back(0x00);
  payload.push_back(0x01);
  payload.push_back(0x40);
  payload.push_back(0x82);

  // offset 12-15 : message_length (32 bit BE) = 후속 UPER 메시지 길이
  const uint32_t inner = static_cast<uint32_t>(uper_len);
  payload.push_back(static_cast<uint8_t>((inner >> 24) & 0xff));
  payload.push_back(static_cast<uint8_t>((inner >> 16) & 0xff));
  payload.push_back(static_cast<uint8_t>((inner >>  8) & 0xff));
  payload.push_back(static_cast<uint8_t>( inner        & 0xff));

  // offset 16.. : UPER MessageFrame
  payload.insert(payload.end(), uper_buf, uper_buf + uper_len);

  // 첫 publish hex dump 로그
  if (verbose_first_msg_ && first_publish_.exchange(false)) {
    char hex[3 * 16 + 1] = {0};
    const size_t dump_n = std::min<size_t>(16, payload.size());
    for (size_t i = 0; i < dump_n; ++i) {
      std::snprintf(hex + i * 3, sizeof(hex) - i * 3, "%02x ", payload[i]);
    }
    ROS_INFO_ONCE("[mqtt_bsm_tx] first BSM published total=%zu inner=%zu hex[0..16]=%s",
                  payload.size(), uper_len, hex);
  }

  int rc = mosquitto_publish(mosq_,
                             /*mid=*/nullptr,
                             topic_.c_str(),
                             static_cast<int>(payload.size()),
                             payload.data(),
                             qos_,
                             retain_);
  if (rc != MOSQ_ERR_SUCCESS) {
    ROS_WARN_THROTTLE(2.0, "[mqtt_bsm_tx] publish rc=%d (%s)",
                      rc, mosquitto_strerror(rc));
  }
}

// ---------------------------------------------------------------------------
// BSM 필드 채움 (bsm_tx_node::fillBsm 복제)
// ---------------------------------------------------------------------------
bool MqttBsmTxNode::fillBsm(j2735BasicSafetyMessage& bsm,
                             uint8_t id_storage[4],
                             uint8_t brake_bits[1]) {
  std::lock_guard<std::mutex> lk(mtx_input_);
  auto& core = bsm.coreData;

  // msgCnt (0..127 wrap)
  core.msgCnt = static_cast<j2735MsgCount>(bsm_msg_cnt_ & 0x7F);
  bsm_msg_cnt_++;

  // id (TemporaryID = OCTET STRING SIZE(4))
  std::memcpy(id_storage, vehicle_id_, 4);
  core.id.buf = id_storage;
  core.id.len = 4;

  // secMark: 0..65535 ms within current minute
  {
    uint64_t ms_now = ros::Time::now().toNSec() / 1000000ULL;
    core.secMark = static_cast<j2735DSecond>(ms_now % 60000ULL);
  }

  // lat/lon: Inspva[deg] → BSM[1/10 microdeg = 1e-7 deg]
  {
    long long lat_raw = static_cast<long long>(
        std::round(latest_inspva_.latitude  * 1e7));
    long long lon_raw = static_cast<long long>(
        std::round(latest_inspva_.longitude * 1e7));
    if (lat_raw < -900000000LL)  lat_raw = -900000000LL;
    if (lat_raw >  900000000LL)  lat_raw =  900000000LL;
    if (lon_raw < -1799999999LL) lon_raw = -1799999999LL;
    if (lon_raw >  1800000000LL) lon_raw =  1800000000LL;
    core.lat  = static_cast<j2735Latitude>(lat_raw);
    core.Long = static_cast<j2735Longitude>(lon_raw);
  }

  // elev: Inspva.height[m] → BSM Elevation[0.1m]
  {
    long long e = static_cast<long long>(
        std::round(latest_inspva_.height * 10.0));
    if (e < -4096LL)  e = -4096LL;
    if (e > 61439LL)  e = 61439LL;
    core.elev = static_cast<j2735Elevation>(e);
  }

  // positional accuracy : Inspva 에 std 가 없어 unavailable
  core.accuracy.semiMajor    = 255;    // unavailable
  core.accuracy.semiMinor    = 255;    // unavailable
  core.accuracy.orientation  = 65535;  // unavailable

  // transmission
  core.transmission = has_vcan_
      ? static_cast<j2735TransmissionState>(
            gearToTransmission(latest_vcan_.gear_status))
      : j2735TransmissionState_unavailable;

  // speed: Inspva.north/east_velocity[m/s] → BSM Speed[0.02 m/s]
  {
    double vN = latest_inspva_.north_velocity;
    double vE = latest_inspva_.east_velocity;
    double v  = std::sqrt(vN * vN + vE * vE);
    int s = static_cast<int>(std::round(v / 0.02));
    if (s < 0) s = 0;
    if (s > 8190) s = 8190;
    core.speed = static_cast<j2735Speed>(s);
  }

  // heading: Inspva.azimuth[deg, 0=North] → BSM Heading[0.0125 deg]
  {
    double az = latest_inspva_.azimuth;
    while (az < 0.0)     az += 360.0;
    while (az >= 360.0)  az -= 360.0;
    int hraw = static_cast<int>(std::round(az / 0.0125));
    if (hraw < 0)      hraw = 0;
    if (hraw > 28799)  hraw = 28799;
    core.heading = static_cast<j2735Heading>(hraw);
  }

  // angle: v_can.steering_angle[deg] → BSM Angle[1.5 deg]
  if (has_vcan_) {
    int a = static_cast<int>(std::round(latest_vcan_.steering_angle / 1.5));
    core.angle = static_cast<j2735SteeringWheelAngle>(clampi(a, -126, 126));
  } else {
    core.angle = 127;  // unavailable
  }

  // accelSet
  if (has_vcan_) {
    core.accelSet.Long = static_cast<j2735Acceleration>(
        clampi(static_cast<int>(std::round(latest_vcan_.long_acceleration * 100.0)),
               -2000, 2000));
    core.accelSet.lat  = static_cast<j2735Acceleration>(
        clampi(static_cast<int>(std::round(latest_vcan_.lat_acceleration  * 100.0)),
               -2000, 2000));
    core.accelSet.vert = static_cast<j2735VerticalAcceleration>(-127);  // unavailable
    core.accelSet.yaw  = static_cast<j2735YawRate>(
        clampi(static_cast<int>(std::round(latest_vcan_.yaw_rate * 100.0)),
               -32767, 32767));
  } else {
    core.accelSet.Long = 2001;  // unavailable
    core.accelSet.lat  = 2001;
    core.accelSet.vert = -127;
    core.accelSet.yaw  = 0;
  }

  // brakes
  {
    bool applied = has_vcan_ && (latest_vcan_.brake_pedal_pos > 0.05);
    brake_bits[0] = applied ? 0x78 : 0x80;
    core.brakes.wheelBrakes.buf = brake_bits;
    core.brakes.wheelBrakes.len = 5;  // 5 bits
    core.brakes.traction  = j2735TractionControlStatus_unavailable;
    core.brakes.abs       = j2735AntiLockBrakeStatus_unavailable;
    core.brakes.scs       = j2735StabilityControlStatus_unavailable;
    core.brakes.brakeBoost= j2735BrakeBoostApplied_unavailable;
    core.brakes.auxBrakes = j2735AuxiliaryBrakeStatus_unavailable;
  }

  // size (cm)
  core.size.width  = static_cast<j2735VehicleWidth>(
                       clampi(vehicle_width_cm_, 0, 1023));
  core.size.length = static_cast<j2735VehicleLength>(
                       clampi(vehicle_length_cm_, 0, 4095));

  // partII / regional: omit
  bsm.partII_option   = FALSE;
  bsm.partII.tab      = nullptr;
  bsm.partII.count    = 0;
  bsm.regional_option = FALSE;
  bsm.regional.tab    = nullptr;
  bsm.regional.count  = 0;

  return true;
}

int MqttBsmTxNode::gearToTransmission(uint8_t g) {
  switch (g) {
    case 1: return j2735TransmissionState_park;
    case 2: return j2735TransmissionState_reverseGears;
    case 3: return j2735TransmissionState_neutral;
    case 4: return j2735TransmissionState_forwardGears;
    default: return j2735TransmissionState_unavailable;
  }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "mqtt_bsm_tx_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  MqttBsmTxNode node(nh, pnh);
  ros::spin();
  return 0;
}

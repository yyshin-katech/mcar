// =============================================================================
// bsm_tx_node.cpp
//
// J2735 BasicSafetyMessage (BSM) 송신 노드.
// - NavPVT (/ublox/navpvt) + v_can (/sensors/v_can) 를 캐시하여 10Hz 로 BSM 을
//   채우고 MessageFrame 으로 감싸 UPER 인코딩 → /siheung_v2x/bsm_tx 토픽 발행.
// - 옵션 (~enable_udp=true) 시 OBU 헤더(5바이트) prefix 후 UDP sendto 까지 수행.
//
// 사양: claude_work_list/bsm_tx.md (analyst 작성, 2026-05-21).
// =============================================================================

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <ublox_msgs/NavPVT.h>
#include <katech_custom_msgs/v_can_msg.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <mutex>
#include <string>
#include <vector>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

// 헤더 자체가 extern "C" 블록을 포함하므로 직접 include 가능.
// (j2735_decode.cpp 와 동일한 패턴)
#include "ffasn1-j2735-2026-KSR1600.h"
#include "asn1defs.h"

// J2735 BSM messageId
#define J2735_MSG_ID_BSM 20  // 0x14

namespace {

// ---------------------------------------------------------------------------
// 유틸: C++14 호환 clamp
// ---------------------------------------------------------------------------
inline int clampi(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ---------------------------------------------------------------------------
// vehicle_id 파라미터 파싱 ("AABBCCDD" 또는 "0xAABBCCDD")
// 성공 시 true, 실패 시 false 반환 (호출자가 default 유지).
// ---------------------------------------------------------------------------
bool parseVehicleIdHex(const std::string& in, uint8_t out[4]) {
  std::string s = in;
  // 양 끝 공백 제거
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.erase(s.begin());
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) s.pop_back();
  // "0x" / "0X" prefix 제거
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

}  // namespace

// ---------------------------------------------------------------------------
// BsmTxNode
// ---------------------------------------------------------------------------
class BsmTxNode {
 public:
  BsmTxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~BsmTxNode();

 private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_navpvt_;
  ros::Subscriber sub_vcan_;
  ros::Publisher  pub_bsm_;
  ros::Timer      timer_;

  // 최신 상태 캐시
  std::mutex                     mtx_;
  ublox_msgs::NavPVT             latest_navpvt_;
  bool                           has_navpvt_ = false;
  katech_custom_msgs::v_can_msg  latest_vcan_;
  bool                           has_vcan_ = false;

  // 카운터
  uint8_t msg_cnt_ = 0;

  // 파라미터
  std::string navpvt_topic_     = "/ublox/navpvt";
  std::string vcan_topic_       = "/sensors/v_can";
  std::string bsm_topic_        = "/siheung_v2x/bsm_tx";
  uint8_t     vehicle_id_[4]    = {0x00, 0x00, 0x00, 0x01};
  int         vehicle_width_cm_ = 190;
  int         vehicle_length_cm_= 464;
  bool        enable_udp_       = false;
  std::string obu_ip_           = "192.168.0.10";
  int         obu_port_         = 9999;

  // UDP
  int          sock_fd_  = -1;
  sockaddr_in  obu_addr_ {};
  uint8_t      obu_seq_  = 0;

  // 콜백
  void onNavPvt(const ublox_msgs::NavPVT::ConstPtr& msg);
  void onVCan(const katech_custom_msgs::v_can_msg::ConstPtr& msg);
  void onTimer(const ros::TimerEvent&);

  // 핵심 로직
  bool fillBsm(j2735BasicSafetyMessage& bsm,
               uint8_t id_storage[4],
               uint8_t brake_bits[1]);

  // 유틸
  static int gearToTransmission(uint8_t gear_status);
};

BsmTxNode::BsmTxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {
  // 파라미터 로드
  pnh_.param<std::string>("navpvt_topic", navpvt_topic_, navpvt_topic_);
  pnh_.param<std::string>("vcan_topic",   vcan_topic_,   vcan_topic_);
  pnh_.param<std::string>("bsm_topic",    bsm_topic_,    bsm_topic_);

  std::string vid_str;
  // default 빈 문자열 → 파라미터 미지정 시 default vehicle_id 유지
  pnh_.param<std::string>("vehicle_id", vid_str, std::string(""));
  if (!vid_str.empty()) {
    uint8_t parsed[4];
    if (parseVehicleIdHex(vid_str, parsed)) {
      std::memcpy(vehicle_id_, parsed, 4);
    } else {
      ROS_WARN_STREAM("[bsm_tx] failed to parse ~vehicle_id='" << vid_str
                       << "' (expect 'AABBCCDD' or '0xAABBCCDD'); using default "
                          "{0x00,0x00,0x00,0x01}");
    }
  }

  pnh_.param("vehicle_width_cm",  vehicle_width_cm_,  vehicle_width_cm_);
  pnh_.param("vehicle_length_cm", vehicle_length_cm_, vehicle_length_cm_);
  pnh_.param("enable_udp",        enable_udp_,        enable_udp_);
  pnh_.param<std::string>("obu_ip", obu_ip_, obu_ip_);
  pnh_.param("obu_port",          obu_port_,          obu_port_);

  // Subscribe / Advertise
  sub_navpvt_ = nh_.subscribe(navpvt_topic_, 10, &BsmTxNode::onNavPvt, this);
  sub_vcan_   = nh_.subscribe(vcan_topic_,   10, &BsmTxNode::onVCan,   this);
  pub_bsm_    = nh_.advertise<std_msgs::UInt8MultiArray>(bsm_topic_, 10);

  // UDP (옵션)
  if (enable_udp_) {
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
      ROS_ERROR("[bsm_tx] failed to create UDP socket; disabling UDP");
      enable_udp_ = false;
    } else {
      std::memset(&obu_addr_, 0, sizeof(obu_addr_));
      obu_addr_.sin_family = AF_INET;
      obu_addr_.sin_port   = htons(static_cast<uint16_t>(obu_port_));
      if (inet_pton(AF_INET, obu_ip_.c_str(), &obu_addr_.sin_addr) != 1) {
        ROS_ERROR_STREAM("[bsm_tx] invalid ~obu_ip='" << obu_ip_
                          << "'; disabling UDP");
        close(sock_fd_);
        sock_fd_ = -1;
        enable_udp_ = false;
      } else {
        ROS_INFO_STREAM("[bsm_tx] UDP enabled → " << obu_ip_ << ":" << obu_port_);
      }
    }
  }

  // 10Hz timer
  timer_ = nh_.createTimer(ros::Duration(0.1), &BsmTxNode::onTimer, this);

  ROS_INFO_STREAM("[bsm_tx] node started"
                  << " navpvt=" << navpvt_topic_
                  << " vcan="   << vcan_topic_
                  << " out="    << bsm_topic_
                  << " enable_udp=" << (enable_udp_ ? "true" : "false"));
}

BsmTxNode::~BsmTxNode() {
  if (sock_fd_ >= 0) {
    close(sock_fd_);
    sock_fd_ = -1;
  }
}

void BsmTxNode::onNavPvt(const ublox_msgs::NavPVT::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  latest_navpvt_ = *msg;
  has_navpvt_    = true;
}

void BsmTxNode::onVCan(const katech_custom_msgs::v_can_msg::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  latest_vcan_ = *msg;
  has_vcan_    = true;
}

void BsmTxNode::onTimer(const ros::TimerEvent&) {
  if (!has_navpvt_) {
    ROS_WARN_THROTTLE(2.0, "[bsm_tx] waiting for NavPVT");
    return;
  }

  // BSM 채움. id/wheelBrakes 가 가리키는 storage 는 호출자가 보유.
  j2735BasicSafetyMessage bsm{};
  uint8_t id_storage[4]   = {0};
  uint8_t brake_bits[1]   = {0};
  if (!fillBsm(bsm, id_storage, brake_bits)) {
    return;
  }

  // MessageFrame 으로 감쌈 (type-pointer 방식)
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
        "[bsm_tx] UPER encode failed: len=%ld bit_pos=%d msg='%s'",
        static_cast<long>(len), err.bit_pos, err.msg);
    if (buf) {
      asn1_free(buf);
    }
    return;
  }

  // (1) ROS topic publish
  std_msgs::UInt8MultiArray out;
  out.data.assign(buf, buf + static_cast<size_t>(len));
  pub_bsm_.publish(out);

  // (2) UDP (옵션)
  if (enable_udp_ && sock_fd_ >= 0) {
    // OBU 헤더 5 bytes
    //   [0] Frame Type   : 1 = PC → OBU
    //   [1] Seq No
    //   [2] msg source   : 0 = from RSU
    //   [3] isMsgFrame   : 0 = MessageFrame
    //   [4] Reserved
    std::vector<uint8_t> pkt(5 + static_cast<size_t>(len));
    pkt[0] = 1;
    pkt[1] = obu_seq_++;
    pkt[2] = 0;
    pkt[3] = 0;
    pkt[4] = 0;
    std::memcpy(pkt.data() + 5, buf, static_cast<size_t>(len));
    ssize_t n = sendto(sock_fd_, pkt.data(), pkt.size(), 0,
                       reinterpret_cast<sockaddr*>(&obu_addr_),
                       sizeof(obu_addr_));
    if (n < 0) {
      ROS_WARN_THROTTLE(2.0, "[bsm_tx] UDP sendto failed: errno=%d", errno);
    }
  }

  asn1_free(buf);
  // bsm 내부의 id.buf / brakes.wheelBrakes.buf 는 우리가 채운 local storage 이므로
  // asn1_free_value 호출하지 않는다.
}

bool BsmTxNode::fillBsm(j2735BasicSafetyMessage& bsm,
                         uint8_t id_storage[4],
                         uint8_t brake_bits[1]) {
  std::lock_guard<std::mutex> lk(mtx_);
  auto& core = bsm.coreData;

  // msgCnt (0..127 wrap)
  core.msgCnt = static_cast<j2735MsgCount>(msg_cnt_ & 0x7F);
  msg_cnt_++;

  // id (TemporaryID = OCTET STRING SIZE(4))
  std::memcpy(id_storage, vehicle_id_, 4);
  core.id.buf = id_storage;
  core.id.len = 4;

  // secMark: 0..65535 ms within current minute
  {
    uint64_t ms_now = ros::Time::now().toNSec() / 1000000ULL;
    core.secMark = static_cast<j2735DSecond>(ms_now % 60000ULL);
  }

  // lat/lon: NavPVT 와 BSM 모두 1/10 microdeg → identity
  core.lat  = static_cast<j2735Latitude>(
                clampi(latest_navpvt_.lat,  -900000000,  900000000));
  core.Long = static_cast<j2735Longitude>(
                clampi(latest_navpvt_.lon, -1799999999, 1800000000));

  // elev: NavPVT.hMSL[mm] → BSM Elevation[0.1m]
  core.elev = static_cast<j2735Elevation>(
                clampi(latest_navpvt_.hMSL / 100, -4096, 61439));

  // positional accuracy
  {
    int a = static_cast<int>(latest_navpvt_.hAcc / 50);  // mm → 0.05m
    core.accuracy.semiMajor    = (a > 254) ? 255 : a;
    core.accuracy.semiMinor    = core.accuracy.semiMajor;
    core.accuracy.orientation  = 65535;  // unavailable
  }

  // transmission
  core.transmission = has_vcan_
      ? static_cast<j2735TransmissionState>(
            gearToTransmission(latest_vcan_.gear_status))
      : j2735TransmissionState_unavailable;

  // speed: NavPVT.gSpeed[mm/s] → BSM Speed[0.02 m/s]
  {
    int s = latest_navpvt_.gSpeed / 20;
    if (s < 0) s = 0;
    if (s > 8190) s = 8190;  // 8191 = unavailable 충돌 회피
    core.speed = static_cast<j2735Speed>(s);
  }

  // heading: NavPVT.heading[deg*1e-5] → BSM Heading[0.0125 deg]
  {
    int32_t h = latest_navpvt_.heading;
    if (h < 0) h += 36000000;  // -360e5..360e5 → 0..360e5
    double hd  = static_cast<double>(h) * 1e-5;          // 0..360 deg
    int hraw   = static_cast<int>(hd / 0.0125 + 0.5);
    if (hraw < 0)      hraw = 0;
    if (hraw > 28799)  hraw = 28799;
    core.heading = static_cast<j2735Heading>(hraw);
  }

  // angle: v_can.steering_angle[deg] → BSM Angle[1.5 deg], -126..126 (127 = unavailable)
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
    // bit0 = unavailable, bit1=leftFront, bit2=leftRear, bit3=rightFront, bit4=rightRear
    // MSB-first bit ordering 가정 (ffasn1 컨벤션).
    //   applied:   0b01111000 = 0x78 (4륜 적용)
    //   else:      0b10000000 = 0x80 (unavailable)
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

  // partII / regional: 1단계는 모두 omit
  bsm.partII_option   = FALSE;
  bsm.partII.tab      = nullptr;
  bsm.partII.count    = 0;
  bsm.regional_option = FALSE;
  bsm.regional.tab    = nullptr;
  bsm.regional.count  = 0;

  return true;
}

int BsmTxNode::gearToTransmission(uint8_t g) {
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
  ros::init(argc, argv, "bsm_tx_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  BsmTxNode node(nh, pnh);
  ros::spin();
  return 0;
}

# MQTT BSM TX 노드 사양서 (siheung_v2x/mqtt_bsm_tx_node)

작성일: 2026-05-23 / 브랜치: `siheung_dev` / 작업물: **사양만, 코드 변경 없음**.

대상: `src/v2x/siheung_v2x/` 안에 J2735 BSM 송신용 **MQTT** ROS C++ 노드 신규 추가.
기존 `bsm_tx_node` 는 OBU 로 UDP 6666 전송만 한다. 본 노드는 **동일 BSM 인코딩 결과**를 **MQTT broker** 로도 publish 한다. SPaT 수신용 `mqtt_spat_rx_node` (mosquitto C API) 의 클라이언트 패턴을 재활용한다.

핵심 차이점:

| 구분 | `bsm_tx_node` | **`mqtt_bsm_tx_node` (본 사양)** |
|------|---------------|-----------------------------------|
| 전송 매체 | UDP (소켓) `192.168.1.5:6666` | **MQTT (mosquitto C API)** |
| 출력 포맷 | OBU 5-byte 헤더 + UPER MessageFrame | **V2N 16-byte 커스텀 헤더 + UPER MessageFrame** |
| 출력 토픽 | `/siheung_v2x/bsm_tx` (ROS) + UDP sendto | (ROS publish 없음) **MQTT publish: `V2N/1321103202/bsm`** |
| 라이브러리 | socket(AF_INET, SOCK_DGRAM) | libmosquitto-dev |
| 시퀀스 카운터 | `obu_seq_` (uint8) — non-atomic, 단일 콜백 스레드 | **`std::atomic<uint8_t> seq_`** — mosquitto loop_start 스레드와 ROS Timer 스레드 사이 안전 |

---

## §1. 목적·범위

`/sensors/gps/inspva` (`novatel_gps_msgs/Inspva`, 50 Hz) 를 캐시하여 **10 Hz** 로 J2735 `BasicSafetyMessage` 를 채우고 `MessageFrame` (msgID=20) UPER 로 인코딩한 뒤, **V2N 16-byte 커스텀 헤더** 를 prepend 하여 MQTT broker 의 BSM 토픽 (`V2N/1321103202/bsm`) 으로 publish 한다.

본 사양서는 신규 1개 + 수정 2개 (CMakeLists, launch) 만 다룬다. `bsm_tx_node.cpp` 와 `mqtt_spat_rx_node.cpp` 는 **일절 수정하지 않는다** (BSM 인코딩 로직은 file-local 복제, MQTT 패턴도 file-local 복제). 추후 리팩토링 여지를 위해 함수명은 기존 노드와 동일하게 맞춘다.

---

## §2. ROS 파라미터 표

| 파라미터명 | 타입 | 기본값 | 설명 |
|-----------|------|--------|------|
| `~broker_host` | string | `121.137.106.141` | MQTT broker 호스트. launch arg `$(arg mqtt_host)` 로 dispatch (test↔prod) |
| `~broker_port` | int | `23312` | MQTT broker 포트. test=23312, prod=10044 |
| `~username` | string | `ut_adcp` | MQTT 인증 username. launch arg `$(arg mqtt_user)` |
| `~password` | string | `ut_adcp123!@#` | MQTT 인증 password. launch arg `$(arg mqtt_pass)` |
| `~topic` | string | `V2N/1321103202/bsm` | publish 토픽 |
| `~client_id` | string | `""` (빈 문자열) | MQTT client_id. 빈 문자열이면 mosquitto가 random id 할당 (mqtt_spat_rx 와 동일 패턴) |
| `~keepalive` | int | `60` | MQTT keepalive (초) |
| `~qos` | int | `0` | MQTT QoS (0/1/2). BSM 은 손실 허용 → 0 |
| `~retain` | bool | `false` | MQTT retain flag. BSM 은 실시간이라 false 고정 |
| `~vehicle_id` | string | `"EV01"` | 차량 식별자. `bsm_tx_node` 와 동일하게 hex("AABBCCDD"/"0xAABBCCDD") 또는 ASCII 4바이트 fallback |
| `~vehicle_width_cm` | int | `190` | 차량 폭 (cm), BSM size.width |
| `~vehicle_length_cm` | int | `464` | 차량 길이 (cm), BSM size.length |
| `~inspva_topic` | string | `/sensors/gps/inspva` | INSPVA 입력 토픽 |
| `~publish_rate` | double | `10.0` | publish 주기 (Hz). `bsm_tx_node` 와 동일 10 Hz |
| `~verbose_first_msg` | bool | `true` | 첫 BSM publish 시 hex dump 로그 출력 |
| `~reconnect_min` | int | `1` | mosquitto 재접속 최소 지연 (초) |
| `~reconnect_max` | int | `30` | mosquitto 재접속 최대 지연 (초, exponential backoff) |
| `~v_can_topic` | string | `/sensors/v_can` | (옵션) v_can 입력 토픽. v_can 가 없으면 transmission/angle/brakes 는 unavailable. 1차에서는 GPS-only 도 허용 |

**기본값 선정 근거**: launch arg `mqtt_server:=test|prod` dispatch 패턴은 `mqtt_spat_rx_node` 와 동일하게 재사용하고, 파라미터 default 자체는 test 서버 (`121.137.106.141:23312`) 로 둔다. 토픽명 `V2N/1321103202/bsm` 는 SPaT 토픽 `V2N/1321103202/trf_drct/spat` 와 동일 intersectionId namespace 하위에 둔다 (사용자 명시).

---

## §3. 노드 구조

### §3.1 클래스 멤버

```cpp
class MqttBsmTxNode {
 public:
  MqttBsmTxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~MqttBsmTxNode();

 private:
  // ── ROS ─────────────────────────────────────────────
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_inspva_;
  ros::Subscriber sub_vcan_;       // optional
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
  // mosquitto loop_start 스레드와 ROS Timer 콜백이
  // 동시에 접근할 수 있으므로 atomic.
  std::atomic<uint8_t> seq_{0};

  // ── 차량 파라미터 ───────────────────────────────────
  uint8_t  vehicle_id_[4]    = {0x00, 0x00, 0x00, 0x01};
  int      vehicle_width_cm_ = 190;
  int      vehicle_length_cm_= 464;

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
  bool mosq_started_  = false;     // loop_start 성공 여부 (소멸자에서 loop_stop 호출 조건)
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
```

### §3.2 스레드 모델

mosquitto C API 는 **두 가지** 네트워크 처리 모드를 제공한다:

1. `mosquitto_loop_forever` — 호출 스레드 점유. 본 노드는 ROS Timer 도 돌려야 하므로 부적합.
2. `mosquitto_loop_start` — 별도 백그라운드 스레드에서 네트워크 I/O 수행. **본 노드 채택**.

본 노드의 스레드 구성:

| 스레드 | 역할 | mosquitto 호출 |
|--------|------|----------------|
| ROS spinner (main) | `ros::spin()` — 콜백 dispatch | (없음) |
| ROS subscriber 콜백 | `onInspva()` / `onVCan()` 에서 `latest_*` 갱신 (mtx_input_ 보호) | (없음) |
| ROS Timer 콜백 (10 Hz) | `onTimer()` → fillBsm → UPER 인코딩 → V2N 헤더 prepend → **`mosquitto_publish`** 직접 호출 | `mosquitto_publish` |
| mosquitto 백그라운드 (loop_start) | 네트워크 송수신, ping, reconnect, callbacks | `onConnect` / `onDisconnect` 발화 |

**mutex 보호 정책**:

- `mtx_input_`: `latest_inspva_`, `latest_vcan_`, `bsm_msg_cnt_`. subscriber 콜백(write) ↔ Timer 콜백(read+inc) 사이.
- `seq_` (atomic): publish 마다 `fetch_add(1)`. Timer 콜백 단일 사용이지만 atomic 으로 두는 게 향후 다중 publisher 확장 시 안전.
- `first_publish_` (atomic): Timer 콜백 단일 호출이라 atomic 불필요하지만 명시성 차원.
- `mosq_`: mosquitto C API 는 단일 핸들에 대한 `mosquitto_publish` 호출이 내부적으로 thread-safe (libmosquitto >= 1.6). **추가 mutex 불필요**. (mqtt_spat_rx 도 동일 가정.)
- ROS subscriber 갱신은 default callback queue 단일 thread 라 race 없음. (단일 `ros::spin()` 사용.)

### §3.3 콜백 흐름 다이어그램

```
/sensors/gps/inspva (50 Hz) ──▶ onInspva() ──▶ mtx_input_ → latest_inspva_
                                                                      │
                                                                      ▼
                              ros::Timer (10 Hz) ──▶ onTimer() ──▶ fillBsm()
                                                                      │
                                                                      ▼
                                                       asn1_uper_encode2(MessageFrame)
                                                                      │
                                                                      ▼
                                                       buildAndPublish(buf, len)
                                                                      │
                                                          ┌───────────┴───────────┐
                                                          ▼                       │
                                                  V2N 헤더 16B prepend            │
                                                  hdr[7] = seq_.fetch_add(1)      │
                                                          ▼                       │
                                                  mosquitto_publish(payload)──────┘
                                                          │
                                                          ▼ (network thread)
                                                  loop_start → broker
```

---

## §4. 코드 재사용 정책

### §4.1 재사용 후보와 의사결정

| 단위 | 원본 | 재사용 방법 |
|------|------|-------------|
| BSM 필드 채움 로직 (`fillBsm`) | `bsm_tx_node.cpp:296-433` | **file-local 복제**. 약 140 라인. 함수 시그니처/필드 이름 동일 유지 |
| `gearToTransmission` | `bsm_tx_node.cpp:435-443` | **file-local 복제**. 7 라인 |
| `parseVehicleIdHex` / `parseVehicleIdAscii` | `bsm_tx_node.cpp:53-80` | **file-local 복제**. anonymous namespace |
| `clampi` | `bsm_tx_node.cpp:45` | **file-local 복제**. 1 라인 |
| MessageFrame UPER 인코딩 (`asn1_uper_encode2`) | `bsm_tx_node.cpp:243-261` | **file-local 복제**. type-pointer 방식 동일 |
| mosquitto C API 패턴 (init/connect/loop_start/callbacks/reconnect/disconnect/cleanup) | `mqtt_spat_rx_node.cpp:350-516` | **file-local 복제**. static→instance dispatch 패턴 |
| `mosquitto_reconnect_delay_set` (exponential backoff) | `mqtt_spat_rx_node.cpp:410` | **file-local 복제** |

**결론**: 신규 `mqtt_bsm_tx_node.cpp` **1개 파일** 안에 위 모든 로직을 file-local 로 복제. 기존 `bsm_tx_node.cpp` 와 `mqtt_spat_rx_node.cpp` 는 **건드리지 않는다**. 두 노드가 동시에 동작해도 충돌하지 않는 이유:

- `bsm_tx_node` 는 UDP 송신, ROS topic `/siheung_v2x/bsm_tx` publish. MQTT 는 사용 안 함.
- `mqtt_bsm_tx_node` 는 MQTT publish only, ROS topic publish 없음 (ROS subscribe 만).
- 두 노드 모두 동일한 INSPVA 토픽을 subscribe 하지만 ROS subscription 은 다중 subscriber 안전.
- MQTT broker 측에서도 BSM 토픽 (`V2N/1321103202/bsm`) 은 본 노드만 publisher.

### §4.2 ASN.1 헤더/라이브러리 선택

`bsm_tx_node` 와 동일:

- include: `"ffasn1-j2735-2026-KSR1600.h"` + `"asn1defs.h"`
- 링크: `ffasn1-base`, `ffasn1-j2735-2020` (KSR1600 변형 export 포함)
- **`src/ffasn1-j2735-2026-KSR1600.c` 는 링크 안 함** (header + .so 패턴, `bsm_tx_node` 와 동일)
- 추가 링크: `${MOSQUITTO_LIB}` (find_library 캐시 변수, mqtt_spat_rx_node 의 find_library 결과 재사용)

---

## §5. 페이로드 구성

### §5.1 V2N 16-byte 헤더 사양

`mqtt_spat_rx_node.cpp:131-152` 의 `skipMqttV2nHeader()` 가 RX 측에서 보던 헤더와 **동일 포맷**. SPaT 헤더 60프레임 통계로 도출:

| Offset | Bytes | 값 | 설명 |
|--------|-------|-----|------|
| 0..3   | 4 | `04 00 ff 11` | **Magic (고정)** — RX 측에서 이 4 바이트로 V2N 헤더 인식 |
| 4..6   | 3 | `01 00 00` | **프로토콜 버전/reserved (고정)** — SPaT 통계상 60/60 고정 |
| 7      | 1 | `SS` | **Sequence counter** — `std::atomic<uint8_t> seq_` 가 publish 마다 fetch_add(1). 0xFF → 0x00 자동 wrap |
| 8..11  | 4 | `00 01 40 85` | **Sender/Site ID (고정)** — 정확한 의미 불명. SPaT 헤더에서 60/60 고정값. BSM 도 동일하게 사용 (추후 ROS 파라미터화 여지 남김) |
| 12..15 | 4 | `LL LL LL LL` | **후속 UPER MessageFrame 길이** — Big-Endian uint32. `asn1_uper_encode2` 반환 길이를 그대로 BE 인코딩 |
| 16..   | N | `00 14 ...` | **J2735 UPER MessageFrame** — `messageId=20` (BSM, 0x14). UPER 인코딩 결과. 일반적으로 40~60 byte |

### §5.2 V2N 헤더 채우는 의사 코드

```cpp
// uper_buf, uper_len: asn1_uper_encode2() 결과
std::vector<uint8_t> payload;
payload.reserve(16 + uper_len);

// offset 0..3 : magic
payload.push_back(0x04);
payload.push_back(0x00);
payload.push_back(0xff);
payload.push_back(0x11);

// offset 4..6 : version/reserved (SPaT 통계 기반 고정)
payload.push_back(0x01);
payload.push_back(0x00);
payload.push_back(0x00);

// offset 7 : sequence counter (atomic 후 후증가)
const uint8_t cur_seq = seq_.fetch_add(1, std::memory_order_relaxed);
payload.push_back(cur_seq);

// offset 8..11 : sender/site ID (고정)
payload.push_back(0x00);
payload.push_back(0x01);
payload.push_back(0x40);
payload.push_back(0x85);

// offset 12..15 : BE uint32 length
const uint32_t inner = static_cast<uint32_t>(uper_len);
payload.push_back(static_cast<uint8_t>((inner >> 24) & 0xff));
payload.push_back(static_cast<uint8_t>((inner >> 16) & 0xff));
payload.push_back(static_cast<uint8_t>((inner >>  8) & 0xff));
payload.push_back(static_cast<uint8_t>( inner        & 0xff));

// offset 16.. : UPER MessageFrame
payload.insert(payload.end(), uper_buf, uper_buf + uper_len);

// publish
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
```

### §5.3 BSM (UPER inner) 채움

`bsm_tx_node::fillBsm` 동일 복제. 핵심 필드만 요약:

| 필드 | 입력 | 변환 | BSM 단위 |
|------|------|------|----------|
| `msgCnt` | `bsm_msg_cnt_++` | `& 0x7F` | 0..127 wrap |
| `id` | `vehicle_id_[4]` | OCTET STRING len=4 | TemporaryID |
| `secMark` | `ros::Time::now()` | `ms % 60000` | DSecond |
| `lat` | `inspva.latitude` (deg) | `× 1e7` round, clamp ±9e8 | 1/10 μdeg |
| `Long` | `inspva.longitude` (deg) | `× 1e7` round, clamp ±1.8e9 | 1/10 μdeg |
| `elev` | `inspva.height` (m) | `× 10` round, clamp [-4096, 61439] | 0.1 m |
| `accuracy` | (없음) | unavailable (255/255/65535) | - |
| `speed` | `sqrt(vN² + vE²)` (m/s) | `/ 0.02` round, clamp [0, 8190] | 0.02 m/s |
| `heading` | `inspva.azimuth` (deg) | normalize [0, 360) → `/ 0.0125` | 0.0125 deg |
| `transmission` | `v_can.gear_status` | gearToTransmission() | enum |
| `angle` | `v_can.steering_angle` (deg) | `/ 1.5` clamp ±126 | 1.5 deg |
| `accelSet` | `v_can.long/lat/yaw` | `× 100` clamp | enum |
| `brakes` | `v_can.brake_pedal_pos > 0.05` | bit pattern 0x78/0x80 | 5-bit |
| `size` | `vehicle_width_cm_` / `vehicle_length_cm_` | clamp | cm |

### §5.4 첫 메시지 hex dump 예상치

- 총 길이: 헤더 16 B + UPER 약 40~60 B = **56~76 byte**
- offset 0..15 (헤더): `04 00 ff 11 01 00 00 SS 00 01 40 85 00 00 00 LL`
  - `SS` = 첫 publish 면 `0x00`, 그 다음 `0x01`, `0x02`, ...
  - `LL` = UPER 길이의 하위 1 byte (0~63 정도)
- offset 16..17 (MessageFrame UPER 시작): `00 14`
  - `messageId = 20` (BSM, 0x14) 를 UPER 로 인코딩하면 첫 16비트가 0x0014 형태로 나타남 (`mqtt_spat_rx` 에서 SPaT 가 `00 13` 으로 시작하는 것과 동일 패턴)

---

## §6. Sequence Counter

`std::atomic<uint8_t> seq_{0};` 멤버로 선언. publish 마다:

```cpp
const uint8_t cur_seq = seq_.fetch_add(1, std::memory_order_relaxed);
hdr[7] = cur_seq;
```

특성:

- **Thread-safe**: ROS Timer 콜백(10 Hz) 외 다른 스레드가 publish 하지 않더라도, mosquitto loop_start 스레드가 만에 하나 internal publish 콜백 같은 형태로 접근할 가능성에 대비.
- **자동 wrap**: `uint8_t` 라 0xFF + 1 = 0x00 자동 overflow. 256 publish 마다 1 cycle.
- **Atomic 메모리 순서**: publish 가 단일 producer 라 `relaxed` 로 충분. (필요 시 `seq_cst` 로 강화 가능.)
- **첫 publish**: `seq_ = 0` 으로 초기화되어 있고 `fetch_add` 가 **후증가 (post-increment)** 이므로 첫 헤더의 offset 7 = `0x00`, 두 번째 = `0x01` ...

검증 (§10 시나리오 ②): `mosquitto_sub -C 5` 로 5개 받아서 offset 7 만 추출하면 `00 01 02 03 04` 순으로 나와야 한다.

---

## §7. Build 통합 (CMakeLists.txt 수정)

`src/v2x/siheung_v2x/CMakeLists.txt` 끝에 다음 5줄 추가 (find_library 결과 `${MOSQUITTO_LIB}` 는 mqtt_spat_rx_node 블록에서 이미 캐싱되어 있어 재사용 가능):

```cmake
## ── MQTT BSM TX ────────────────────────────────────────────
## mqtt_bsm_tx_node: MQTT broker(V2N)로 BSM publish
## (bsm_tx_node 의 BSM 인코딩 로직을 file-local 복제, libmosquitto 링크)
add_executable(mqtt_bsm_tx_node src/mqtt_bsm_tx_node.cpp)
target_link_libraries(mqtt_bsm_tx_node
  ${catkin_LIBRARIES}
  ${MOSQUITTO_LIB}
  ffasn1-base
  ffasn1-j2735-2020
)
```

요약:

| 항목 | 변경 |
|------|------|
| `add_executable` | 1개 추가 (`mqtt_bsm_tx_node`) — source `src/mqtt_bsm_tx_node.cpp` 한 개만 |
| `target_link_libraries` | catkin + mosquitto + ffasn1-base + ffasn1-j2735-2020 (KSR1600.c **링크 안 함**, bsm_tx_node 와 동일 패턴) |
| `find_library(MOSQUITTO_LIB ...)` | mqtt_spat_rx_node 블록의 결과를 그대로 사용 (CMake cache 변수). 재호출 불필요 |
| `find_package(catkin REQUIRED COMPONENTS ...)` | **변경 불필요**. 이미 `roscpp std_msgs novatel_gps_msgs katech_custom_msgs` 모두 포함되어 있음 |

**package.xml**: 이미 `<build_depend>libmosquitto-dev</build_depend>` + `<exec_depend>libmosquitto</exec_depend>` 가 mqtt_spat_rx 도입 때 추가되어 있어 **수정 불필요**.

---

## §8. Launch 통합 (`launch/siheung.launch` 수정)

`mqtt_spat_rx_node` 블록 (line 39-51) 직후에 다음 노드 블록 1개 추가:

```xml
<!-- V2X TX BSM via MQTT (PC → Cloud V2N). mqtt_server arg 로 test/prod 전환 -->
<node name="mqtt_bsm_tx_node" type="mqtt_bsm_tx_node" pkg="siheung_v2x"
      respawn="true" output="screen">
    <param name="broker_host"  value="$(arg mqtt_host)" />
    <param name="broker_port"  value="$(arg mqtt_port)" />
    <param name="username"     value="$(arg mqtt_user)" />
    <param name="password"     value="$(arg mqtt_pass)" />
    <param name="topic"        value="V2N/1321103202/bsm" />
    <param name="client_id"    value="" />
    <param name="keepalive"    value="60" />
    <param name="qos"          value="0" />
    <param name="retain"       value="false" />
    <param name="vehicle_id"   value="EV01" />
    <param name="publish_rate" value="10.0" />
    <param name="verbose_first_msg" value="true" />
</node>
```

특성:

- 기존 `bsm_tx_node` (UDP TX) 블록은 **그대로 둔다**. 두 노드가 병행 동작 — UDP 6666 (OBU) + MQTT broker 양쪽으로 BSM 송신.
- `mqtt_server:=test` (기본) → `121.137.106.141:23312` / `ut_adcp` 자격으로 publish.
- `mqtt_server:=prod` → `192.168.255.173:10044` / `xcms-mtqq` 자격으로 publish.
- `respawn="true"`: broker 접속 실패가 누적되어 노드가 die 해도 자동 재시작. (단, libmosquitto 자체의 auto-reconnect 가 1차 방어선.)

---

## §9. 로그 사양

다음 4가지가 필수. 모두 한 줄.

| # | 시점 | 레벨 | 포맷 | 예시 |
|---|------|------|------|------|
| 1 | 생성자 끝 | INFO | `[mqtt_bsm_tx] started - broker=<host>:<port> topic=<topic> vehicle_id=<vid>` | `[mqtt_bsm_tx] started - broker=121.137.106.141:23312 topic=V2N/1321103202/bsm vehicle_id=EV01` |
| 2 | `handleConnect(rc)` | INFO | `[mqtt_bsm_tx] connected to broker (rc=<rc>)` | `[mqtt_bsm_tx] connected to broker (rc=0)` |
| 3 | 첫 publish 직후 (`verbose_first_msg=true`) | INFO | `[mqtt_bsm_tx] first BSM published total=<N> inner=<M> hex[0..16]=<16 bytes>` | `[mqtt_bsm_tx] first BSM published total=58 inner=42 hex[0..16]=04 00 ff 11 01 00 00 00 00 01 40 85 00 00 00 2a` |
| 4 | `handleDisconnect(rc != 0)` | WARN (THROTTLE 10s) | `[mqtt_bsm_tx] disconnected rc=<rc>, reconnecting` | `[mqtt_bsm_tx] disconnected rc=7, reconnecting` |

부수 로그 (선택):

- INSPVA 부재 시: `ROS_WARN_THROTTLE(2.0, "[mqtt_bsm_tx] waiting for Inspva");`
- UPER 인코딩 실패: `ROS_ERROR_THROTTLE(2.0, "[mqtt_bsm_tx] UPER encode failed: len=%ld bit_pos=%d msg='%s'", ...);`
- mosquitto_publish 실패: `ROS_WARN_THROTTLE(2.0, "[mqtt_bsm_tx] publish rc=%d (%s)", rc, mosquitto_strerror(rc));`
- 첫 mosquitto_connect 실패 (생성자 안): `ROS_ERROR("[mqtt_bsm_tx] mosquitto_connect rc=%d (%s) - will retry via loop", rc, mosquitto_strerror(rc));`

`verbose_first_msg=false` 일 때는 3번 로그 생략. 1, 2, 4 는 항상 출력.

---

## §10. 검증 시나리오

### §10.1 빌드

```bash
cd /home/ads/mcar_v13 && catkin_make --pkg siheung_v2x
```

성공 기준:

- `[100%] Built target mqtt_bsm_tx_node`
- 실행파일: `devel/lib/siheung_v2x/mqtt_bsm_tx_node` 생성
- 기존 `bsm_tx_node`, `mqtt_spat_rx_node`, `siheung_v2x_node`, `j2735_universal_node`, `tim_test` 모두 빌드 유지 (회귀 없음)

### §10.2 라이브 검증

**(1) bag replay** (별도 터미널):

```bash
roscore                                                            # term-1
rosbag play ~/bag_data/20260508/2026-05-08-11-19-30_2026-05-08-11-19-31_0.bag  # term-2
rosrun siheung_v2x mqtt_bsm_tx_node \
  _broker_host:=121.137.106.141 _broker_port:=23312 \
  _username:=ut_adcp _password:='ut_adcp123!@#' \
  _topic:=V2N/1321103202/bsm _vehicle_id:=EV01 \
  _publish_rate:=10.0 _verbose_first_msg:=true                     # term-3
```

bag 안 `/sensors/gps/inspva` 가 50 Hz 로 replay → 노드가 10 Hz 로 `V2N/1321103202/bsm` publish.

**(2) MQTT subscribe** (term-4):

```bash
mosquitto_sub -h 121.137.106.141 -p 23312 -u ut_adcp -P 'ut_adcp123!@#' \
              -t 'V2N/1321103202/bsm' -C 5 -F '%x'
```

`-C 5` 로 5개 페이로드만 받고 종료. `-F '%x'` 로 hex dump 출력.

**(3) Pass criteria** (5개 페이로드 hex dump 검사):

| # | 검증 항목 | 기준 |
|---|----------|------|
| ① | offset 0..3 = magic | 모든 5 packet 이 `04 00 ff 11` |
| ② | offset 7 = sequence 가 +1 씩 증가 | `00, 01, 02, 03, 04` (또는 노드 재시작 후 시작값에서 +1 stride) |
| ③ | offset 12..15 = BE uint32 가 후속 길이와 일치 | `payload_len - 16 == BE_uint32(offset[12..15])` |
| ④ | offset 16..17 = MessageFrame 시작 | 모든 5 packet 이 `00 14` (BSM msgID=20 의 UPER) |

추가 검증 (선택):

- offset 4..6 = `01 00 00` (5/5)
- offset 8..11 = `00 01 40 85` (5/5)
- 총 payload 길이 = 56~76 byte 범위 (헤더 16 + UPER 40~60)
- 10 Hz 발행 검증: `mosquitto_sub` 의 5개 수신 사이 wall-clock 간격 ≈ 100 ms (=0.4 sec for 5 messages)

**(4) ROS 토픽 부재 확인**: 본 노드는 ROS topic publish 안 함.

```bash
rostopic list | grep mqtt_bsm  # → (no output)
```

### §10.3 회귀 확인

- `bsm_tx_node` 의 UDP 송신 정상 (term-5 에서 `tcpdump -i any -nn 'udp port 6666'` 로 확인).
- `mqtt_spat_rx_node` 의 SPaT 수신 정상 (`rostopic echo /siheung_v2x/mqtt_spat`).
- `siheung_v2x_node` 의 OBU SPaT/SDSM 수신 정상 (`rostopic echo /siheung_spat`).

---

## §11. 부록 — 5 packet hex dump 예시

`mosquitto_sub -h ... -t 'V2N/1321103202/bsm' -C 5 -F '%x'` 의 **예상 출력**:

```
# packet 1 (seq=0x00, inner=42 byte)
0400ff11 010000 00 00014085 0000002a
00140d2a3041000000a8000000000000ff7f...

# packet 2 (seq=0x01, inner=42 byte, ~100ms 후)
0400ff11 010000 01 00014085 0000002a
00140d2a3041000000a8001f4400000ff7f...

# packet 3 (seq=0x02, inner=42 byte, ~200ms 후)
0400ff11 010000 02 00014085 0000002a
00140d2a3041000000a8001f5500000ff7f...

# packet 4 (seq=0x03, inner=43 byte, ~300ms 후)
0400ff11 010000 03 00014085 0000002b
00140d2a3041000000a8001f6600000ff7f00...

# packet 5 (seq=0x04, inner=42 byte, ~400ms 후)
0400ff11 010000 04 00014085 0000002a
00140d2a3041000000a8001f7700000ff7f...
```

해석 (packet 1 기준):

- `0400ff11` : magic ✓
- `010000` : version/reserved ✓
- `00` : sequence (첫 publish) ✓
- `00014085` : sender/site ID ✓
- `0000002a` : inner length = 42 byte (BE) ✓
- `0014...` : MessageFrame messageId=20 (BSM) UPER 시작 ✓
- 이후 42 byte = BSM coreData 의 lat/lon/elev/speed/heading 등이 UPER 패킹된 결과 (inspva 의 변화에 따라 패킷별로 약간씩 차이)

각 패킷의 inner length 가 정확히 42인 것은 보장되지 않는다 (BSM 필드 값에 따라 UPER 가 약간 가변). 보통 38~52 byte 범위. **§10.2 ③** 검증은 "실제 페이로드 길이 - 16 = offset[12..15] BE uint32" 가 성립하는지를 본다.

---

## 코드 변경 범위 박스

| 종류 | 경로 | 변경 분량 |
|------|------|-----------|
| **신규** | `src/v2x/siheung_v2x/src/mqtt_bsm_tx_node.cpp` | ~400~500 라인 (BSM fill 140 + mosquitto wrapper 150 + V2N 헤더 50 + main/생성자 등) |
| **수정** | `src/v2x/siheung_v2x/CMakeLists.txt` | +6 라인 (add_executable 1 + target_link_libraries 5) |
| **수정** | `launch/siheung.launch` | +14 라인 (node 블록 1개) |
| 변경 없음 | `src/v2x/siheung_v2x/src/bsm_tx_node.cpp` | (수정 금지 — BSM 인코딩은 file-local 복제) |
| 변경 없음 | `src/v2x/siheung_v2x/src/mqtt_spat_rx_node.cpp` | (수정 금지 — mosquitto 패턴은 file-local 복제) |
| 변경 없음 | `src/v2x/siheung_v2x/package.xml` | (`libmosquitto-dev` / `libmosquitto` 이미 포함) |

**총 신규 1 + 수정 2 = 3 파일**.

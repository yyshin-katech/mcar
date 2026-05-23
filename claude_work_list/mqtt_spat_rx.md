# MQTT SPaT RX 노드 사양서

대상: `src/v2x/siheung_v2x/` 패키지 신규 ROS 노드 — MQTT broker 의 V2N SPaT 토픽을 구독해 J2735 SPaT 을 디코드하고 `v2x_msgs/intersection_array_msg` 로 발행한다.

## MQTT broker 정보 (두 종류)

| 이름 | Host | Port | Username | Password | 비고 |
|------|------|------|----------|----------|------|
| **테스트 서버 (test)** | `121.137.106.141` | `23312` | `ut_adcp` | `ut_adcp123!@#` | 인터넷 공인망, 개발/검증용 |
| **실제 서버 (prod)** | `192.168.255.173` | `10044` | `xcms-mtqq` | `xcms123!` | 차량 사설망, 실제 운영용 |

launch arg `mqtt_server:=test|prod` 로 선택 (기본 `test`).

- 노드 이름 (제안): **`mqtt_spat_rx_node`**
- 소스 파일 (제안): `src/v2x/siheung_v2x/src/mqtt_spat_rx_node.cpp`
- 출력 토픽 (제안): **`/siheung_v2x/mqtt_spat`** (기본값, 파라미터로 변경 가능. 기존 `/siheung_spat` 과 격리)
- MQTT 라이브러리: **`libmosquitto-dev`** (C API). C++ 코드에서 `extern "C"` 헤더 그대로 사용
- 참고 작성일: 2026-05-22, 작성 환경 브랜치: `siheung_dev`

---

## 1. 기존 코드 분석

### 1.1 재사용 후보

`src/v2x/siheung_v2x/src/j2735_decode.cpp` 의 다음 단위를 활용한다.

| 단위 | 종류 | 재사용 방법 (권장) |
|------|------|--------------------|
| `class SpatDecoder` | 클래스 | 그대로 사용. 단, `publishSpat()` 는 멤버 함수라 `decode()` 와 분리되어 있어 외부에서도 호출 가능 |
| `SpatDecoder::publishSpat(j2735SPAT*, ros::Publisher&)` | 함수 | **권장 재사용 단위**. 입력 `j2735SPAT*`, 출력은 `ros::Publisher` 로 발행. 디코드된 SPaT 객체만 넘기면 `to_control_team` 기반 필터링·매칭 로직이 그대로 동작 |
| `SpatDecoder::decode(ReceivedMsg*, ...)` | 함수 | **그대로는 부적합**. 첫 5 바이트를 OBU 헤더로 가정해 스킵하기 때문 — MQTT 페이로드에는 이 헤더가 없을 가능성이 높다 (§2 참조). MQTT 노드는 자체 `decodeSpatPayload(buf, len)` 를 작성해 OBU 헤더 스킵 단계만 제외하고 동일 로직 수행 |
| `SpatDecoder::skipWsmpHeader()` | static 함수 | **재사용 가능**. WSMP 헤더(`0x03 0x80 + BER len`)는 MQTT 페이로드에도 포함될 가능성 있음 |
| `g_link_info` / `toControlTeamCallback` | 전역 + 콜백 | **재사용 권장**. `to_control_team` 기반 필터링은 두 노드 모두 동일하게 필요. 단, **두 실행파일이 별도 프로세스이므로 전역 변수는 공유되지 않음** → MQTT 노드도 자체적으로 `/localization/to_control_team` 을 구독해야 한다 |

### 1.2 재사용 전략 (의사결정)

**권장안: 헤더 추출 방식** — `SpatDecoder` 와 `g_link_info` 를 새 헤더 `include/siheung_v2x/spat_decoder.h` 로 추출하고, `j2735_decode.cpp` 와 `mqtt_spat_rx_node.cpp` 가 함께 include. 단, **본 사양서는 src/ 미수정 정책을 따르므로 코드 추출 작업은 별도 ticket 으로 분리**. 이번 mqtt_spat_rx_node 1차 버전은:

- **방안 A (코드 복제, 1차 권장)**: `publishSpat()` 의 핵심 로직 (intersection ID 필터, signalGroup 필터, MANUAVER → movementName 매핑, `v2x_msgs::intersection_array_msg` 빌드) 을 새 노드 cpp 안에 복제. 약 100 라인. 빠른 통합, j2735_decode.cpp 무수정.
- **방안 B (헤더 추출, 2차 리팩토링)**: 위의 헤더 추출 작업. `SpatDecoder` 정의가 한 곳에 모임. j2735_decode.cpp 수정 필요.

본 사양은 **방안 A 채택**. 단, 추후 리팩토링 여지를 남기기 위해 새 노드 내부 함수명을 `publishSpat`/`skipWsmpHeader` 와 동일하게 맞춘다.

### 1.3 핵심 시그니처

```
struct ReceivedMsg { std::vector<uint8_t> data; size_t len; };

class SpatDecoder {
    static size_t skipWsmpHeader(const uint8_t* body, size_t body_len);
    void publishSpat(j2735SPAT* spat, ros::Publisher& spat_pub);
    void decode(const ReceivedMsg* rmsg, ros::Publisher& spat_pub, ros::Publisher& sdsm_pub);
};
```

`publishSpat()` 가 참조하는 외부 의존성:
- `g_link_info` (mutex 보호, 전역) — `intersection_id`, `signal_group_id`, `manuaver`
- `ros::Time::now()`
- 메시지: `v2x_msgs::intersection_array_msg`, `v2x_msgs::intersection_msg`
- ASN.1 구조체: `j2735SPAT` (필드 `intersections.tab[].id.id`, `.states.tab[].signalGroup`, `.movementName`, `.state_time_speed.tab[].eventState`, `.timing.minEndTime`, `.moy`, `.timeStamp`)

→ MQTT 노드는 위 의존성을 동일하게 가짐 (libffasn1-j2735-2020.so + KSR1600 헤더, `to_control_team` 구독 추가).

### 1.4 OBU 5-byte 헤더 처리 vs MQTT 페이로드

`decode()` 함수는 **무조건** 첫 5 바이트를 OBU 헤더로 스킵한다 (line 244, 253). 헤더 구조:

```
[0] Frame Type   : 0=OBU→PC, 1=PC→OBU
[1] Seq No
[2] msg source   : 0=from RSU, 1=from Uu
[3] isMsgFrame   : 0=MessageFrame, 1=not MessageFrame
[4] Reserved
```

MQTT 페이로드 (broker → client) 는 OBU 디바이스를 거치지 않는 클라우드 경로이므로 **이 5-byte 헤더가 없을 가능성이 매우 높다**. 즉, MQTT 노드는 **OBU 헤더 스킵 단계를 건너뛰고 곧바로 MessageFrame UPER 디코드를 시도**해야 한다.

또한 `decode()` 는 `is_msg_frame` 분기에 따라:
- `is_msg_frame == 0` → `asn1_uper_decode(asn1_type_j2735MessageFrame, body, body_len, ...)` 시도. messageId 가 19(SPaT)면 처리
- `is_msg_frame == 1` → SDSM 직접 디코드 시도 → 실패하면 SPaT 직접 디코드(`asn1_type_j2735SPAT`) 시도

MQTT 노드는 OBU 헤더가 없으므로 분기 정보를 알 수 없다 → **MessageFrame UPER 디코드 우선 시도, 실패 시 SPaT UPER 직접 디코드로 fallback** 하는 2-step 전략을 사용한다 (§2.3).

### 1.5 출력 토픽 충돌 회피

기존 `siheung_v2x_node` 는 `/siheung_spat` 에 발행한다. MQTT 노드도 같은 토픽에 발행하면:

- 장점: downstream consumer (`to_control_team_demo.py` 등) 수정 불필요. OBU 가 없는 환경에서도 신호등 데이터 수급 가능
- 단점: 두 publisher 가 동시에 동작할 때 메시지 순서·중복 발생. 특히 같은 intersection 정보가 서로 다른 latency 로 두 번 도착하면 downstream 로직 (timing 비교) 이 혼란

**권장 (1차)**: 별도 토픽 **`/siheung_v2x/mqtt_spat`** 사용. 파라미터 `~spat_topic` 으로 노출. 운영자가 필요 시 launch 에서 `/siheung_spat` 으로 remap.

추후 (2차) downstream 이 안정되면, OBU vs MQTT 중 하나를 선택해 같은 `/siheung_spat` 으로 publish 하는 launch arg (`v2x_source: obu|mqtt`) 를 추가.

---

## 2. MQTT 페이로드 가정 + 페이로드 포맷 추정

### 2.1 가정 (Primary)

한국형 V2N SPaT 토픽 (`V2N/<intersectionId>/trf_drct/spat`) 페이로드는 **raw J2735 MessageFrame UPER 바이트** 로 가정한다.

근거:
- 한국 ITS V2N 서비스는 J2735 MessageFrame 표준 그대로 전달하는 것이 일반적
- broker → client 경로라 WSMP/OBU 헤더 불필요
- intersection ID 가 토픽 path 에 박혀 있으므로 페이로드는 순수 ASN.1

### 2.2 Sanity check (첫 메시지 수신 시 로그 + 헥스 덤프)

페이로드의 시작 바이트로 raw UPER MessageFrame 여부를 판별한다.

**J2735 MessageFrame UPER 인코딩 사양**:
- 첫 바이트 = `messageId` (UPER 로 encoding 되지만 16비트 INTEGER 범위라 보통 1 바이트로 시작하지 않고, 길이 prefix + value 형태)
- 실제 SPaT MessageFrame UPER 의 시작 패턴은 일반적으로 `0x00 0x13` (messageId=19) 부근 — 단, UPER 의 padding/alignment 영향으로 변동 가능
- 안전한 판별은 디코드 시도 후 `frame->messageId == 19` 확인

**Sanity check 코드 사양 (의사 코드)**:
```
on first message:
  log payload length
  hex dump first 16 bytes (ROS_INFO with %02x)
  log subsequent ASN1 decode result (messageId, intersections.count, etc.)
  log only once via static bool first_msg = true;
```

### 2.3 Fallback 시나리오

| 페이로드 종류 | 판별 | 처리 |
|---------------|------|------|
| Raw MessageFrame UPER | `asn1_uper_decode(j2735MessageFrame, buf, len)` 성공 & `messageId == 19` | `publishSpat()` 호출 |
| Raw SPaT UPER (MessageFrame 래핑 없음) | MessageFrame 디코드 실패 → SPaT 직접 디코드 시도 성공 | `publishSpat()` 호출 |
| WSMP 헤더 prefix (0x03 0x80 ...) | `skipWsmpHeader()` 후 위 두 단계 재시도 | 동일 |
| JSON 래핑 (base64 등) | 첫 바이트가 `{` (0x7B) 또는 ASCII printable | **1차 미지원**. ROS_WARN 로그 + 디버그 덤프. 운영 검증 후 2차에서 JSON 파서 추가 |
| 빈/짧은 페이로드 (< 4 바이트) | `len < 4` | ROS_WARN 후 스킵 |

### 2.4 messageId 값 확인

`j2735_decode.cpp` line 38: `#define J2735_MSG_ID_SPAT 19`. 이는 J2735-2020/2026 표준과 일치 (SPaT = 19, 0x13). MQTT 노드도 동일 매크로 정의.

### 2.5 페이로드 포맷 검증 후 결정 사항 (운영 중 확인 필요)

첫 수신 후 다음을 확인하고 본 사양 후속 버전에 반영:

- [ ] 페이로드 첫 16 바이트 헥스 패턴
- [ ] MessageFrame UPER 디코드 성공 여부
- [ ] WSMP 헤더 존재 여부
- [ ] 페이로드 평균 길이 (참고: 일반 SPaT UPER 100~500 바이트)
- [ ] 토픽 `V2N/1321103202/trf_drct/spat` 의 intersection ID(1321103202)가 SPaT 내부 `intersections.tab[0].id.id` 와 일치하는지

---

## 3. 노드 설계

### 3.1 클래스 구조

```
class MqttSpatRxNode {
public:
    MqttSpatRxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~MqttSpatRxNode();
    void spin();   // queue 소비 + ros::spinOnce 루프

private:
    // ROS
    ros::Publisher  spat_pub_;
    ros::Subscriber ctrl_sub_;

    // MQTT (mosquitto C handle)
    struct mosquitto* mosq_;
    std::string broker_host_, username_, password_, topic_, client_id_, spat_topic_out_;
    int broker_port_, keepalive_, qos_;

    // 페이로드 큐 (lock + cv)
    MessageQueue queue_;

    // SPaT decoder 로직 (publishSpat / skipWsmpHeader 복제)
    SpatDecoder decoder_;

    // mosquitto static callbacks (C API → C++ instance)
    static void onConnect (struct mosquitto*, void* userdata, int rc);
    static void onMessage (struct mosquitto*, void* userdata, const struct mosquitto_message* msg);
    static void onDisconnect(struct mosquitto*, void* userdata, int rc);

    // 인스턴스 메서드
    void handleConnect (int rc);
    void handleMessage (const struct mosquitto_message* msg);
    void handleDisconnect(int rc);
    void decodePayload (const std::vector<uint8_t>& buf);
};
```

### 3.2 생명주기 (생성자)

```
MqttSpatRxNode(nh, pnh):
  1. pnh.param 으로 broker_host, broker_port, username, password,
     topic, client_id, keepalive, qos, spat_topic_out 로드
  2. spat_pub_ = nh.advertise<v2x_msgs::intersection_array_msg>(spat_topic_out, 1);
  3. ctrl_sub_ = nh.subscribe("/localization/to_control_team", 1, toControlTeamCallback);
  4. mosquitto_lib_init();
  5. mosq_ = mosquitto_new(client_id_.c_str(), /*clean_session=*/true, this);
     - client_id 가 빈 문자열이면 nullptr 넘김 (broker 가 random id 할당)
  6. mosquitto_username_pw_set(mosq_, username_.c_str(), password_.c_str());
  7. mosquitto_connect_callback_set(mosq_, &MqttSpatRxNode::onConnect);
  8. mosquitto_message_callback_set(mosq_, &MqttSpatRxNode::onMessage);
  9. mosquitto_disconnect_callback_set(mosq_, &MqttSpatRxNode::onDisconnect);
 10. int rc = mosquitto_connect(mosq_, broker_host_.c_str(), broker_port_, keepalive_);
     - rc != MOSQ_ERR_SUCCESS 면 ROS_ERROR + ros::shutdown() 또는 reconnect_delay 후 재시도
 11. mosquitto_loop_start(mosq_);  // 백그라운드 네트워크 스레드 시작
```

소멸자:
```
~MqttSpatRxNode():
  mosquitto_disconnect(mosq_);
  mosquitto_loop_stop(mosq_, /*force=*/false);
  mosquitto_destroy(mosq_);
  mosquitto_lib_cleanup();
```

### 3.3 콜백 흐름

- **`onConnect(rc)`** : `ROS_INFO("[mqtt_spat_rx] connected to broker %s:%d (rc=%d)", host, port, rc);` rc==0 이면 `mosquitto_subscribe(mosq_, NULL, topic_.c_str(), qos_)` 호출. rc != 0 이면 ROS_ERROR.
- **`onMessage(msg)`** : `msg->payload` (void*), `msg->payloadlen` (int) 를 `std::vector<uint8_t>` 로 복사해 `queue_.push()`. **여기서 ROS publisher 를 직접 호출하지 않는다** (mosquitto 스레드에서 ROS API 호출 회피).
- **`onDisconnect(rc)`** : ROS_WARN 로깅. mosquitto 라이브러리가 `mosquitto_loop_start` 모드에서 자동 reconnect 시도하지만, 명시적으로 `mosquitto_reconnect_async()` 호출 가능 (rate-limit 위해 1초 sleep 후).

### 3.4 메인 루프 (`spin()`)

```
ros::Rate loop_rate(1000);
ReceivedMsg msg;
while (ros::ok()) {
    if (queue_.pop(msg))           // 10ms timeout 내장
        decodePayload(msg.data);
    ros::spinOnce();
    loop_rate.sleep();
}
```

기존 `j2735_decode.cpp` main 의 패턴을 그대로 따른다 (line 400~414).

### 3.5 `decodePayload(buf)` 의사 코드

```
decodePayload(const std::vector<uint8_t>& buf):
    if (buf.size() < 4) { ROS_WARN("payload too short"); return; }

    const uint8_t* body = buf.data();
    size_t body_len = buf.size();

    // (선택) WSMP 헤더 스킵
    size_t wsmp_off = SpatDecoder::skipWsmpHeader(body, body_len);
    if (wsmp_off > 0) { body += wsmp_off; body_len -= wsmp_off;
                        ROS_DEBUG("[mqtt_spat_rx] WSMP skipped (%zu B)", wsmp_off); }

    // 1차: MessageFrame UPER 디코드 시도
    ASN1Error err;
    void* frame_msg = nullptr;
    asn1_ssize_t ret = asn1_uper_decode(&frame_msg, asn1_type_j2735MessageFrame,
                                         body, body_len, &err);
    if (ret > 0 && frame_msg) {
        j2735MessageFrame* frame = (j2735MessageFrame*)frame_msg;
        if (frame->messageId == J2735_MSG_ID_SPAT) {
            j2735SPAT* spat = nullptr;
            void* spat_standalone = nullptr;
            if (frame->value.type != nullptr)
                spat = (j2735SPAT*)frame->value.u.data;
            else {
                ASN1String* raw = &frame->value.u.octet_string;
                asn1_ssize_t r2 = asn1_uper_decode(&spat_standalone, asn1_type_j2735SPAT,
                                                    raw->buf, raw->len, &err);
                if (r2 > 0 && spat_standalone) spat = (j2735SPAT*)spat_standalone;
            }
            if (spat) decoder_.publishSpat(spat, spat_pub_);
            if (spat_standalone) asn1_free_value(asn1_type_j2735SPAT, spat_standalone);
            asn1_free_value(asn1_type_j2735MessageFrame, frame_msg);
            return;
        }
        asn1_free_value(asn1_type_j2735MessageFrame, frame_msg);
        ROS_DEBUG_THROTTLE(5.0, "[mqtt_spat_rx] MessageFrame but not SPaT (msgId=%d)", frame->messageId);
    }

    // 2차 fallback: SPaT UPER 직접 디코드 시도
    void* spat_msg = nullptr;
    asn1_ssize_t r3 = asn1_uper_decode(&spat_msg, asn1_type_j2735SPAT,
                                        body, body_len, &err);
    if (r3 > 0 && spat_msg) {
        decoder_.publishSpat((j2735SPAT*)spat_msg, spat_pub_);
        asn1_free_value(asn1_type_j2735SPAT, spat_msg);
        return;
    }

    ROS_WARN_THROTTLE(5.0, "[mqtt_spat_rx] failed to decode payload (len=%zu)", body_len);
```

### 3.6 스레드 안전성

- mosquitto `loop_start` 가 별도 네트워크 스레드를 띄움 → `onMessage` 콜백은 그 스레드에서 호출됨
- `MessageQueue` 는 이미 `std::mutex` + `condition_variable` 으로 안전 (기존 코드 재사용 가능)
- ROS publisher 호출은 메인 스레드 (`spin()` 의 루프) 에서만 수행 → ros::spinOnce 와 안전 공존
- `g_link_info` 는 자체 mutex 보유. `toControlTeamCallback` (ROS 콜백 스레드) 와 `publishSpat` (메인) 사이 보호됨

대안 (성능 최적화 필요 시): `boost::lockfree::spsc_queue` 단일 producer / 단일 consumer. 단, MQTT 콜백 1 스레드 + ROS 메인 1 스레드라 SPSC 적합. 1차 버전은 std::mutex 큐 (기존 패턴) 로 충분.

### 3.7 재접속 전략

- mosquitto `loop_start` 모드는 기본적으로 disconnect 시 자동 reconnect 시도 (지수 backoff 가능 - `mosquitto_reconnect_delay_set(min=1, max=30, exponential=true)`)
- 생성자에서 `mosquitto_reconnect_delay_set(mosq_, 1, 30, true)` 호출
- `onDisconnect` 콜백에서 rc != 0 면 `ROS_WARN_THROTTLE(10, ...)` 로깅. 명시적 재호출 불필요 (loop_start 가 처리)

### 3.8 ROS 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `~broker_host` | string | `121.137.106.141` (test) / `192.168.255.173` (prod) | MQTT broker IP — launch arg `mqtt_server` 로 dispatch |
| `~broker_port` | int | `23312` (test) / `10044` (prod) | MQTT broker port |
| `~username` | string | `ut_adcp` (test) / `xcms-mtqq` (prod) | MQTT 인증 사용자 |
| `~password` | string | `ut_adcp123!@#` (test) / `xcms123!` (prod) | MQTT 인증 비밀번호 |
| `~topic` | string | `V2N/1321103202/trf_drct/spat` | 구독 토픽 |
| `~client_id` | string | `""` (broker 자동 할당) | MQTT client id |
| `~keepalive` | int | `60` | keepalive 초 |
| `~qos` | int | `0` | MQTT QoS level (0/1/2) |
| `~spat_topic` | string | `/siheung_v2x/mqtt_spat` | 출력 ROS 토픽 |
| `~reconnect_min` | int | `1` | reconnect_delay min sec |
| `~reconnect_max` | int | `30` | reconnect_delay max sec |
| `~verbose_first_msg` | bool | `true` | 첫 메시지 헥스 덤프 활성화 |

### 3.9 로그 메시지 (검증 verifier 매칭 기준)

다음 ROS_INFO 패턴을 반드시 포함 (verifier 가 grep 으로 확인):

```
ROS_INFO("[mqtt_spat_rx] started - broker=%s:%d topic=%s out=%s",
         broker_host_.c_str(), broker_port_, topic_.c_str(), spat_topic_out_.c_str());
ROS_INFO("[mqtt_spat_rx] connected to broker (rc=%d)", rc);
ROS_INFO("[mqtt_spat_rx] subscribed to %s qos=%d", topic_.c_str(), qos_);
ROS_INFO("[mqtt_spat_rx] first payload len=%d, head=%02x %02x %02x %02x ...", ...);
```

---

## 4. 빌드 설정 추가 (수정 명세)

### 4.1 `src/v2x/siheung_v2x/CMakeLists.txt` 추가 사항

```cmake
## ── MQTT SPaT RX ────────────────────────────────────────────
## mqtt_spat_rx_node: MQTT broker(V2N)에서 SPaT 수신
find_library(MOSQUITTO_LIB mosquitto)
if(NOT MOSQUITTO_LIB)
  message(FATAL_ERROR "libmosquitto not found. Install: sudo apt install libmosquitto-dev")
endif()

add_executable(mqtt_spat_rx_node
  src/mqtt_spat_rx_node.cpp
  src/ffasn1-j2735-2026-KSR1600.c
)
target_link_libraries(mqtt_spat_rx_node
  ${catkin_LIBRARIES}
  ${MOSQUITTO_LIB}
  ffasn1-base
  ffasn1-j2735-2020
)
```

주의:
- `ffasn1-j2735-2026-KSR1600.c` 를 같이 빌드해야 KSR1600 type registrar(`asn1_type_j2735SPAT` 등) 가 link 됨 — 기존 `siheung_v2x_node` 와 동일 패턴 (line 49)
- `find_library(MOSQUITTO_LIB mosquitto)` 는 시스템 `/usr/lib/x86_64-linux-gnu/libmosquitto.so` 를 탐지. pkg-config 미설치 환경 (현재 머신 상태) 에서도 안전
- include path 는 `/usr/include/mosquitto.h` 가 표준 경로이므로 추가 `include_directories` 불필요

### 4.2 `src/v2x/siheung_v2x/package.xml` 추가 사항

```xml
<build_depend>libmosquitto-dev</build_depend>
<exec_depend>libmosquitto</exec_depend>
```

`rosdep` 의 `apt key` 매핑:
- `libmosquitto-dev` → ubuntu/debian: `libmosquitto-dev`
- `libmosquitto` → runtime shared lib package

만약 rosdep DB 에 키가 없다고 보고되면 `<build_depend>` / `<exec_depend>` 그대로 두되, README 또는 launch 노트에 `sudo apt install libmosquitto-dev` 안내 추가.

### 4.3 헤더 include 위치 (`mqtt_spat_rx_node.cpp` 최상단)

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <vector>
#include <cstring>

extern "C" {
#include <mosquitto.h>
}

#include "ffasn1-j2735-2026-KSR1600.h"
#include "asn1defs.h"

#include <v2x_msgs/intersection_msg.h>
#include <v2x_msgs/intersection_array_msg.h>
#include <mmc_msgs/to_control_team_from_local_msg.h>
```

`mosquitto.h` 는 이미 C linkage 로 작성되어 있지만 안전을 위해 `extern "C"` 로 감싼다.

---

## 5. Launch 통합 명세

### 5.1 `launch/siheung.launch` 추가 사항

기존 노드 뒤에 다음 블록 추가:

```xml
<!-- V2X RX via MQTT (Cloud V2N → PC) -->
<node name="mqtt_spat_rx_node" type="mqtt_spat_rx_node" pkg="siheung_v2x"
      respawn="true" output="screen">
    <param name="broker_host"  value="121.137.106.141" />
    <param name="broker_port"  value="23312" />
    <param name="username"     value="ut_adcp" />
    <param name="password"     value="ut_adcp123!@#" />
    <param name="topic"        value="V2N/1321103202/trf_drct/spat" />
    <param name="client_id"    value="" />
    <param name="keepalive"    value="60" />
    <param name="qos"          value="0" />
    <param name="spat_topic"   value="/siheung_v2x/mqtt_spat" />
    <param name="verbose_first_msg" value="true" />
</node>
```

### 5.2 보안 메모

사용자가 명시적으로 launch 에 평문 password 를 요청. 다음은 향후 보안 권장 (이번 1차 버전엔 미적용):
- launch 에는 placeholder 만 두고 `roslaunch launch/siheung.launch broker_password:=$(cat ~/.mqtt_pass)` 형태로 전달
- 또는 ROS private namespace `~mqtt_pass_file:=/etc/mcar/mqtt.pass` 로 파일 경로 받아 노드 내부에서 읽기
- launch 파일을 git 추적 시 ignore 권장

### 5.3 launch 동작 시나리오

- `siheung_v2x_node` (UDP 9999) 와 `mqtt_spat_rx_node` 가 **공존**. 출력 토픽 분리 (`/siheung_spat` vs `/siheung_v2x/mqtt_spat`) → 충돌 없음
- OBU 없는 PC-only 검증 환경에서는 `siheung_v2x_node` 출력 없음, MQTT 노드만 발행
- 운영자가 `siheung_v2x_node` 라인을 주석 처리하면 MQTT 전용

---

## 6. 검증 절차 (verifier 사용)

### 6.1 사전 조건

```bash
# 시스템 패키지 설치 확인
dpkg -l | grep libmosquitto-dev
ls /usr/include/mosquitto.h
```

미설치 시: `sudo apt install libmosquitto-dev libmosquitto1`

### 6.2 빌드 통과

```bash
cd /home/sim/mcar
catkin_make --pkg siheung_v2x 2>&1 | tee /tmp/mqtt_spat_build.log
echo "exit=$?"
grep -E "(error|Error|FATAL|undefined reference)" /tmp/mqtt_spat_build.log
```

기대:
- exit 코드 0
- `Linking CXX executable /home/sim/mcar/devel/lib/siheung_v2x/mqtt_spat_rx_node` 로그
- error grep 결과 빈 출력
- `find_library(MOSQUITTO_LIB ...)` 결과 `FATAL_ERROR` 미발생

### 6.3 launch 파싱

```bash
source /home/sim/mcar/devel/setup.bash
roslaunch --files /home/sim/mcar/launch/siheung.launch
# 또는 dry-run 으로
roslaunch --dump-params /home/sim/mcar/launch/siheung.launch > /tmp/siheung_params.txt
grep mqtt /tmp/siheung_params.txt
```

기대:
- `~broker_host`, `~broker_port`, `~topic`, `~spat_topic` 등 param 출력
- launch XML 파싱 에러 없음

### 6.4 노드 단독 실행 — broker 접속 로그 확인

```bash
roscore &
sleep 2
rosrun siheung_v2x mqtt_spat_rx_node \
    _broker_host:=121.137.106.141 \
    _broker_port:=23312 \
    _username:=ut_adcp \
    _password:='ut_adcp123!@#' \
    _topic:=V2N/1321103202/trf_drct/spat \
    _spat_topic:=/siheung_v2x/mqtt_spat \
    2>&1 | tee /tmp/mqtt_run.log &

sleep 5
grep "connected to broker" /tmp/mqtt_run.log
grep "subscribed to" /tmp/mqtt_run.log
```

기대 로그 패턴:
```
[ INFO] [...] [mqtt_spat_rx] started - broker=121.137.106.141:23312 topic=V2N/1321103202/trf_drct/spat out=/siheung_v2x/mqtt_spat
[ INFO] [...] [mqtt_spat_rx] connected to broker (rc=0)
[ INFO] [...] [mqtt_spat_rx] subscribed to V2N/1321103202/trf_drct/spat qos=0
```

네트워크 차단/방화벽 환경에서는 connect 실패 로그(`rc != 0`) 가 나올 수 있음 — 이는 코드 결함이 아닌 환경 이슈.

### 6.5 SPaT 메시지 수신 확인 (네트워크 가능 시)

```bash
rostopic info /siheung_v2x/mqtt_spat
rostopic hz /siheung_v2x/mqtt_spat
rostopic echo -n 3 /siheung_v2x/mqtt_spat
```

기대:
- `Type: v2x_msgs/intersection_array_msg`
- `Publishers: /mqtt_spat_rx_node`
- rate ≈ 1~10 Hz (V2N 서비스 주기에 따라)
- echo 출력에 `IntersectionID`, `MovementPhaseStatus`, `TimeChangeDetails` 필드 값 존재

### 6.6 to_control_team 연동 확인

```bash
# to_control_team_demo.py 또는 시뮬레이터 별도 실행 후
rostopic echo /localization/to_control_team -n 1 | grep -E "(IntersectionID|signalGroupID|MANUAVER)"
# mqtt 노드 로그에 매칭 결과 출력 확인
grep "\[SPaT\] IntID=" /tmp/mqtt_run.log
```

기대 로그 (publishSpat 내부 ROS_INFO):
```
[SPaT] IntID=1321103202 SigGrp=2 Move=STRAIGHT Phase=STOP(red) minEnd=12.3s
```

### 6.7 페이로드 sanity check 결과 검토

```bash
grep "first payload" /tmp/mqtt_run.log
```

기대: 첫 메시지 헥스 덤프 1줄. 패턴이 예상과 다르면 §2.5 결정 사항을 운영 노트에 반영.

### 6.8 종료 처리 확인

```bash
# 노드 중지
rosnode kill /mqtt_spat_rx_node
sleep 2
# coredump 또는 mosquitto destroy 누락 시 valgrind 로 별도 확인 (선택)
```

기대: 클린 종료, 다음 실행 시 client_id 충돌 없음 (clean_session=true 이라 자동 정리).

### 6.9 추가 점검 항목

- `/siheung_spat` 과 `/siheung_v2x/mqtt_spat` 가 **별도 토픽** 으로 존재하는지 (`rostopic list | grep spat`)
- 두 노드 동시 가동 시 ROS publisher 충돌 없는지
- `respawn="true"` 가 동작하는지 (`pkill -9 mqtt_spat_rx_node` 후 ros launch 가 재시작하는지)
- `package.xml` 의 `<build_depend>libmosquitto-dev</build_depend>` 가 rosdep check 통과하는지: `rosdep check siheung_v2x`

---

## 부록 A: 코드 복제 vs 헤더 추출 비교

| 항목 | 방안 A (복제, 1차) | 방안 B (헤더 추출, 2차) |
|------|---------------------|--------------------------|
| j2735_decode.cpp 수정 | 없음 | 있음 (SpatDecoder 헤더로 이동) |
| 코드 중복 | ~100 라인 publishSpat 중복 | 없음 |
| 빌드 영향도 | 신규 executable 만 빌드 | siheung_v2x_node 도 재빌드 |
| 리스크 | 낮음 (기존 노드 무영향) | 중간 (헤더/링크 충돌 가능) |
| 권장 시점 | 즉시 (1차 통합) | MQTT 운영 안정화 후 |

## 부록 B: 향후 확장

- BSM / SDSM / MAP 도 MQTT 토픽으로 들어올 가능성 → MQTT 노드를 `MqttV2xRxNode` 로 일반화하고 토픽별 messageId 분기
- 인증서 기반 TLS (MQTT 8883 포트) 지원: `mosquitto_tls_set()`
- 메트릭 발행: `/diagnostic/mqtt_spat` 토픽 (수신 rate, 디코드 성공률, 마지막 수신 시각)

# BSM TX 노드 사양서 (siheung_v2x/bsm_tx_node)

작성일: 2026-05-21 / 브랜치: siheung_dev / 작업물: 사양만, 코드 변경 없음.

대상: `src/v2x/siheung_v2x/` 안에 J2735 BSM 송신용 ROS C++ 노드 신규 추가. `katri_obu_interface` 는 참고만 함.

목표: J2735 `BasicSafetyMessage` 를 `MessageFrame` 으로 감싸 **UPER 인코딩** 후 ROS 토픽으로 발행 (1단계). 추후 UDP 송신 옵션 추가 (2단계).

---

## 1. ASN.1 타입 결정

**선택: `ffasn1-j2735-2026-KSR1600.h` (KSR1600 변형)**

근거 (기존 빌드 충돌 분석):

`src/v2x/siheung_v2x/CMakeLists.txt:41-53`

```cmake
## siheung_v2x_node: SPaT(KSR1600) + SDSM(2020)
## sdsm_decode.cpp → ffasn1-j2735-2020 (별도 번역 단위, 헤더 충돌 방지)
## j2735_decode.cpp + KSR1600.c → ffasn1-j2735-2026-KSR1600
add_executable(${PROJECT_NAME}_node
  src/j2735_decode.cpp
  src/sdsm_decode.cpp
  src/ffasn1-j2735-2026-KSR1600.c
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ffasn1-base
  ffasn1-j2735-2020
)
```

- `src/j2735_decode.cpp:16` 는 `"ffasn1-j2735-2026-KSR1600.h"` 를 include 하고 SPaT/MessageFrame 처리.
- `src/sdsm_decode.cpp:4` 는 `"ffasn1-j2735-2020.h"` 를 include — 헤더 충돌을 막기 위해 **번역 단위 분리**.
- `src/j2735_universal_decoder.cpp:17` 와 `src/tim_decode_test.cpp:6` 는 2020 헤더 사용. 이쪽은 별도 executable.

**BSM TX 노드는 `j2735_decode.cpp` (KSR1600) 와 동일 그룹으로 가는 게 가장 자연스럽다.**
- KSR1600 가 한국 V2X 규격이고, 같은 OBU 와 송수신을 주고받으므로 송신 BSM 도 동일 규격으로 보내야 RX 측이 디코딩한다.
- 단, **`bsm_tx_node` 는 별도 executable** 로 만들어 `siheung_v2x_node` 와 분리한다 (RX/TX 책임 분리). 같은 KSR1600 헤더를 쓰지만 다른 .cpp 라서 충돌 없음.
- 링크 라이브러리: `ffasn1-base`, `ffasn1-j2735-2020` (2020.so 가 KSR1600 변형까지 export 함 — 이미 검증됨).
- KSR1600 generated source `src/ffasn1-j2735-2026-KSR1600.c` 는 **링크하지 않는다**. 이미 `siheung_v2x_node` 에서 컴파일되며 multi-symbol 충돌 발생 위험. → 헤더만 include 하고 type 메타데이터는 **`.so` 의 export 심볼**에서 가져오는 것으로 가정 (다른 executable 인 `j2735_universal_node`, `tim_test` 도 동일한 패턴이며, 2020 헤더 + .so 만으로 빌드 성공).

→ **BSM TX 도 헤더 + .so 패턴**: `#include "ffasn1-j2735-2026-KSR1600.h"` + `target_link_libraries(... ffasn1-base ffasn1-j2735-2020)`. KSR1600.c 는 **링크 X**.

검증 필요사항 (빌드 시 확인): `libffasn1-j2735-2020.so` 에 `asn1_type_j2735BasicSafetyMessage` / `asn1_type_j2735MessageFrame` 심볼이 export 되어 있는지 (해 사실은 사용자가 미리 확인한 "libffasn1-j2735-2020.so 와 libffasn1-j2735-2026-KSR1600.so 둘 다 BSM 타입 정의 export 함"에서 보장됨). 만약 충돌 시 KSR1600.so 로 링크 교체.

---

## 2. j2735MessageFrame 구조 + BSM 래핑 방법

### MessageFrame 정의

`include/ffasn1-j2735-2026-KSR1600.h:11-21`

```c
typedef int j2735DSRCmsgID;
extern const ASN1CType asn1_type_j2735DSRCmsgID[];

typedef struct j2735MessageFrame {
  j2735DSRCmsgID messageId;
  ASN1OpenType value;
} j2735MessageFrame;

extern const ASN1CType asn1_type_j2735MessageFrame[];
```

### ASN1OpenType 정의

`include/asn1defs.h:93-101`

```c
typedef struct ASN1OpenType {
    ASN1CType *type; /* if the 'type' is NULL, 'octet_string' contains
                        the raw content. Otherwise, 'data' is the
                        corresponding value. */
    union {
        void *data;
        ASN1String octet_string;
    } u;
} ASN1OpenType;
```

### BSM 래핑 방법 (선택: type-pointer 방식)

두 가지 채움 방식이 가능. **권장은 (A)**.

**(A) Type-pointer 방식 (인코딩 한 번)**

`value.type` 에 `asn1_type_j2735BasicSafetyMessage` 를 넣고 `value.u.data` 에 채운 `j2735BasicSafetyMessage*` 를 넣는다. 라이브러리가 OpenType 인코딩 중에 자동으로 inner BSM 을 UPER 로 인코딩.

```c
j2735BasicSafetyMessage bsm{};
fillBSM(bsm, ...);

j2735MessageFrame mf{};
mf.messageId = 20;                                  // BSM
mf.value.type = (ASN1CType*)asn1_type_j2735BasicSafetyMessage;
mf.value.u.data = &bsm;

uint8_t* buf = nullptr;
asn1_ssize_t len = asn1_uper_encode(&buf, asn1_type_j2735MessageFrame, &mf);
// len > 0 이면 buf[0..len-1] 가 인코딩된 MessageFrame.
asn1_free(buf);
```

**(B) Pre-encoded octet_string 방식 (인코딩 두 번)**

BSM 만 먼저 UPER 인코딩 → octet_string 으로 감쌈.

```c
uint8_t* bsm_buf = nullptr;
asn1_ssize_t bsm_len = asn1_uper_encode(&bsm_buf, asn1_type_j2735BasicSafetyMessage, &bsm);

j2735MessageFrame mf{};
mf.messageId = 20;
mf.value.type = nullptr;
mf.value.u.octet_string.buf = bsm_buf;
mf.value.u.octet_string.len = (size_t)bsm_len;

uint8_t* frame_buf = nullptr;
asn1_ssize_t frame_len = asn1_uper_encode(&frame_buf, asn1_type_j2735MessageFrame, &mf);
asn1_free(bsm_buf);
asn1_free(frame_buf);
```

- 디코더 측 패턴 (`src/j2735_decode.cpp:284-295`) 이 양쪽 모두 처리하므로 어느 방식이든 RX 측 호환.
- 효율: **(A) 권장** (메모리 할당 1회, 코드 단순).

### messageId 상수

J2735 표준에서 `basicSafetyMessage = 20 (= 0x14)`. SPaT 가 19, BSM 이 20.

```c
#define J2735_MSG_ID_BSM 20  // 0x14
```

(기존 코드 `src/j2735_decode.cpp:38` 가 `J2735_MSG_ID_SPAT 19` 로 사용 중인 동일 패턴.)

### 인코더 선택

`include/asn1defs.h:319-330`

```c
asn1_ssize_t asn1_uper_encode(uint8_t **pbuf, const ASN1CType *p, const void *data);
asn1_ssize_t asn1_uper_encode2(uint8_t **pbuf, const ASN1CType *p, const void *data, ASN1Error *err);
```

`include/asn1defs.h:344-345`

```c
asn1_ssize_t asn1_ber_encode(uint8_t **pbuf, const ASN1CType *p, const void *data, const ASN1BERParams *params);
```

- **권장: UPER** (`asn1_uper_encode2`). 기존 SPaT RX 도 UPER 사용 (`src/j2735_decode.cpp:272`). 한국 V2X 표준 (KISA/TTA) 도 UPER.
- BER 옵션도 가능 (TIM/SDSM 일부 패킷이 BER 사용 — `src/j2735_universal_decoder.cpp:168, 195`). 호환성 확인 시 fallback.

→ **선택: `asn1_uper_encode2`** (error 정보를 받을 수 있음).

---

## 3. j2735BSMcoreData 최소 필드 매핑

### BSMcoreData 정의

`include/ffasn1-j2735-2026-KSR1600.h:199-214`

```c
typedef struct j2735BSMcoreData {
  j2735MsgCount msgCnt;
  j2735TemporaryID id;
  j2735DSecond secMark;
  j2735Latitude lat;
  j2735Longitude Long;
  j2735Elevation elev;
  j2735PositionalAccuracy accuracy;
  j2735TransmissionState transmission;
  j2735Speed speed;
  j2735Heading heading;
  j2735SteeringWheelAngle angle;
  j2735AccelerationSet4Way accelSet;
  j2735BrakeSystemStatus brakes;
  j2735VehicleSize size;
} j2735BSMcoreData;
```

### 각 필드 typedef (KSR1600 헤더)

| 필드 typedef | 헤더 위치 | 정의 |
|---|---|---|
| `j2735MsgCount` | line 31 | `typedef int j2735MsgCount;` |
| `j2735TemporaryID` | line 35 | `typedef ASN1String j2735TemporaryID;` (OCTET STRING) |
| `j2735DSecond` | line 39 | `typedef int j2735DSecond;` |
| `j2735Latitude` | line 43 | `typedef int j2735Latitude;` |
| `j2735Longitude` | line 47 | `typedef int j2735Longitude;` |
| `j2735Elevation` | line 51 | `typedef int j2735Elevation;` |
| `j2735SemiMajorAxisAccuracy` | line 55 | `typedef int` |
| `j2735SemiMinorAxisAccuracy` | line 59 | `typedef int` |
| `j2735SemiMajorAxisOrientation` | line 63 | `typedef int` |
| `j2735PositionalAccuracy` | line 67-71 | `struct { semiMajor, semiMinor, orientation }` |
| `j2735TransmissionState` | line 76-85 | enum: `neutral, park, forwardGears, reverseGears, reserved1..3, unavailable` |
| `j2735Speed` | line 89 | `typedef int` |
| `j2735Heading` | line 93 | `typedef int` |
| `j2735SteeringWheelAngle` | line 97 | `typedef int` |
| `j2735Acceleration` | line 101 | `typedef int` |
| `j2735VerticalAcceleration` | line 105 | `typedef int` |
| `j2735YawRate` | line 109 | `typedef int` |
| `j2735AccelerationSet4Way` | line 113-118 | `struct { Long, lat, vert, yaw }` (대문자 Long 주의) |
| `j2735BrakeAppliedStatus` | line 123 | `typedef ASN1BitString` |
| `j2735TractionControlStatus` | line 127-132 | enum |
| `j2735AntiLockBrakeStatus` | line 136-141 | enum |
| `j2735StabilityControlStatus` | line 145-150 | enum |
| `j2735BrakeBoostApplied` | line 154-158 | enum |
| `j2735AuxiliaryBrakeStatus` | line 162-167 | enum |
| `j2735BrakeSystemStatus` | line 171-178 | `struct { wheelBrakes, traction, abs, scs, brakeBoost, auxBrakes }` |
| `j2735VehicleWidth` | line 183 | `typedef int` |
| `j2735VehicleLength` | line 187 | `typedef int` |
| `j2735VehicleSize` | line 191-194 | `struct { width, length }` |

> 주: 헤더에 INTEGER 범위 매크로(`HAS_LOW/HIGH`)는 없지만 J2735 표준에 따라 아래 표의 범위를 그대로 사용한다 (인코더가 범위 위반 시 error 반환). 사양상 범위는 SAE J2735-2016/2020 동일.

### 필드 매핑 표

| 필드 | C 타입 | 표준 단위/스케일/범위 | 소스 토픽/메시지 (소스 필드 인용) | 매핑식 / 비고 |
|---|---|---|---|---|
| `msgCnt` | int | 0..127, 100ms 주기마다 +1 wrap | (자체 카운터) | `static uint8_t cnt = 0; bsm.msgCnt = cnt++ & 0x7F;` |
| `id` | ASN1String (OCTET STRING SIZE(4)) | 4 bytes 차량 ID | 파라미터 `~vehicle_id` (string "0xAABBCCDD" or hex) | `id.buf = {0xAA,0xBB,0xCC,0xDD}; id.len = 4;` 4-byte 정확히. |
| `secMark` | int | 0..65535, ms within current minute (65535 = unavailable) | `ros::Time::now()` | `auto t = ros::Time::now(); bsm.secMark = (t.toNSec() / 1000000ULL) % 60000;` |
| `lat` | int | 1/10 microdegree, range -900000000..900000001 (= 900000001 unavailable) | `/ublox/navpvt` 의 `int32 lat` (`NavPVT.msg:72`, 단위: deg / 1e-7) | **동일 단위!** `bsm.lat = navpvt.lat;` (변환 불필요). 범위 클램프 권장. |
| `Long` | int | 1/10 microdegree, range -1799999999..1800000001 | `/ublox/navpvt` 의 `int32 lon` (`NavPVT.msg:71`, 단위: deg / 1e-7) | **동일 단위.** `bsm.Long = navpvt.lon;` |
| `elev` | int | 0.1 m, range -4096..61439 (61440 unavailable) | `/ublox/navpvt` 의 `int32 hMSL` (`NavPVT.msg:74`, 단위: mm) | `bsm.elev = navpvt.hMSL / 100;` (mm → 0.1m). 범위 클램프. |
| `accuracy.semiMajor` | int | 0.05 m / step, 0..254, 255 = unavailable | `/ublox/navpvt` 의 `uint32 hAcc` (`NavPVT.msg:75`, 단위: mm) | `auto a = (navpvt.hAcc / 50); accuracy.semiMajor = a > 254 ? 255 : a;` |
| `accuracy.semiMinor` | int | 동상 | `hAcc` (등방 가정) | `accuracy.semiMinor = accuracy.semiMajor;` |
| `accuracy.orientation` | int | 0.0054932479 deg / step, 0..65534 (65535 unavailable) | (불명, 기본값 사용) | `accuracy.orientation = 65535;` (unavailable). 추후 covariance 매트릭스에서 도출 가능. |
| `transmission` | enum | `neutral=0/park=1/forwardGears=2/reverseGears=3/.../unavailable=7` | `/sensors/v_can` 의 `uint8 gear_status` (`v_can_msg.msg:5`) | mapping (4-2절 참고). 기본 forwardGears. |
| `speed` | int | 0.02 m/s, 0..8191 (8191 unavailable) | `/ublox/navpvt` 의 `int32 gSpeed` (`NavPVT.msg:81`, 단위: mm/s) | `auto s = navpvt.gSpeed / 20; bsm.speed = s < 0 ? 0 : (s > 8190 ? 8191 : s);` (8190으로 clamp 권장하여 unavailable 충돌 회피.) 또는 v_can wheel_speed 평균 사용 가능. |
| `heading` | int | 0.0125 deg, 0..28800 (28800 unavailable) | `/ublox/navpvt` 의 `int32 heading` (`NavPVT.msg:82`, 단위: deg / 1e-5) | `auto h = navpvt.heading; if (h<0) h += 36000000; bsm.heading = (int)((h * 1e-5) / 0.0125 + 0.5);` (모션 방향). `headVeh` (line 90) 가 있으면 우선. |
| `angle` | int | 1.5 deg, -126..127 (127 = unavailable / fully right) | `/sensors/v_can` 의 `float64 steering_angle` (`v_can_msg.msg:17`, 단위: deg, 좌+/우-) | `int a = (int)round(steering_angle / 1.5); bsm.angle = clamp(a, -126, 126);` |
| `accelSet.Long` | int | 0.01 m/s², -2000..2001 (2001 unavailable) | `/sensors/v_can` 의 `float64 long_acceleration` (`v_can_msg.msg:36`, 단위: m/s²) | `int al = (int)round(long_acceleration * 100); clamp(al, -2000, 2000);` |
| `accelSet.lat` | int | 동상 | `/sensors/v_can` 의 `float64 lat_acceleration` (`v_can_msg.msg:37`) | 동상 |
| `accelSet.vert` | int | 0.02 G, -127..127 (-127 unavailable) | (불명, 기본값) | `accelSet.vert = -127;` |
| `accelSet.yaw` | int | 0.01 deg/s, -32767..32767 | `/sensors/v_can` 의 `float64 yaw_rate` (`v_can_msg.msg:40`, 단위: deg/s) | `int y = (int)round(yaw_rate * 100); clamp(y, -32767, 32767);` |
| `brakes.wheelBrakes` | ASN1BitString | SIZE(5): bit0=unavailable, bit1=leftFront, bit2=leftRear, bit3=rightFront, bit4=rightRear | `/sensors/v_can` 의 `float64 brake_pedal_pos` (`v_can_msg.msg:13`) → 임계값 | bit pattern: `brake_pedal_pos > 0.05 ? 0b01111000 : 0b10000000` (4륜 brake applied or unavailable). `len = 5`. |
| `brakes.traction` | enum | `unavailable/off/on/engaged` | 없음 | `unavailable` (0). |
| `brakes.abs` | enum | 동상 | 없음 | `unavailable`. |
| `brakes.scs` | enum | 동상 | 없음 | `unavailable`. |
| `brakes.brakeBoost` | enum | `unavailable/off/on` | 없음 | `unavailable`. |
| `brakes.auxBrakes` | enum | `unavailable/off/on/reserved` | 없음 | `unavailable`. |
| `size.width` | int | cm, 0..1023 (0 unavailable) | 파라미터 `~vehicle_width_cm` (기본 190 = IONIQ5 폭 1.890 m) | `bsm.size.width = vehicle_width_cm;` |
| `size.length` | int | cm, 0..4095 | 파라미터 `~vehicle_length_cm` (기본 464 = IONIQ5 4.635 m) | `bsm.size.length = vehicle_length_cm;` |

### partII / regional

`include/ffasn1-j2735-2026-KSR1600.h:257-263`

```c
typedef struct j2735BasicSafetyMessage {
  j2735BSMcoreData coreData;
  BOOL partII_option;
  j2735BasicSafetyMessage_1 partII;
  BOOL regional_option;
  j2735BasicSafetyMessage_2 regional;
} j2735BasicSafetyMessage;
```

→ 1단계는 `partII_option = FALSE; regional_option = FALSE;` 로 둠. (`partII.tab=nullptr; partII.count=0;` 마찬가지.)

---

## 3-1. 실제 토픽/메시지 인용

### `/ublox/navpvt` — `ublox_msgs/NavPVT.msg` (행 번호 = `src/sensing/gps/ublox/ublox_msgs/msg/NavPVT.msg`)

```
line 71:  int32 lon               # Longitude [deg / 1e-7]
line 72:  int32 lat               # Latitude [deg / 1e-7]
line 73:  int32 height            # Height above Ellipsoid [mm]
line 74:  int32 hMSL              # Height above mean sea level [mm]
line 75:  uint32 hAcc             # Horizontal Accuracy Estimate [mm]
line 76:  uint32 vAcc             # Vertical Accuracy Estimate [mm]
line 81:  int32 gSpeed            # Ground Speed (2-D) [mm/s]
line 82:  int32 heading           # Heading of motion 2-D [deg / 1e-5]
line 83:  uint32 sAcc             # Speed Accuracy Estimate [mm/s]
line 84:  uint32 headAcc          # Heading Accuracy Estimate [deg / 1e-5]
line 90:  int32 headVeh           # Heading of vehicle (2-D) [deg / 1e-5]
```

**중요:** ublox `lon/lat` 의 단위 (deg / 1e-7 = 1/10 microdeg) 가 BSM `Latitude/Longitude` 의 단위 (1/10 microdeg) **완전 동일**. 직접 대입 가능.

토픽 이름: 일반적으로 `/ublox/navpvt` (siheung_v2x_node 는 navpvt 를 안 쓰지만 다른 노드 — `localization` — 가 사용 중). 토픽명은 launch 파일에서 remap 확인 필요. 파라미터 `~navpvt_topic` 로 노출 권장.

### `/sensors/v_can` — `katech_custom_msgs/v_can_msg.msg`

```
line 5:   uint8 gear_status                # GearInfo (ID 117)
line 13:  float64 brake_pedal_pos          # LongitudinalInfo
line 14:  float64 brake_pressure
line 17:  float64 steering_angle           # SteeringInfo (deg)
line 30:  float64 wheel_speed_fl           # WheelInfo
line 31:  float64 wheel_speed_fr
line 32:  float64 wheel_speed_rl
line 33:  float64 wheel_speed_rr
line 36:  float64 long_acceleration        # DynamicInfo (m/s²)
line 37:  float64 lat_acceleration
line 40:  float64 yaw_rate                 # (deg/s 또는 rad/s — DBC 확인 필요)
```

`yaw_rate` 의 단위는 DBC 매핑에 의존 (V_CAN_Release.dbc). 가정: deg/s. 다를 경우 매핑식 조정.

`gear_status` 의 값 매핑은 chassis CAN DBC 정의에 따름. 일반적인 IONIQ5 매핑:
- 0 또는 미정: unknown
- P=1 → `j2735TransmissionState_park`
- R=2 → `j2735TransmissionState_reverseGears`
- N=3 → `j2735TransmissionState_neutral`
- D=4 → `j2735TransmissionState_forwardGears`
- 그 외 → `unavailable`

→ DBC 의 GearInfo (ID 117) signal table 을 BSM TX 노드 구현 시 재확인 필요. 본 문서는 mapping 표만 제공.

### `/sensors/ioniq5_ad_can` — `katech_custom_msgs/ioniq5_ad_can_msg.msg`

```
line 16: uint8 brain_status        # BrainState
line 19: uint8 operation_mode      # AutonomousState
line 20: uint8 autonomous_mode
```

→ BSM core 에는 매핑할 필드 없음. 추후 partII safetyExtensions 의 lights/wipers/AccelSet 등에 사용 가능.

### `novatel_gps_msgs` 사용 여부

`localization/gps-world-tf` 에서 novatel 의존성이 있지만 `siheung_v2x` 측은 ublox 사용. **BSM TX 는 `ublox_msgs::NavPVT` 만 사용** 권장 (siheung_dev 브랜치 기준 ublox 가 primary).

---

## 4. TX 송신 경로

### 옵션 비교

| 옵션 | 장점 | 단점 |
|---|---|---|
| **A) UDP 직접 송신** | OBU 에 즉시 전달, 외부 의존 없음 | OBU IP/Port 환경 의존, 디버깅 어려움 |
| **B) ROS 토픽 publish** | 디버그 용이 (rostopic echo), TX 노드와 송신 노드 분리, 단위 테스트 가능 | 다른 노드가 UDP 송신해야 함 (또는 미송신) |
| **C) 둘 다** | 정상 동작 + 디버깅 모두 가능 | 코드 복잡도 증가 |

### 현재 패키지 UDP 인프라

- `src/v2x/siheung_v2x/src/j2735_decode.cpp` 는 **RX 만** 함 (UDP 9999 bind, `src/j2735_decode.cpp:370-388`). TX 패턴 없음.
- OBU 헤더 5바이트 규약 (`src/j2735_decode.cpp:29-35`):
  ```
  [0] Frame Type   : 0=OBU→PC, 1=PC→OBU
  [1] Seq No       : 0~255
  [2] msg source   : 0=from RSU, 1=from Uu
  [3] isMsgFrame   : 0=MessageFrame, 1=not
  [4] Reserved
  ```
  → TX 시 `Frame Type = 1`, `isMsgFrame = 0`, `Seq No = bsm.msgCnt`, source=0.
- launch 디렉토리 없음 (`siheung_v2x` 내부). 기존 노드는 `launch/katech_test.launch:48` 와 `launch/siheung.launch:7` 에서 띄움.
- katri_obu_interface 의 UDP send 패턴 (`src/v2x/katri_obu_interface/lib/katri_v2x.cpp:20-32`) 도 동일한 UDP socket + sockaddr_in 구조. OBU IP/Port 는 `katech_test.launch` 등에서 파라미터로 주입.

### 권장: **C 안 (UDP send + ROS 토픽 publish)**

이유:
- 사용자 요청은 "단계별 확장" — 1단계는 ROS 토픽 만으로 시작하고 추후 UDP 추가가 자연스러움.
- 하지만 별도 클래스로 한 번에 구성하는 게 재작업이 적음.

**구현 우선순위**:
1. **1단계 (B 안)**: 토픽 `/siheung_v2x/bsm_tx` (`std_msgs/UInt8MultiArray`) publish. `roslaunch` 파라미터 `~enable_udp=false` 기본값.
2. **2단계 (A 추가)**: `~enable_udp=true` 일 때 `~obu_ip`, `~obu_port`, `~obu_seq` 파라미터로 UDP `sendto` 추가. 5바이트 OBU 헤더 prefix.

### 토픽 메시지 타입 선택

`src/msgs/v2x_msgs/msg/` 안에 byte payload 담을 메시지 없음 (`intersection_array_msg`, `intersection_msg`, `movement_msg` 만 존재). → **`std_msgs/UInt8MultiArray`** 사용 권장 (또는 `std_msgs/ByteMultiArray` — 의미 동일, 일부 툴에서 ByteMultiArray 가 사용 안 됨).

토픽 이름: `/siheung_v2x/bsm_tx` (`/siheung_spat` 와 같은 prefix 규약).

---

## 5. 노드 골격 (코드 스켈레톤)

파일: `src/v2x/siheung_v2x/src/bsm_tx_node.cpp` (신규)

### 클래스 구조 (의사 코드)

```cpp
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <ublox_msgs/NavPVT.h>
#include <katech_custom_msgs/v_can_msg.h>
#include <cstring>
#include <mutex>
#include <atomic>

// UDP 옵션
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "ffasn1-j2735-2026-KSR1600.h"
#include "asn1defs.h"

#define J2735_MSG_ID_BSM 20

class BsmTxNode {
 public:
  BsmTxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~BsmTxNode();

 private:
  // ROS
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_navpvt_;
  ros::Subscriber sub_vcan_;
  ros::Publisher  pub_bsm_;
  ros::Timer      timer_;

  // 최신 상태 캐시 (각 콜백에서 갱신)
  std::mutex                 mtx_;
  ublox_msgs::NavPVT         latest_navpvt_;
  bool                       has_navpvt_ = false;
  katech_custom_msgs::v_can_msg latest_vcan_;
  bool                       has_vcan_ = false;

  // 카운터
  uint8_t msg_cnt_ = 0;

  // 파라미터
  uint8_t  vehicle_id_[4] = {0x00, 0x00, 0x00, 0x01};
  int      vehicle_width_cm_ = 190;
  int      vehicle_length_cm_ = 464;
  bool     enable_udp_ = false;
  std::string obu_ip_ = "192.168.0.10";
  int      obu_port_ = 9999;

  // UDP
  int      sock_fd_ = -1;
  sockaddr_in obu_addr_{};
  uint8_t  obu_seq_ = 0;

  // 콜백
  void onNavPvt(const ublox_msgs::NavPVT::ConstPtr& msg);
  void onVCan(const katech_custom_msgs::v_can_msg::ConstPtr& msg);
  void onTimer(const ros::TimerEvent&);

  // 핵심 로직
  bool fillBsm(j2735BasicSafetyMessage& bsm);
  bool encodeAndSend();

  // 유틸
  static int  gearToTransmission(uint8_t gear_status);
  static int  clampi(int v, int lo, int hi);
};

// 생성자: 파라미터 로드, subscribe/advertise, 10Hz timer 시작
BsmTxNode::BsmTxNode(...) {
  pnh_.param<std::string>("navpvt_topic", topic, "/ublox/navpvt");
  pnh_.param("vehicle_width_cm",  vehicle_width_cm_, 190);
  pnh_.param("vehicle_length_cm", vehicle_length_cm_, 464);
  pnh_.param("enable_udp",        enable_udp_, false);
  pnh_.param<std::string>("obu_ip", obu_ip_, "192.168.0.10");
  pnh_.param("obu_port",          obu_port_, 9999);
  // ~vehicle_id 는 "AABBCCDD" hex string → 4 bytes 파싱

  sub_navpvt_ = nh_.subscribe(topic, 10, &BsmTxNode::onNavPvt, this);
  sub_vcan_   = nh_.subscribe("/sensors/v_can", 10, &BsmTxNode::onVCan, this);
  pub_bsm_    = nh_.advertise<std_msgs::UInt8MultiArray>("/siheung_v2x/bsm_tx", 10);

  if (enable_udp_) {
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    obu_addr_.sin_family = AF_INET;
    obu_addr_.sin_port = htons(obu_port_);
    inet_pton(AF_INET, obu_ip_.c_str(), &obu_addr_.sin_addr);
  }

  timer_ = nh_.createTimer(ros::Duration(0.1), &BsmTxNode::onTimer, this); // 10Hz
}

// 콜백: 캐시만 갱신
void BsmTxNode::onNavPvt(const ublox_msgs::NavPVT::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  latest_navpvt_ = *msg; has_navpvt_ = true;
}
void BsmTxNode::onVCan(const katech_custom_msgs::v_can_msg::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  latest_vcan_ = *msg; has_vcan_ = true;
}

// 10Hz 타이머: BSM 채움 → MessageFrame 으로 감쌈 → UPER 인코딩 → publish (+UDP)
void BsmTxNode::onTimer(const ros::TimerEvent&) {
  if (!has_navpvt_) {
    ROS_WARN_THROTTLE(2.0, "[bsm_tx] waiting for NavPVT");
    return;
  }

  j2735BasicSafetyMessage bsm{};
  if (!fillBsm(bsm)) return;

  j2735MessageFrame mf{};
  mf.messageId = J2735_MSG_ID_BSM;
  mf.value.type = (ASN1CType*)asn1_type_j2735BasicSafetyMessage;
  mf.value.u.data = &bsm;

  uint8_t* buf = nullptr;
  ASN1Error err{};
  asn1_ssize_t len = asn1_uper_encode2(&buf, asn1_type_j2735MessageFrame, &mf, &err);
  if (len <= 0 || buf == nullptr) {
    ROS_ERROR_THROTTLE(2.0, "[bsm_tx] UPER encode failed");
    return;
  }

  // (1) ROS topic publish
  std_msgs::UInt8MultiArray out;
  out.data.assign(buf, buf + len);
  pub_bsm_.publish(out);

  // (2) UDP (선택)
  if (enable_udp_ && sock_fd_ >= 0) {
    std::vector<uint8_t> pkt(5 + (size_t)len);
    pkt[0] = 1;             // Frame Type: PC → OBU
    pkt[1] = obu_seq_++;    // Seq No
    pkt[2] = 0;             // source: from RSU (0)
    pkt[3] = 0;             // isMsgFrame
    pkt[4] = 0;             // reserved
    std::memcpy(pkt.data() + 5, buf, len);
    sendto(sock_fd_, pkt.data(), pkt.size(), 0,
           (sockaddr*)&obu_addr_, sizeof(obu_addr_));
  }

  asn1_free(buf);
  // bsm 의 내부 OCTET STRING (id, wheelBrakes 의 ASN1BitString.buf) 은
  // 모두 stack/local-array 로 채웠으므로 asn1_free_value 호출하지 않는다.
  // (asn1_free_value 는 라이브러리가 alloc 한 객체에만 사용.)
}

// BSM 채움 (필드 매핑은 3절 표대로)
bool BsmTxNode::fillBsm(j2735BasicSafetyMessage& bsm) {
  std::lock_guard<std::mutex> lk(mtx_);
  auto& core = bsm.coreData;

  // msgCnt
  core.msgCnt = msg_cnt_ & 0x7F;
  msg_cnt_++;

  // id (4 bytes)
  static uint8_t id_storage[4];
  std::memcpy(id_storage, vehicle_id_, 4);
  core.id.buf = id_storage;
  core.id.len = 4;

  // secMark
  uint64_t ms_now = ros::Time::now().toNSec() / 1000000ULL;
  core.secMark = (int)(ms_now % 60000);

  // lat/lon/elev
  core.lat  = latest_navpvt_.lat;                  // 동일 단위 (1/10 microdeg)
  core.Long = latest_navpvt_.lon;                  // 동일 단위
  core.elev = clampi(latest_navpvt_.hMSL / 100, -4096, 61439);

  // accuracy
  {
    int a = (int)(latest_navpvt_.hAcc / 50);       // mm → 0.05m
    core.accuracy.semiMajor = (a > 254) ? 255 : a;
    core.accuracy.semiMinor = core.accuracy.semiMajor;
    core.accuracy.orientation = 65535;             // unavailable
  }

  // transmission
  core.transmission = has_vcan_
    ? (j2735TransmissionState)gearToTransmission(latest_vcan_.gear_status)
    : j2735TransmissionState_unavailable;

  // speed (mm/s → 0.02 m/s)
  {
    int s = latest_navpvt_.gSpeed / 20;
    core.speed = s < 0 ? 0 : (s > 8190 ? 8190 : s);
  }

  // heading (deg*1e-5 → 0.0125 deg)
  {
    int32_t h = latest_navpvt_.heading;            // -360e5 .. +360e5
    if (h < 0) h += 36000000;
    double hd = (double)h * 1e-5;                  // 0..360 deg
    int hraw = (int)(hd / 0.0125 + 0.5);
    if (hraw < 0) hraw = 0;
    if (hraw > 28799) hraw = 28799;
    core.heading = hraw;
  }

  // angle (steering, deg → 1.5 deg, -126..127)
  if (has_vcan_) {
    int a = (int)std::round(latest_vcan_.steering_angle / 1.5);
    core.angle = clampi(a, -126, 126);
  } else {
    core.angle = 127;                              // unavailable
  }

  // accelSet
  if (has_vcan_) {
    core.accelSet.Long = clampi((int)std::round(latest_vcan_.long_acceleration * 100), -2000, 2000);
    core.accelSet.lat  = clampi((int)std::round(latest_vcan_.lat_acceleration  * 100), -2000, 2000);
    core.accelSet.vert = -127;                     // unavailable
    core.accelSet.yaw  = clampi((int)std::round(latest_vcan_.yaw_rate * 100), -32767, 32767);
  } else {
    core.accelSet.Long = 2001; core.accelSet.lat = 2001;
    core.accelSet.vert = -127; core.accelSet.yaw = 0;
  }

  // brakes
  {
    static uint8_t brake_bits[1];
    bool applied = has_vcan_ && latest_vcan_.brake_pedal_pos > 0.05;
    brake_bits[0] = applied ? 0x78 : 0x80;        // 0b01111000 (4륜) or unavailable bit
    core.brakes.wheelBrakes.buf = brake_bits;
    core.brakes.wheelBrakes.len = 5;              // bits
    core.brakes.traction  = j2735TractionControlStatus_unavailable;
    core.brakes.abs       = j2735AntiLockBrakeStatus_unavailable;
    core.brakes.scs       = j2735StabilityControlStatus_unavailable;
    core.brakes.brakeBoost = j2735BrakeBoostApplied_unavailable;
    core.brakes.auxBrakes = j2735AuxiliaryBrakeStatus_unavailable;
  }

  // size
  core.size.width  = vehicle_width_cm_;
  core.size.length = vehicle_length_cm_;

  // partII / regional
  bsm.partII_option = FALSE;
  bsm.partII.tab = nullptr; bsm.partII.count = 0;
  bsm.regional_option = FALSE;
  bsm.regional.tab = nullptr; bsm.regional.count = 0;

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

int main(int argc, char** argv) {
  ros::init(argc, argv, "bsm_tx_node");
  ros::NodeHandle nh, pnh("~");
  BsmTxNode node(nh, pnh);
  ros::spin();
  return 0;
}
```

### 메모리 관리 포인트

- `asn1_uper_encode2` 가 alloc 한 `buf` 는 `asn1_free(buf)` (libffasn1-base export 함수) 로 해제. 또는 `free(buf)` 가능 (라이브러리가 `malloc` 사용 시 — 헤더 확인 필요).
- BSM 내부의 `id.buf`, `brakes.wheelBrakes.buf` 는 **로컬 static array 포인터** 이므로 free 하지 않는다.
- **`asn1_free_value(asn1_type_j2735BasicSafetyMessage, &bsm)` 호출하지 말 것** — 라이브러리가 alloc 한 게 아니라 우리가 채운 stack 객체임.
- 매 timer 마다 새 stack 인스턴스 생성 → race 없음.

### 주기

- ROS Timer `ros::Duration(0.1)` = 10Hz. BSM 표준 (J2735) 권장 주기 100ms.

---

## 6. CMakeLists 변경안

`src/v2x/siheung_v2x/CMakeLists.txt` 에 추가 (기존 항목 수정 X, append).

```cmake
## ── BSM TX ────────────────────────────────────────────────
## bsm_tx_node: BSM 송신. KSR1600 헤더 + 2020.so 링크
## (j2735_decode.cpp 와 같은 헤더지만 별도 executable 이라 충돌 없음)
add_executable(bsm_tx_node src/bsm_tx_node.cpp)
target_link_libraries(bsm_tx_node
  ${catkin_LIBRARIES}
  ffasn1-base
  ffasn1-j2735-2020
)
```

### CMakeLists 의존성 추가 (필요 시)

상단 `find_package(catkin REQUIRED COMPONENTS ...)` 에 다음이 없으면 추가:

- `ublox_msgs` — NavPVT 사용
- `katech_custom_msgs` — v_can_msg 사용
- `std_msgs` — 이미 있음

`src/v2x/siheung_v2x/package.xml` 의 `<build_depend>` / `<exec_depend>` 에도 동일하게 `ublox_msgs`, `katech_custom_msgs` 추가.

### KSR1600.c 링크 충돌 여부

`src/ffasn1-j2735-2026-KSR1600.c` 는 **`siheung_v2x_node` 만** 컴파일 단위에 포함. `bsm_tx_node` 는 `.c` 를 포함 안 함. 두 executable 이 같은 `.so` (`ffasn1-j2735-2020`) 를 링크해도 별개 프로세스라서 런타임 충돌 없음. 빌드 시간 약간 단축.

만약 KSR1600.so 가 BSM 심볼 export 만 있고 2020.so 가 없는 경우 → 링크를 `ffasn1-j2735-2026-KSR1600` 으로 교체 필요. **빌드 후 nm/objdump 로 확인**:

```bash
nm -D /home/sim/mcar/src/v2x/siheung_v2x/lib/x86_64/libffasn1-j2735-2020.so | grep BasicSafetyMessage
nm -D /home/sim/mcar/src/v2x/siheung_v2x/lib/x86_64/libffasn1-j2735-2026-KSR1600.so | grep BasicSafetyMessage
```

(사용자가 이미 "두 라이브러리 모두 BSM 타입 정의 export 함" 이라 확인 완료.)

### launch 추가 (선택)

`launch/siheung.launch` 에 한 줄 추가 가능 (지금은 사양만, 추후 적용):

```xml
<node name="bsm_tx_node" type="bsm_tx_node" pkg="siheung_v2x" output="screen">
  <param name="navpvt_topic"     value="/ublox/navpvt"/>
  <param name="vehicle_id"       value="AABBCCDD"/>
  <param name="vehicle_width_cm" value="190"/>
  <param name="vehicle_length_cm" value="464"/>
  <param name="enable_udp"       value="false"/>
  <!-- 2단계용 -->
  <!--<param name="obu_ip"       value="192.168.0.10"/>-->
  <!--<param name="obu_port"     value="9999"/>-->
</node>
```

---

## 7. 위험·미지원 항목 체크리스트

| 항목 | 위험 / 미지원 | 조치 |
|---|---|---|
| **J2735 버전 차이** | 2016 / 2020 / 2026-KSR1600 간 partII / regional 정의 다름. KSR1600 헤더 사용 → 한국 RSU 와 호환. 글로벌 RSU 와 호환은 불보장. | 1단계는 partII/regional 모두 omit. 한국 시화 시범운행 전용. |
| **partII (VehicleSafetyExtensions)** | events, pathHistory, pathPrediction, lights 등 — RSU 의 일부 어플리케이션 (예: emergency vehicle preemption) 에서 사용. | 1단계 nullptr / count=0. 추후 단계별 추가. 헤더 `j2735BasicSafetyMessage_1` 의 `j2735PartIIcontent_1` 사용. |
| **regional (KOR/SAE)** | 한국형 확장 (`j2735BasicSafetyMessage_KOR` 등 line 5332+). | 1단계 nullptr. 추후 한국형 ADAS extension (line 5385) 활용 가능. |
| **vehicle_id 4-byte 인코딩** | TemporaryID 는 OCTET STRING SIZE(4). 표준상 매 5분 마다 재발생 권장. | 1단계: 파라미터 정적 hex. 추후 startup 시 random 4-byte + 5min 갱신 timer. |
| **secMark 처리** | DSecond 는 0..65535 ms (60000 = 분 경계, 65535 = unavailable). 60초 + 일부 ms 일 때 65499 까지만 유효. | `% 60000` 으로 wrap. ROS Time 정확도 ≥ 1ms 보장 가정. |
| **단위 환산: heading** | NavPVT heading 단위 `deg / 1e-5` (예: 12345678 = 123.45678 deg). BSM Heading 0.0125 deg step. | `(navpvt.heading * 1e-5) / 0.0125`. clamp 28799. 음수 → +360 처리. |
| **단위 환산: speed** | NavPVT gSpeed 단위 mm/s. BSM Speed 0.02 m/s = 20 mm/s. | `gSpeed / 20`. clamp 0..8190 (8191 = unavailable 충돌 회피). |
| **단위 환산: lat/lon** | NavPVT 와 BSM 모두 1/10 microdeg = deg/1e-7. **변환 없음.** | 직접 대입. 단 범위 검증 (-900M..900M / -1800M..1800M). |
| **단위 환산: yaw_rate** | v_can yaw_rate 의 단위가 DBC 따라 deg/s 또는 rad/s. | DBC 확인 필수. 본 사양은 deg/s 가정. rad/s 면 `* 180/π` 추가. |
| **steering_angle 부호** | IONIQ5 SAS_Angle 부호 (좌+ / 우-) 는 DBC 정의. BSM 표준 부호 (좌+ / 우-) 와 일치 가정. | DBC 와 BSM 표준 부호 확인. 어긋나면 `-` 추가. |
| **wheelBrakes BitString len=5** | `len` 은 비트 수, `buf` 는 ceil(5/8)=1 byte. bit ordering (MSB first or LSB first) 라이브러리 의존. | `0x80` = bit0 (unavailable). `0x78` = bits 1-4 (FL,RL,FR,RR). 라이브러리 컨벤션은 보통 MSB first (bit0 = MSB). 디코더 측에서 확인 필요. |
| **NavPVT 부재 (Timer 만 도는 경우)** | GPS 없으면 BSM 발행 의미 없음. | `!has_navpvt_` 면 skip + ROS_WARN_THROTTLE. |
| **v_can 부재** | NavPVT 만 있으면 BSM 최소 정보로 발행 가능. | speed/heading 은 NavPVT 로 OK. angle/accel/brakes 는 unavailable 마커로 채움. |
| **asn1_free 함수 이름** | `asn1defs.h` 에 명시적 `asn1_free()` declaration 미확인. `free()` 또는 `asn1_free()` 중 어느 것을 써야 하는지 .so 의 export 심볼 확인. | 빌드 후 `nm -D libffasn1-base.so \| grep -i free` 로 확인. 보통 `asn1_free()` 노출됨. |
| **NavPVT 토픽명** | `/ublox/navpvt` 가 표준이지만 launch 에서 remap 될 수 있음. | 파라미터 `~navpvt_topic` 로 노출, 기본 `/ublox/navpvt`. |
| **gear_status 매핑** | IONIQ5 DBC 의 GearInfo (ID 117) signal 의 raw 값 매핑. | DBC 와 chassis_CAN_reader 코드에서 매핑 재확인. 본 사양은 1=P/2=R/3=N/4=D 가정. |
| **2020 헤더 vs KSR1600 헤더 충돌** | 같은 typedef 가 두 헤더에 있으므로 동시 include 불가. | bsm_tx_node.cpp 는 KSR1600 헤더만 include. 다른 .cpp 와 link-only 공유. |
| **인코딩 실패 시 retry** | UPER encode 가 -1 반환 시 어떤 필드가 range 위반인지 ASN1Error 로 알 수 있음. | `asn1_uper_encode2` 의 `err` 사용해 ROS_ERROR 로 출력 후 다음 주기. |
| **RSU/OBU SecMark 동기화** | RSU 측이 GPS 시간 기반이면 secMark 가 NavPVT 의 GPS time 으로부터 와야 함. ROS time vs GPS time 차이. | NavPVT.iTOW / nano 로부터 GPS minute boundary 계산 가능. 1단계 ros::Time::now() 로 시작, 정확도 요구 시 NavPVT 기반으로 교체. |
| **BSM ID 충돌** | 표준 J2735 messageId 20 가 KSR1600 에서도 동일한지 검증. | KSR1600 표준 (TTAK.KO-06.0307/R2) 가 SAE J2735 와 동일 messageId 사용. 안전. |
| **partII 옵션 비트 (BOOL)** | `partII_option = FALSE` 일 때 partII struct 무시되는지 라이브러리 동작. | 헤더상 OPTIONAL 마커이므로 `FALSE` 면 인코더가 skip. 안전. |

---

## 부록: 단위 환산 빠른 참조

| 변환 | 식 |
|---|---|
| NavPVT lat(deg*1e-7) → BSM Latitude(0.1µdeg) | identity (그대로) |
| NavPVT gSpeed(mm/s) → BSM Speed(0.02 m/s) | `/ 20` |
| NavPVT heading(deg*1e-5) → BSM Heading(0.0125 deg) | `* 1e-5 / 0.0125` 또는 `* 8e-6` ≈ `/ 125`  |
| NavPVT hMSL(mm) → BSM Elevation(0.1 m) | `/ 100` |
| NavPVT hAcc(mm) → semiMajor(0.05 m) | `/ 50` |
| v_can steering(deg) → BSM angle(1.5 deg) | `/ 1.5` (round) |
| v_can accel(m/s²) → BSM accel(0.01 m/s²) | `* 100` |
| v_can yaw(deg/s) → BSM yawRate(0.01 deg/s) | `* 100` |
| veh dim(m) → BSM size(cm) | `* 100` |

---

## 정리: 작업 단계별 To-do (구현 단계에서 사용)

1. `bsm_tx_node.cpp` 신규 작성 (5절 스켈레톤 기반).
2. `CMakeLists.txt` 에 `add_executable(bsm_tx_node ...)` 추가, `target_link_libraries` 에 `ffasn1-base ffasn1-j2735-2020`.
3. `package.xml` 에 `ublox_msgs`, `katech_custom_msgs` 의존성 추가 (없을 시).
4. 빌드 검증: `catkin_make --pkg siheung_v2x`. 링크 에러 시 KSR1600.so 로 교체.
5. 단위 테스트: 임의 NavPVT/v_can 발행 → `/siheung_v2x/bsm_tx` 토픽으로 buffer 발행 확인.
6. RSU 또는 같은 BSM 디코더 노드로 round-trip 디코드 검증 (`asn1_uper_decode` 로 다시 풀어서 필드 일치 확인).
7. (2단계) `enable_udp=true` 로 실제 OBU 송신 + Wireshark 캡처.
8. (3단계) partII safetyExtensions, regional KOR 확장 추가.

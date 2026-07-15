# BSM 에 "곧 지날 링크 ID 시퀀스" 싣기 — 설계

대상 노드: `src/v2x/siheung_v2x/src/mqtt_bsm_tx_node.cpp` (BSM 인코딩은 `bsm_tx_node.cpp` 와 동일 `fillBsm`)
J2735 codec: `ffasn1-j2735-2026-KSR1600` (UPER), MessageFrame(messageId=20) 래핑.

목표: 차량이 **곧 지날 링크 ID 순서열**을 BSM 안에 실어 V2N 브로커로 전송.
방식 2가지 — (A) `regional` extension, (B) `partII` 추가. 둘 다 **동일한 inner payload 포맷**을 공유한다.

---

## 0. 공통: inner payload 포맷 (regExtValue / partII_Value 에 들어갈 바이트)

OpenType 의 `octet_string` 에 들어가는 raw 바이트. 두 가지 인코딩 옵션:

### P1) JSON (권장 — 기존 인프라 선례와 동일)
OBU 가 TIM regional 에서 이미 쓰는 방식. 디버깅 쉽고 수신측 파서 재사용 가능.
```json
{"v":1,"cur":5057,"next":[5058,5059,5060,5064,5065],"ts":231358}
```
- `v`: 포맷 버전 (1)
- `cur`: 현재 LINK_ID
- `next`: 곧 지날 링크 ID 배열 (앞에서부터 가까운 순, 최대 N개)
- `ts`: timeStamp(MinuteOfYear ms) — 선택

크기: 링크 10개 ≈ 90~110 byte.

### P2) 컴팩트 바이너리 TLV (크기 최적화용 대안)
```
[0]    ver      : u8   = 0x01
[1]    count N  : u8   (next 링크 개수, 0..32)
[2..5] cur_link : u32 BE (현재 LINK_ID)
[6..]  next[i]  : u32 BE × N
```
크기: 6 + 4N. 링크 10개 = 46 byte. 단, 송수신 양측이 포맷 합의 필수.

> 권장: **P1(JSON)**. 10Hz 브로드캐스트지만 closed V2N 망 + 링크 수 cap 으로 부하 무시 가능,
> 그리고 수신측이 TIM rx 노드의 `jsonCandidatesFromOpenType()` 로직을 그대로 재사용할 수 있음.

---

## 방식 A — Regional Extension

### A.1 구조
`BasicSafetyMessage.regional` = `SEQUENCE OF RegionalExtension { regionId: INTEGER(0..255), regExtValue: OpenType }`
```
bsm.regional_option = TRUE;
bsm.regional.count  = 1;
bsm.regional.tab    = &reg;          // j2735RegionalExtension_1

reg.regionId          = ROUTE_REGION_ID;     // ← V2N 규격에 예약 (아래 주의)
reg.regExtValue.type  = NULL;                // raw octet_string 모드
reg.regExtValue.u.octet_string.buf = payload_buf;   // P1 또는 P2 바이트
reg.regExtValue.u.octet_string.len = payload_len;
```

### A.2 regionId 선정
`RegionId` 는 INTEGER(0..255), 표준 add-group(addGrpA/B/C 등)과 충돌 주의.
- OBU 선례: TIM 에서 regionId **200/201/202** 사용 → 같은 사설 대역(20x) 관례 채택 권장.
- 제안: `ROUTE_REGION_ID = 210` (또는 V2N 인터페이스 문서에 정식 예약).
- **반드시 시흥 V2N 인터페이스 규격 문서에 "regionId=210 = 차량 주행예정 링크열" 로 등재**해야 수신측이 해석.

### A.3 fillBsm 변경 (의사코드)
```cpp
// 멤버: std::vector<uint8_t> route_payload_; j2735RegionalExtension_1 reg_storage_;
//        (encode 시점까지 살아있어야 함 — 멤버 or static)
bool MqttBsmTxNode::fillBsm(...) {
  ... // 기존 coreData
  if (buildRoutePayload(route_payload_)) {        // 최신 링크열 → JSON/TLV
    reg_storage_.regionId = ROUTE_REGION_ID;
    reg_storage_.regExtValue.type = nullptr;
    reg_storage_.regExtValue.u.octet_string.buf = route_payload_.data();
    reg_storage_.regExtValue.u.octet_string.len = route_payload_.size();
    bsm.regional_option = TRUE;
    bsm.regional.tab    = &reg_storage_;
    bsm.regional.count  = 1;
  } else {
    bsm.regional_option = FALSE;                  // 경로 없으면 기존대로 생략
  }
  ...
}
```
> 주의: `route_payload_` / `reg_storage_` 는 `asn1_uper_encode2()` 호출이 끝날 때까지 유효해야 함.
> 현재 코드의 `id_storage`/`brake_bits` 처럼 onTimer 스코프 또는 멤버로 보관. `asn1_free_value` 는 호출하지 않음(외부 버퍼라서).

### A.4 장단점
- ➕ nesting 1단계로 가장 단순. OBU 선례와 동일 패턴 → 검증된 디코딩 경로.
- ➕ 수신측이 TIM rx 의 regional 파서 거의 그대로 재사용.
- ➖ regionId 네임스페이스가 표준과 공유 → 사설 id 예약 관리 필요.

---

## 방식 B — PartII 추가

### B.1 구조
`BasicSafetyMessage.partII` = `SEQUENCE OF PartIIcontent { partII_Id: INTEGER, partII_Value: OpenType }`
표준 partII_Id: `vehicleSafetyExt(0)`, `specialVehicleExt(1)`, `supplementalVehicleExt(2)`.
주행예정 링크열은 표준 컨텐츠가 아니므로 **사설 partII_Id** 사용.

```cpp
part_storage_.partII_Id          = ROUTE_PARTII_ID;   // 예: 64 (사설 대역, 0/1/2 회피)
part_storage_.partII_Value.type  = NULL;
part_storage_.partII_Value.u.octet_string.buf = route_payload_.data();
part_storage_.partII_Value.u.octet_string.len = route_payload_.size();

bsm.partII_option = TRUE;
bsm.partII.tab    = &part_storage_;   // j2735PartIIcontent_1
bsm.partII.count  = 1;
```

### B.2 partII_Id 선정
- 0/1/2 는 표준 점유 → 사설값 사용. 제안 `ROUTE_PARTII_ID = 64`.
- regional 과 마찬가지로 V2N 규격 문서에 정식 등재 필요.

### B.3 fillBsm 변경 (의사코드)
방식 A 와 동일하게 `route_payload_` 생성 후 partII 리스트에 1개 원소로 연결:
```cpp
if (buildRoutePayload(route_payload_)) {
  part_storage_.partII_Id = ROUTE_PARTII_ID;
  part_storage_.partII_Value.type = nullptr;
  part_storage_.partII_Value.u.octet_string.buf = route_payload_.data();
  part_storage_.partII_Value.u.octet_string.len = route_payload_.size();
  bsm.partII_option = TRUE;
  bsm.partII.tab    = &part_storage_;
  bsm.partII.count  = 1;
} else {
  bsm.partII_option = FALSE;
}
```

### B.4 장단점
- ➕ "차량 발신 확장 데이터" 라는 의미상 자연스러움. partII 는 list 라 다른 확장과 공존(여러 블록) 쉬움.
- ➕ 향후 표준 supplementalVehicleExt 로 마이그레이션 시 자리 잡기 좋음.
- ➖ partII_Id 도 사설 예약 필요. regional 보다 nesting/관리 약간 복잡.

---

## 권장안

| 기준 | 방식 A (regional) | 방식 B (partII) |
|---|---|---|
| 구현 단순성 | ◎ | ○ |
| 인프라 선례 | ◎ (OBU TIM regional = JSON) | △ |
| 의미 정합성 | ○ | ◎ (vehicle 확장 데이터) |
| 다중 확장 공존 | △ | ◎ |

→ **1차: 방식 A (regional + JSON payload)** — 기존 OBU/TIM rx 패턴과 동일해 리스크 최소.
→ 추후 다른 차량발신 확장과 함께 쓰거나 표준 정렬이 필요해지면 방식 B 로 이전.
두 방식 모두 inner payload(P1/P2)는 동일하게 두어 **수신 파서 1개로 공용**.

---

## 미해결 / 선결 과제 (중요)

1. **경로 시퀀스 원천 부재.** `/localization/to_control_team` 은 `LINK_ID`+`NEXT_LINK_ID` 2개만 발행.
   - 옵션 (a) 경로 플래너가 산출한 full link 시퀀스를 새 토픽으로 노출.
   - 옵션 (b) BSM 노드에서 `LINK_ID`/`NEXT_LINK_ID` 를 시간축 rolling buffer 로 누적해 근사.
   - → `buildRoutePayload()` 가 어느 소스를 쓸지 먼저 결정해야 함.
2. **비표준 사설 확장.** regionId/partII_Id 모두 사설 → **closed 시흥 V2N 망 + 합의된 수신측에서만** 해석됨. 일반 J2735 수신기는 무시. V2N 인터페이스 규격 문서에 ID·payload 포맷 등재 필수.
3. **크기/주기.** 10Hz 브로드캐스트 → next 링크 개수 cap (예: 10) 권장. 변동 없으면 매 프레임 동일 payload 재전송.
4. **수명 관리.** payload/extension 구조체는 `asn1_uper_encode2` 종료까지 유효해야 하며 `asn1_free_value` 대상 아님(외부 버퍼).

## 검증 계획 (구현 후)
- encode 후 `asn1_uper_decode(MessageFrame)` 라운드트립 → regional/partII 의 octet_string 복원 확인.
- inner JSON 파싱 결과가 `cur`/`next` 와 일치.
- 기존 SPaT/BSM 송신 경로(메인 launch) 영향 없음(스모크).

---

## 구현 완료 (2026-06-11) — B 방식 + P1(JSON), mat LINK_ID_string

실제 적용 내역:
- **파일**: `src/v2x/siheung_v2x/src/mqtt_bsm_tx_node.cpp` (B 방식 = partII)
- **partII_Id = 63** (⚠️ J2735 `PartII-Id ::= INTEGER(0..63)` 제약. 64는 범위 초과로 encode 실패 → 63 사용. 0/1/2 는 표준 vehicleSafetyExt/specialVehicleExt/supplementalVehicleExt 라 회피)
- **payload(P1 JSON, mat LINK_ID_string 사용)**:
  ```json
  {"v":1,"links":["A222BF785057","A222BF785082",...],"n":N}
  ```
  `links[0]` = 현재 링크, 이후 = NEXT_LINK_ID 체인을 따라간 곧 지날 링크. 모두 mat 의 `LINK_ID_string`.
- **현재 링크 소스**: `/localization/to_control_team` `.LINK_ID`(int) 구독 → 링크맵으로 string 변환 + 체인.
- **링크맵**: `scripts/build_senario_link_map.py` 가 senario `link_*.mat` → `config/senario_link_map.csv`(308 링크, `link_id,link_id_string,next_link_id`). 노드가 startup 에 로드.
- **파라미터**(launch/siheung.launch mqtt_bsm_tx_node 블록):
  `enable_route_partii`(true), `route_partii_id`(63), `route_lookahead`(10), `route_topic`, `link_map_path=$(find siheung_v2x)/config/senario_link_map.csv`
- **동작**: to_control_team 미수신/링크맵에 없는 LINK_ID 면 partII 생략(기존 coreData-only 동작 유지).

검증(오프라인 ffasn1 라운드트립, `/tmp/tim_encode/verify_route_partii.cpp`):
- `buildRouteJson(cur=417,N=10)` → 위 JSON (NEXT 체인 정상).
- partII(63)+JSON 붙여 MessageFrame UPER 인코딩 = **218 byte** → 디코딩 시 `partII[0].partII_Id=63`, octet_string == 원본 JSON. **ROUND-TRIP PASS.**
- catkin_make PASS.

미반영(기존 설계의 선결 과제 유지):
- route_lookahead 만큼 NEXT 체인이 끊기면(next=0) 거기서 종료 — 분기/차선변경 경로는 단일 NEXT 체인만 따름.
- 비표준 사설 확장이므로 수신측이 partII_Id=63 + JSON 포맷을 알아야 해석 가능. V2N 규격 문서 등재 필요.

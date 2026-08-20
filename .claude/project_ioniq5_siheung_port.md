---
name: ioniq5-siheung-dev-siheung-release
description: web_hmi + senario 맵 + to_control_team_demo.py 를 siheung_release 에 정렬한 포팅 (2026-08-07)
metadata: 
  node_type: memory
  type: project
  originSessionId: 2395a1f6-e9bf-4df6-a273-6d8d2c8d5727
  modified: 2026-08-07T07:26:24.734Z
---

`ioniq5_siheung_dev`(IONIQ 5 차량 + 시흥 시범운행 노선) 를 만들기 위해 `siheung_release`(아이오닉 EV) 의
web_hmi·맵·localizer 를 옮겨온 작업. 2026-08-07 완료.

## 맵 (siheung_release 와 동일 셋)
- mat: `mapfiles/senario` **293개**, `LINK_ID` 417~3877 **비연속**. `link_N.mat` 의 `LINK_ID == N`.
  K-City 처럼 인덱스==ID 가 아니다. 상세 [[map_data]]
- shp(web_hmi): `src/shp_map/senario_shp_20260623` (EPSG:32652 → 브리지에서 5179 재투영, MOLIT 11 표준 레이어)
- `katech_test.launch` 의 `mat_scenario` default = `senario`

## to_control_team_demo.py — K-City 하드코딩 전면 제거
로더는 siheung 방식(`sorted(glob('link_*.mat'))` → append → `valid_link_ids` set). `road_{i}` 동적 속성 없음.
- **`sorted()` 는 사전순** → `lane_id`/`lane_name` 은 링크 ID 와 무관한 인덱스. 링크 417 → `road_210`/`lane_id 211`.
  siheung 과 동일 동작이므로 그대로 둔다.
- ODD 판정은 **링크 공간**: `ODD_id_list = valid_link_ids`, 가드는 `p.LINK_ID == 0 or p.LINK_ID not in ODD_id_list`.
  `current_lane_id < 0` early return 이 있어야 이 가드가 성립한다(없으면 `target_roads[-1]` 이 잡힘).
- 제거로 잡힌 실동작 버그 2개:
  1. `else: p.Speed_Limit = 30` 이 **senario 전 링크의 맵 제한속도(50/60/40/30 혼재)를 30으로 덮어씀**
  2. 같은 `else` 가 ODD 이탈 가드에서 0으로 만든 Speed_Limit 을 30 으로 되살림 → 맵 밖에서 30 km/h 송출
- 채택: N-step look-ahead 정지선 전파(50m 체인), `p.MANUAVER`, `have_to_LangeChange_*=0`,
  `ODD_OCCUPIED_OFFSET_THRESHOLD` 2.0→**0.95**
- 유지(ioniq5 고유, K-City 무관): 20Hz `publish_timer_cb`+`last_p`, `host_altitude`,
  navpvt+TimeRangeSetting effective time. 이 때문에 early return 은 `publish()` 대신 `self.last_p = p`.
- **`SCHOOL_ZONE_LINK_ID = []`** — 기존 `[61,86,87,88]` 은 K-City 링크. 시흥 어린이보호구역 링크 ID 미확정이라
  취약시간대 20 km/h 감속은 현재 무동작(`effective_hour/minute`·`safety_vulnerable_time` 송출은 동작).
- `GPS_Over` 를 1 로 만들던 코드는 K-City 터널 링크 71/72 전용이었고 같이 제거됨 → 항상 0.
  msg/CAN(`On_ODD_Stat`)·stat_display 경로는 유지.

## to_control_team_from_local_msg.msg
- `NEXT_LINK_ID`/`LINK_ID` int8→**int32**, `lane_id` uint8→**int32**. senario ID(417~3877) 와 293 인덱스가
  int8/uint8 를 넘겨 `struct.error: byte format requires -128 <= number <= 127` 발생했었다.
- `int8 MANUAVER` 복원(siheung 과 동일). 단 **이 브랜치엔 소비자가 아직 없다** — SPaT 방향매칭
  (spat_CAN_writer/stat_display/hmi_state)은 siheung 에만 있음. [[spat_dir_match]]
- 최종 필드 집합 = siheung 필드 + ioniq5 전용 5개(`host_altitude`, `effective_hour/minute`,
  `safety_vulnerable_time`, `time_source`).

## DBC — CANdb_IONIQ5_AD_CAN_v8.dbc `BO_ 1824 LOCAL_MAP_INFO`
siheung `CANdb_IONIQev_PCAN1.dbc` 와 **비트 단위 동일**하게 재배치. 12bit 확장은 국소 변경이 아니라
8바이트 전체 repack(12개 중 10개 시그널 이동):
`NEXT_LINK_ID 8|8→8|12`, `LINK_ID 16|8→20|12`, `distance_to_lane_end 24→32`, `is_stop_line 41→48`,
`guard_zone 42→49`, `have_to_LangeChange_left/right 44/45→51/52`, `left/right_LaneChange_avail 46/47→53/54`,
`Speed_Limit 48→55`. (`look_at_signalGroupID 0|4`, `look_at_IntersectionID 4|4` 만 고정)
- **제어팀 수신부 동기화 필수 + CANoe 확인 필요.** 8bit 로는 417 → 161 로 절단됨.
- `local_CAN_writer.cpp` 는 `(double)msg.LINK_ID` 로 넘기고 폭은 DBC 가 정하므로 코드 변경 불필요.
- v3~v7 dbc 는 구 8bit 유지(LOCAL_MAP_INFO 쓰는 노드는 v8 로드하는 local_CAN_writer 뿐).

**Why:** 브랜치 계열이 차량 플랫폼이라([[project_build]]) 같은 시흥 노선을 IONIQ 5 로 돌리려면
siheung_release 의 맵/로직을 그대로 가져와야 한다. K-City 링크 하드코딩은 senario 링크 공간(417~)과
겹치지 않아 조용히 죽어 있는 게 아니라 `else` 분기로 맵 데이터를 파괴하고 있었다.

**How to apply:** 이 브랜치에서 맵/링크 관련 코드를 만질 땐 K-City 방식(인덱스==ID, 링크별 하드코딩)을
되살리지 말 것. 링크별 예외가 필요하면 mat 필드로 넣는다. 관련 [[web_hmi_adapt_harness]] [[map_data]]

## 포팅 검증 (2026-08-20)

**포팅이 만든 회귀 = 1건뿐.** `stat_display` SPaT staleness→TOR 무력화.
- 원인 패턴(재사용 가치 있는 함정): **이벤트 구동 토픽 → 자유구동(타이머) 토픽으로 갈아탈 때 "콜백 진입 = 수신"
  으로 판정하던 staleness 로직이 소리없이 죽는다.** `/katri_v2x_node/katri_spat`(SPaT 있을 때만 발행)
  → `/spat_merged`(`spat_merge_node.cpp:120` 이 빈 배열도 10 Hz 무조건 발행).
  `spat_stale` 영구 false → `if(tl_needed && spat_stale) v2x_status=2`(TOR) 절대 미성립.
- 수정: `stat_display.cpp:86` 을 `if (!msg->data.empty())` 로 가드. spat_merge_node 는 무수정(공유 V2X, siheung_dev diff ∅ 유지).
- **siheung_dev 에는 이 staleness 로직 자체가 없다**(ioniq5 브랜치 고유 안전기능). 회귀 판정의 근거.

**시흥 SPaT 실측 정의역** (출처 `src/visualization/spat_viewer/web/data/replay_timeline.json`,
실주행 3 bag / 2026-06-24 / 512.6 s / 2427 샘플):
- `IntersectionID` = {134,136,165,302,508,509,516,517,518,519} — 10곳, 최대 519
- `SignalGroupID` = {60, 70} / `TimeChangeDetails` = 10~2550
- `MovementStateName` = {LEFT 394, STR 170, STRAIGHT 1071} — MQTT 약어·OBU 풀네임 **둘 다 실재**.
  `spat_CAN_writer.cpp:33-37 movement_matches_manuaver` 가 `man==0` 에서 둘 다 허용하므로 방향매칭 무발신 없음.
- phase 분포 {3:1128, 5:332, 6:161, 7:14} → **permissive 5/7 이 21.2%** (3/6/8 만 인식하면 1/5 를 놓침)

**DBC v8 비트폭 부족 (제어팀 미합의, 시흥에서 즉시 발현)**: `V2X_SPaT_1.Intersection_ID_1` 4bit /
`signalGroup_1` 5bit, `LOCAL_MAP_INFO.look_at_IntersectionID`·`look_at_signalGroupID` 각 4bit.
IID 519·SG 70 수용 불가. 설계상의 "IID 축약표"는 **구현돼 있지 않다**(`spat_CAN_writer.cpp:139` 원값 전달).
`kvaDbStoreSignalValuePhys` 는 초과분을 잘라내고도 **status=0 반환**(무성). 절단 vs 클램프는 CANoe 로 확정 필요.

**pyqt `main_window.py` 는 활성 경로가 아니다** — `main_display.py` 는 `pyqt_hmi/launch/hmi.launch` 에만 있고
`katech_test.launch`·`diagnostic_only.launch` 어느 쪽도 include 하지 않는다(`roslaunch --nodes` 확인).
현 화면은 web_hmi_bridge → `hmi_state.py`. 그래서 main_window 의 방향필터·phase 5/7 누락은 **잠복 결함**.

**기존 결함(포팅 무관, 상속)**: `guard_zone` 생산자 부재 / `/sensors/rpm` 발행자 부재(MOTOR_RPM 미송신) /
`movementName_1` 항상 0 / `Pedestrian_Stat.rel_pos_x` 정의역 [-10.24,30.71] 이라 전방 40 m → -0.96 m 로 반전 /
`crosswalk_ped_fusion.py:93` 이 링크 661/662 에서 항상 CW#15 선택(CW#16 HMI 경보 누락, CAN 은 정상) /
CW#18 접근링크가 percept ROI 우측컷 밖. **percept ROI 5개 상수·3-zone 로직은 siheung_dev 와 동일** → 기준차량과 같은 한계.

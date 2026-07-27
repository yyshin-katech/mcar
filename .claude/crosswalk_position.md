---
name: crosswalk-position
description: 횡단보도 폴리곤 좌표를 katech_ped_detector.py(EPSG:5179) + senario mat 뷰어에 반영하는 하네스/패턴. 좌표계·뷰어 인프라·스코프 함정
metadata:
  node_type: memory
  type: project
  originSessionId: c2a74392-f1b6-49d8-91ad-e78f7a6568f9
  modified: 2026-07-27T08:49:11.512Z
---

`claude_work_list/crosswalk_position.md`(WGS84 lon/lat 폴리곤)를 `src/sensing/can/src/katech_ped_detector.py` 의 EPSG:5179 `crosswalk_data` 로 교체 + senario mat 뷰어 표시. 하네스 `crosswalk-position`(2026-07-15, siheung_dev). 관련 [[reference_tim_pedes_bag_replay]], [[stopline_adj]].

## 대상/구조
- **PART A** (⚠️ 2026-07-21 이후 **활성 검출 노드는 C++ `katech_ped_detector.cpp`** — [[ped_detector_cpp]]. `.py` 는 레거시이나 같은 값 유지 중이므로 좌표/링크 수정 시 **양쪽 동시 갱신**) `initialize_crosswalks()` 의 `crosswalk_data` 딕셔너리(int id → [(east,north),...] EPSG:5179 폴리곤). `Crosswalk.point_in_rectangle` 은 이미 **임의 다각형 ray-casting**(사각형 아님 OK, 닫힘 자동). `find_crosswalks_containing_object` 가 전체 crosswalk 순회 → **검출토픽 `/katech_msg/crosswalk_detection` 은 crosswalk 개수 자동 커버**.
- **PART B** `src/localization/gps_system_localizer/mapfiles/senario/mat_viewer_senario_260514c 1.html` — **이미 CROSSWALK 레이어 인프라 존재**: `var CROSSWALK`(≈1258행), 토글 버튼 `btn-crosswalk`(762), 범례(805), 렌더 IIFE(1256~). 좌표 추가 시 **배열 데이터만 교체**(버튼/범례/렌더 신규 불필요).

## 좌표계 (핵심 함정)
- 변환: `pyproj Transformer.from_crs("EPSG:4326","EPSG:5179", always_xy=True).transform(lon,lat)`. **오프라인 헬퍼로 산출 후 .py 에 리터럴로 붙여넣음**(런타임 pyproj 의존 추가 금지 — 기존 패턴). 소수 6자리.
- **뷰어 좌표 순서 상이**: `var CROSSWALK`/`TYPE5`/`LINEMARK` 는 Leaflet 네이티브 **`[lat,lon]`**(L.polygon), md 는 (lon,lat) → **swap 필요**, 7자리. 반면 `var DATA` 는 L.geoJSON `[lon,lat]`.

## CAN 경로 (보행자 시그널) — crosswalk_data 확장이 CAN 동작을 바꿈
`crosswalk_data`(detector) → `/katech_msg/crosswalk_detection`(ped_crosswalk_check_array_msg) → **`katech_ped_detector_can_writer.cpp`** → CAN `Pedestrian_Stat`(ID 528) + `Pedestrian_Stat_1`(529). `katech_test.launch` 기동.
- detector 가 보행자(status 1/2)를 크로스워크 폴리곤 안에서 찾으면 `on_crosswalk=1` → CAN 시그널 1. **[Phase2] `find_crosswalks_containing_object` 를 ego LINK_ID 로 게이팅** — ego 가 `CW_LINKS[N]`(§134) 링크에 있을 때만 크로스워크 N 검사 → on_crosswalk 은 **접근 중 크로스워크로 한정**(이전엔 링크 무관 기하판정, crosswalk_data 만 1→9 늘려도 자동 커버였음). 게이팅 한 곳으로 검출·CAN·occupancy 동시 적용.
- 단 CAN 은 **crosswalk_id 없음**: `ped_crosswalk_check_msg`=id/status/on_crosswalk/rel_pos_x/y. `on_crosswalk` 은 0/1 플래그, CAN `on_crosswalk_1/2/3` 는 **검출객체 슬롯(0·1·2·3) 인덱스**(횡단보도 번호 아님). writer 는 **객체 4개 cap**(data.size 1~4, >4 는 그 사이클 미전송).

## HMI 경보 (fusion → web_hmi) — Phase2 로 3~9 완성
- **[Phase2]** occupancy_msg 에 `uint8[] occupied_ids` 추가(기존 crosswalk1/2_occupied 보존). detector 가 링크-active 이고 보행자 있는 크로스워크 id 를 채움. 소비자=fusion 뿐이라 확장 안전.
- `crosswalk_ped_fusion.py`: active = `CW_LINKS` 조회(1~18, 하드코딩 1239/1238/1205 제거), own = `active in occupied_ids`, obu = #1→south_pedes/#2→east_pedes/**#3~#18→없음**.
- **OBU 보행자는 #1/#2 전용** (2026-07-27 재확인): `/obu/v2x_pedes_assistance` 의 **유일한 소비자 = fusion 노드**(나머지는 siheung_v2x 발행측 tim_publish/j2735_*). 콜백이 읽는 필드도 `south_pedes`/`east_pedes` 둘뿐 — `north_pedes`/`west_pedes` **미사용**(단일 RSU 4방향 한계). 따라서 **#3~#18 은 source=1(자체/라이다 주황)만** 발생하고 source=2(OBU 빨강)·3(둘 다 자홍)은 #1/#2 에서만.
- **`web_hmi/web/threejs/CrosswalkZones.jsx`**: EPSG:5179 **[E,N] 절대좌표**(crosswalk_data 와 동일, swap 없음 — mat 뷰어 [lat,lon] 과 다름), `CROSSWALK_IDS=Object.keys(CROSSWALK_POLYS)` 루프로 1~18 표시. 점멸 조건 `crosswalk_ped_active===id && present` → **fusion 이 #3 이상 active 발행 시 자동 점멸/배너**(web_hmi 코드 추가변경 없이 Phase2 로 완성). web_hmi_bridge 는 active_id(uint8) 그대로 전달.

## CW_LINKS (크로스워크→접근 LINK_ID)
1~9 = md §145/§155 원문 `{1:{1239,1238,1242,1241},2:{1205},3:{465,467,463},4:{417},5:{1029,1025,1027},6:{1092,1094,1090,1091,1093,1089},7:{1370,1368,1372,1369,1367,1371},8:{1326,1325,1330,1331},9:{1257,1258}}`.
**10~18 은 md 에 링크표가 없어 senario mat 에서 도출**(2026-07-27):
`{10:{870,871,877},11:{1163,1164,1165,1166,1167,1971},12:{1080,1082,1084,1086},13:{876,953},14:{574,579,580},15:{661,662},16:{661,662},17:{787},18:{2086,2206,2374}}`.
detector(cpp/py)·fusion **3곳 동일 dict**. **주의: 매핑 누락 접근링크 = 그 크로스워크 on_crosswalk 미발화**(완전성 사용자 책임).

### 도출 규칙 (mat `is_stop_line`/`NEXT_LINK_ID` 기반 — 재사용 가능)
- A: `is_stop_line==1` AND 폴리곤 미통과 AND 끝점거리 ≤13 m AND 전방(`ahead_cos ≥0.45`)
- B: A 링크에서 `NEXT_LINK_ID` 역추적하며 **누적 길이 20 m** 될 때까지 선행 링크 추가
- 이 규칙으로 md 정답 **1~9 중 8개 완전재현**(#3 만 선행 462/464/466 초과 — md 쪽이 비일관). `next=0` 종단 스텁(#4 의 419/420 형태)은 자동 배제됨. 스크립트 `_crosswalk_position_workspace/derive_links4.py`.
- **더 이상 완전 disjoint 아님**: #15/#16 은 같은 코너 스태거드 쌍이라 661/662 공유. 폴리곤은 비중첩이라 ray-cast 결과는 0/1개 유지, 단 fusion `active` 는 작은 id(#15)를 고름(검출/occupancy 는 둘 다 검사).

## 실행 이력 (2026-07-15)
1. Phase1a: crosswalk_data 1·2 → **1~9 확장** + mat 뷰어 var CROSSWALK. 점개수 (11,11,12,14,12,12,12,10,10). 1·2 byte-identical(0m), 3~9 max|Δ|=6.8e-07m. verifier 7/7. (aaefdc4)
2. Phase1b: `CrosswalkZones.jsx` 표시 1~9(표시 전용). (fd1baee)
3. **Phase2 (LINK 게이팅 + HMI 경보, "둘 다")**: detector `find_crosswalks_containing_object` LINK 게이팅 + occupancy `occupied_ids` / occupancy_msg `uint8[] occupied_ids` / fusion active 1~9·own=occupied_ids·obu #1·#2만. catkin_make EXIT0, verifier 7/7. 코드 3파일(detector.py/occupancy_msg.msg/fusion.py), CAN writer·web_hmi 무변경.

## Phase3 — 10~18 확장 (2026-07-27)
md 에 10~18 좌표만 추가되고 링크표 없음 → 위 도출 규칙으로 CW_LINKS 생성. **검출 노드는 이제 C++ [[ped_detector_cpp]] 가 활성**(`katech_test.launch:57`), `.py` 는 레거시 동기용으로 같은 값 유지.
- 수정 6파일: `katech_ped_detector.cpp`(crosswalk_data+cw_links_), `katech_ped_detector.py`(동일), `crosswalk_ped_fusion.py`(CW_LINKS), `CrosswalkZones.jsx`(CROSSWALK_POLYS 18), mat 뷰어 HTML 1258행, msg 2개(주석만 → **md5 불변**).
- 점 개수 10~18 = (12,12,4,12,12,12,12,12,11). **#12 는 4점**(다른 건 10~14점).
- 검증: 독립 재변환 최대오차 4.98e-07 m, 1~9 regression 0 m, HTML 변경 줄 = 1258 하나(+2646 B), `catkin_make --pkg can/katech_custom_msgs` EXIT0.
- 라이브 드라이런(roscore+C++노드+자극 publisher `_crosswalk_position_workspace/dryrun_pub.py`): 10~18 각각 접근링크에서 `occupied_ids=[N]`, 무관 링크에서 `[]`, #15/#16 은 662 에서 각자 검출.
- 하류 cap 없음 확인: web_hmi_bridge 는 active_id 를 int 그대로 전달, occupied_ids 는 uint8[], CAN writer 는 `on_crosswalk` 만 사용 → **CAN 무변경**.

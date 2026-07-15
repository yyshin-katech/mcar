---
name: crosswalk-position
description: 횡단보도 폴리곤 좌표를 katech_ped_detector.py(EPSG:5179) + senario mat 뷰어에 반영하는 하네스/패턴. 좌표계·뷰어 인프라·스코프 함정
metadata:
  node_type: memory
  type: project
  originSessionId: c2a74392-f1b6-49d8-91ad-e78f7a6568f9
---

`claude_work_list/crosswalk_position.md`(WGS84 lon/lat 폴리곤)를 `src/sensing/can/src/katech_ped_detector.py` 의 EPSG:5179 `crosswalk_data` 로 교체 + senario mat 뷰어 표시. 하네스 `crosswalk-position`(2026-07-15, siheung_dev). 관련 [[reference_tim_pedes_bag_replay]], [[stopline_adj]].

## 대상/구조
- **PART A** `katech_ped_detector.py` `initialize_crosswalks()` 의 `crosswalk_data` 딕셔너리(int id → [(east,north),...] EPSG:5179 폴리곤). `Crosswalk.point_in_rectangle` 은 이미 **임의 다각형 ray-casting**(사각형 아님 OK, 닫힘 자동). `find_crosswalks_containing_object` 가 전체 crosswalk 순회 → **검출토픽 `/katech_msg/crosswalk_detection` 은 crosswalk 개수 자동 커버**.
- **PART B** `src/localization/gps_system_localizer/mapfiles/senario/mat_viewer_senario_260514c 1.html` — **이미 CROSSWALK 레이어 인프라 존재**: `var CROSSWALK`(≈1258행), 토글 버튼 `btn-crosswalk`(762), 범례(805), 렌더 IIFE(1256~). 좌표 추가 시 **배열 데이터만 교체**(버튼/범례/렌더 신규 불필요).

## 좌표계 (핵심 함정)
- 변환: `pyproj Transformer.from_crs("EPSG:4326","EPSG:5179", always_xy=True).transform(lon,lat)`. **오프라인 헬퍼로 산출 후 .py 에 리터럴로 붙여넣음**(런타임 pyproj 의존 추가 금지 — 기존 패턴). 소수 6자리.
- **뷰어 좌표 순서 상이**: `var CROSSWALK`/`TYPE5`/`LINEMARK` 는 Leaflet 네이티브 **`[lat,lon]`**(L.polygon), md 는 (lon,lat) → **swap 필요**, 7자리. 반면 `var DATA` 는 L.geoJSON `[lon,lat]`.

## CAN 경로 (보행자 시그널) — crosswalk_data 확장이 CAN 동작을 바꿈
`crosswalk_data`(detector) → `/katech_msg/crosswalk_detection`(ped_crosswalk_check_array_msg) → **`katech_ped_detector_can_writer.cpp`** → CAN `Pedestrian_Stat`(ID 528) + `Pedestrian_Stat_1`(529). `katech_test.launch` 기동.
- detector 가 보행자(status 1/2)를 **1~9 중 아무 폴리곤** 안에서 찾으면 `on_crosswalk=1` → CAN `on_crosswalk` 시그널 1. **즉 crosswalk_data 를 1→9 로 늘리면(코드 무변경) 검출·CAN 이 자동 3~9 커버.**
- 단 CAN 은 **crosswalk_id 없음**: `ped_crosswalk_check_msg`=id/status/on_crosswalk/rel_pos_x/y. `on_crosswalk` 은 0/1 플래그, CAN `on_crosswalk_1/2/3` 는 **검출객체 슬롯(0·1·2·3) 인덱스**(횡단보도 번호 아님). writer 는 **객체 4개 cap**(data.size 1~4, >4 는 그 사이클 미전송).

## 스코프/HMI (사용자 확정: 표시 위주)
- occupancy_msg 는 `crosswalk1/2_occupied` **2필드뿐** → 3~9 검출돼도 occupancy·tim-pedes 퓨전엔 안 실림(크래시 없음). occupancy_msg 정의 무변경.
- **`web_hmi/web/threejs/CrosswalkZones.jsx`**: EPSG:5179 **[E,N] 절대좌표**(crosswalk_data 와 동일, **swap 없음** — mat 뷰어 [lat,lon] 과 다름), `CROSSWALK_IDS=Object.keys(CROSSWALK_POLYS)` 루프. **1~9 표시 확장됨(표시 전용)** — 3~9 점멸/배너 없음(경보 `crosswalk_ped_active===id` 인데 fusion 은 active_id 0/1/2 만 발행). 3~9 경보까지=occupancy_msg 확장+fusion LINK_ID 매핑+OBU 소스(단일 RSU 4방향 한계) 필요.

## 실행 이력 (2026-07-15)
1. 사용자가 crosswalk_position.md 에 3~9 추가(원천 편집) → crosswalk_data 1·2 → **1~9 확장** + mat 뷰어 var CROSSWALK 1~9. 점개수 (11,11,12,14,12,12,12,10,10). **1·2 재변환 byte-identical(0m)**, 3~9 max|Δ|=6.8e-07m. 수정 .py+.html 2파일. verifier 7/7 PASS. (commit aaefdc4)
2. 후속: `CrosswalkZones.jsx` 표시 1~9 확장(표시 전용, 좌표=crosswalk_data byte-identical, babel PASS).

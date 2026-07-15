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

## 스코프 함정 (사용자 확정: 좌표+뷰어만)
- occupancy_msg 는 `crosswalk1_occupied`/`crosswalk2_occupied` **2필드뿐** → 3~9 검출돼도 occupancy 에 안 실림(크래시 없음). CAN/HMI/tim-pedes 무변경.
- **형제 파일 `web_hmi/web/threejs/CrosswalkZones.jsx`** 도 EPSG:5179 crosswalk 1·2 보유("crosswalk_data 와 동일 유지" 주석). 범위 밖이라 미변경 → **web_hmi 는 1·2, mat_viewer 는 전체 표시(의도된 스코프 차이)**. 3~9 HMI 연동 필요 시 별도 작업.

## 1회 실행 (2026-07-15)
사용자가 crosswalk_position.md 에 3~9 추가(원천 편집, git M) → crosswalk_data 1·2 → **1~9 확장**. 점개수 (11,11,12,14,12,12,12,10,10). **1·2 재변환 byte-identical(regression 0m)**, 3~9 신규 재변환 max|Δ|=6.8e-07m. 수정 = .py(dict 1 hunk) + .html(1258행 1줄) 정확히 2파일. verifier 7/7 PASS.

---
name: reference-tim-pedes-bag-replay
description: web_hmi_replay.launch 로 bag→web_hmi 재생. tim-pedes 퓨전 own 경로는 구 bag(occupancy/track_Multi_RS 없음)으론 검증 불가 — OBU 경로만 동작
metadata:
  type: reference
---

`src/visualization/web_hmi/launch/web_hmi_replay.launch` = web_hmi(threejs_f1) + `~/bag_data` bag 재생 원샷. args: `bag:=<파일명>`(필수, bag_dir 안), `bag_dir`/`rate`/`open_browser`/`run_fusion`. 기본 동작 = bag 녹화 `/hmi/*` 를 브라우저에 **직접 재생**(녹화 HMI 재현), `/hmi/bag`(REC 플래그)·`/client_count` 만 `/sink` 로 remap. `<node>` 안 `/hmi/*` remap 주석 해제 시 = bag `/hmi/*` 전부 `/sink` 로 빼고 live 브리지 재생성분만 보는 검증 모드. ([[project_bag_replay_hmi]] 수동 remap 레시피를 이 launch 가 대체)

**tim-pedes 퓨전(`/katech_msg/crosswalk_ped_fusion`)을 bag 으로 검증할 때 함정 (own 경로 안 뜸):**
- 퓨전 own 경로는 `/katech_msg/crosswalk_occupancy` 를 구독 → 이 토픽은 **라이브 `katech_ped_detector.py`(입력 `/track_Multi_RS`)만** 발행.
- 구 bag(`~/bag_data/2026-07-02-*` 등)엔 `/katech_msg/crosswalk_detection`(자체 인지, on_crosswalk)+`/localization/to_control_team`+`/obu/v2x_pedes_assistance` 는 있으나 **`/track_Multi_RS`·`/crosswalk_occupancy` 없음** → `own_present` 항상 False, OBU `east/south_pedes`(sparse)만 간헐 트리거(팝업 깜빡).
- `crosswalk_detection` 엔 crosswalk_id 없음(CAN 완전 동결 결정) → 퓨전이 #1/#2 구분 불가, bag 만으론 own 경로 재현 불가.
- own 검증법: (1) 라이브 차량/시뮬(`katech_test.launch` — 검출+퓨전+occupancy 동시), (2) 기능 켠 채 새 bag 재녹화, (3) replay 전용 occupancy 재구성 노드 추가(crosswalk_detection rel_pos + host pose + 검출노드 지오메트리 import).

매핑: #1=south_pedes(link 1239/1238) · #2=east_pedes(link 1205). 팝업색 자체=#ff9800 / OBU=#ff3030 / 둘다=#ff30ff. [[feedback_can_frozen_additive]]

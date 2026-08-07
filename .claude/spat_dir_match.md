---
name: spat-manuaver-movementstatename
description: MQTT/OBU SPaT 신호를 ego 진행방향(local MANUAVER -1/0/1)에 맞는 movement 로 매칭. HMI 방향필터 부재 버그 + STR/STRAIGHT 어휘 혼재
metadata: 
  node_type: memory
  type: project
  originSessionId: c2a74392-f1b6-49d8-91ad-e78f7a6568f9
---

시화 SPaT 신호등을 ego 진행방향에 맞게 표시하는 매칭. 하네스 `spat-dir-match` (2026-07-14, siheung_dev). 상세 [[spat-merge-obu-mqtt]].

## 핵심 버그 (bag 2026-06-24 로 실증)
- **HMI 3경로(web_hmi/pyqt/stat_display) 모두 방향 필터 없이 IID+SignalGroup 만으로 "첫 매칭"** 선택.
- `spat_merge_node` 가 `std::map<(IID,SG,MovementStateName)>` 라 같은 SG 안에서 **MovementStateName 알파벳순 (LEFT<PED<RIGHT<STR)** 으로 나옴 → **항상 LEFT 가 먼저** 잡힘.
- 결과: ego 직진(MANUAVER=0) 타깃 IID 516/518/165 에서 **100% 좌회전(LEFT) 신호 오표시**. ego 좌회전인 302 만 우연히 맞아 그동안 안 드러남.

## 확정 매핑 / 어휘
- local `MANUAVER` (int8, `to_control_team.MANUAVER`): -1→LEFT / 0→STR / 1→RIGHT.
- **`/spat_merged` 는 어휘 혼재**: MQTT 약어 `STR/LEFT/RIGHT/PED/BUS/BYC` + OBU 풀네임 `STRAIGHT/PEDESTRIAN/...`. → 매칭 집합 **LEFT={"LEFT"}, STR={"STR","STRAIGHT"}, RIGHT={"RIGHT"}**.
- **signalGroup 단독 매칭 금지**: 같은 SG 가 다방향 보유(예 IID 302 SG70 = LEFT+STR 둘 다). 방향 문자열이 최종 판별자.
- PED/BUS/BYC/PEDESTRIAN = 비차량 → 어휘에 없어 자연 배제. `MovementStateName` empty → 매칭 안 함(안전 스킵).

## 수정 지점 (선택 로직만, 상류 무변경)
- `spat_CAN_writer.cpp`: 이미 방향매칭 있었으나 `"STR"` 정확일치라 OBU **"STRAIGHT" 놓쳐 CAN 간헐 누락**(IID 165 실증). file-local 헬퍼 `movement_matches_manuaver` + 매칭조건 1줄 교체. **CAN 무손상**(canWrite/DBC/ID/dlc/temp_data/주기 불변, target_name 은 로그용 존치).
- `pyqt_hmi/utils/hmi_state.py`: **web_hmi + pyqt 컨트롤러 공용**. 모듈 헬퍼 + `look_at_manuaver` 저장(`_cb_local`) + `_cb_traffic` SG 체크 직후 방향필터 + 실패 시 신호 0 리셋.
- `stat_display.cpp`: file-local 헬퍼 + `traffic_light_callback` 에 `local_msg.MANUAVER` 방향필터(신규 구독 없음) + 실패 시 color/time 0.
- **`main_window.py`(pyqt 네이티브)는 사용자가 스코프 제외** — 미변경.
- **매칭 실패 시 = 안전 리셋**(직전 신호 잔존 금지). 미검증: RIGHT/우회전 타깃이 이 bag ego 시퀀스에 없어 실증 못 함(로직상 대칭).

## 검증 리플레이 뷰어 (offline)
`spat_viewer` 에 추가: `scripts/extract_spat_replay.py` (bag 3개 → `web/data/replay_timeline.json`, 수정 매칭 헬퍼 동일 복제) + `web/replay.html` (self-contained Leaflet, ego 궤적+화살표+신호등 패널+재생/스크럽, 지도배경은 기존 road_links/intersections.json 재사용). 실행: `bash scripts/serve_http.sh 8080` → `http://localhost:8080/replay.html` (roscore 불필요, 지도 타일 CDN 인터넷 필요). bag 2026-06-24 3개 추출 = 2427 샘플/512s/5Hz, 방향 매칭 전 IID 정합(302→LEFT, 516/517/518→STR, 165/519→STRAIGHT). replay_timeline.json 은 파생물(bag 바뀌면 재생성). 상세 실행법은 [[spat-viewer-run]].

---
name: project-v2x-spat-topic
description: V2X 신호등(SPaT) 활성 토픽은 브랜치마다 정반대 — siheung_dev=/siheung_spat, ioniq5_hmi_dev=/katri_v2x_node/katri_spat. 점검 전 git branch 확인
metadata: 
  node_type: memory
  type: project
  originSessionId: 650e6423-ecf2-4547-b012-121965fa01da
---

**SPaT 활성 토픽은 브랜치마다 정반대다 — 점검 전 반드시 `git branch --show-current` 확인.**

## ioniq5_hmi_dev 브랜치 (2026-06-04 코드 확인)
- 활성 SPaT = **`/katri_v2x_node/katri_spat`** (`v2x_msgs/intersection_array_msg`).
- 발행: `katri_obu_interface_node`(노드명 `katri_v2x_node`), `launch/katech_test.launch:35` 에서 **실제 launch됨**.
- 구독(전부 활성): `spat_CAN_writer`(SPaT→차량 CAN V2X_SPaT_1), `v2x_diagnostic`, `stat_display`, `pyqt_hmi` 기본(`main_window.py:665`)+A-1(`hmi_state.py:149`).
- **이 브랜치엔 `/siheung_spat` 자체가 없음** (siheung_v2x 미사용).
- katri_obu_interface UDP 수신: `~/katri_dsrc/.../decodeSample.c` 가 SIG_SPAT[5] 를 1024B UDP(LOCAL_PORT 50000)로 송신 → 수신측 struct byte-identical(wire 포맷 일치). 2026-06-04 movementName 매핑 버그 수정(`[0]` 1바이트만 읽던 것 → 전체 문자열). **단 `MovementStateName` 은 저장소 어느 소비자도 읽지 않음**(spat_CAN_writer 는 V2X_SPaT_1 `movementName_1` 신호에 상수 0 기록, 숫자 4필드 TimeChangeDetails/MovementPhaseStatus/SignalGroupID/IntersectionID 만 사용) → 수정은 발행 정합성만 개선, 현 실동작 무영향.

## siheung_dev 브랜치 (2026-05-18)
- 활성 SPaT = **`/siheung_spat`** 하나만. 발행: `siheung_v2x_node` (`src/v2x/siheung_v2x/src/j2735_decode.cpp:366`).
- 이 브랜치에선 `/katri_v2x_node/katri_spat` 미사용 (katech_test.launch 에서 katri_v2x_node 주석).

**Why:** 같은 repo라도 브랜치(KATRI 시험장 vs 시흥)마다 SPaT 소스가 정반대. 한쪽 기준으로 "활성/미사용" 단정하면 영향평가가 틀어진다 — 2026-06-04 실제로 ioniq5_hmi_dev 를 siheung_dev 기준으로 오판해 "katri_obu_interface 무영향"이라 잘못 말함.

**How to apply:**
- v2x SPaT 흐름 점검 시 브랜치 먼저 확인 → 위 표대로 활성 토픽 결정.
- `src/v2x/katri_obu_interface/` 는 두 브랜치 공용으로 보존(제거 금지). ioniq5_hmi_dev 에선 활성.
- 송신측 `~/katri_dsrc/decodeSample.c` 는 별도 git repo(KATRI OBU 환경). 버그(1024B sendto가 120B 배열 OOB read, 종료 루프 `for i<10 memset(&sig_SPaT[i],0,120)` OOB write) 있으나 수정 범위 밖 — 담당팀 전달.

참고: [[reference-spat-timestamp]] (MOY/DSecond 절대시각 부재), [[project-active-branch-map]]

---
name: project-v2x-spat-topic
description: siheung_dev 브랜치에서 v2x 신호등(SPaT) 토픽은 /siheung_spat 만 사용. /katri_v2x_node/katri_spat 은 옛 토픽이며 미사용
metadata: 
  node_type: memory
  type: project
  originSessionId: 650e6423-ecf2-4547-b012-121965fa01da
---

siheung_dev 브랜치 (senario / senario3 시나리오) 에서 v2x 신호등 SPaT 토픽은 **`/siheung_spat`** (`v2x_msgs/intersection_array_msg`) 하나만 사용. 발행: `siheung_v2x_node` (`src/v2x/siheung_v2x/src/j2735_decode.cpp:366`).

옛 토픽 **`/katri_v2x_node/katri_spat` 은 이 브랜치에서 사용하지 않음** — `launch/katech_test.launch:44` 에서 katri_v2x_node 자체가 주석 처리되어 발행자가 없음.

**Why:** 2026-05-18 사용자가 명시 ("katri_spat은 이 브랜치에서 쓰지않아", "siheung_spat 만 사용할거고 diagnostic 도 siheung_spat 사용할수있도록 수정해줘").

**How to apply:**
- v2x 신호등 흐름을 점검할 때 `/siheung_spat` 만 정상 경로로 간주.
- 잔여 구독 코드는 2026-05-18 정리 완료 — 두 라인을 `/siheung_spat` 으로 1:1 교체:
  - `src/diagnostic/v2x_diagnostic/lib/v2x_diagnostic_pub.cpp:8` — 메시지 타입(`v2x_msgs::intersection_array_msg`) 변경 없음, 토픽명만 교체. catkin_make --pkg v2x_diagnostic PASS.
  - `src/visualization/pyqt_hmi/scripts/widgets/main_window.py:602` — `traffic_light_callback` 그대로 사용, 토픽명만 교체.
- **`src/v2x/katri_obu_interface/` 패키지는 보존**. `katech_test.launch:44` 의 `<!-- KATRI에서는 OFF -->` 주석처럼 KATRI 시험장 환경용. 시흥(siheung_dev) launch 에서만 비활성. 다른 브랜치/환경에서 사용 가능성 있으니 src 에서 제거하지 말 것.
- `src/diagnostic/hmi_diagnostic/lib/hmi_diagnostic_pub.cpp:6` 의 katri_spat 참조는 주석 — 무관.

참고: [[senario-mat-viewer]], [[map-data]]

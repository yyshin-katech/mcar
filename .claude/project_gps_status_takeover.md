---
name: project_gps_status_takeover
description: "GPS 진단 status가 TOR로 가는 경로는 stat_display 경유, pyqt_hmi는 표시 전용. 고장 기준 RTK Fixed 아님/std 임계"
metadata: 
  node_type: memory
  type: project
  originSessionId: dcc247e8-3d20-4e27-835f-040773301873
---

GPS 고장 판정과 Take_Over_Request(TOR) 연결 구조 (2026-06-10, ioniq5_hmi_dev).

**판정 기준 (pyqt_hmi + stat_display 통일):**
- `cpt7_gps_diagnostic`는 raw값만 발행(GPSRTK_StatCode, lon_std/lat_std, GPS_INS_AliveCnt) — 판정 안 함.
- 고장(status=2): 토픽/AliveCnt 단절 | RTK Fixed 아님(`GPSRTK_StatCode < 2`) | std > 15cm
- 경고(status=1): std > 5cm (15cm 이하)
- 코드: `widgets/main_window.py:_evaluate_diag_status`, `utils/hmi_state.py:_evaluate_diag`(A-1 공유), `stat_display.cpp:GPS_Text_Gen`. 임계 상수 GPS_STD_WARN_M=0.05 / GPS_STD_ERROR_M=0.15.

**TOR 경로 (★핵심):**
`stat_display`가 9개 진단 status를 모아 `/diagnostic/system`(katech_diagnostic_msg)으로 발행 → `to_control_team_demo.py:diag_cb`가 구독 → `any(s != 0)` → `takeoverreq=1` → `Take_Over_Request=1`(to_control_team_demo.py:512) → `/localization/to_control_team` → 제어팀(차량)+블랙박스.

**Why/gotcha:**
- TOR를 먹이는 건 **stat_display의 gps_status**다. pyqt_hmi의 gps_status는 화면 표시 전용이라 TOR에 무관. GPS 판정을 TOR에 반영하려면 stat_display를 반드시 같이 고쳐야 함.
- `any(s != 0)`이라 **경고(status=1)도 TOR를 유발**한다(고장만이 아님). 사용자가 현행 유지 결정(2026-06-10). 고장만 TOR로 좁히려면 to_control_team_demo.py:205를 `s==2`로.
- gps_status 계산은 0.1s지만 `/diagnostic/system` 발행은 1.0s 주기(timerCallback) — TOR까지 최대 ~1s 지연. RTK 플랩핑 시 false TOR 위험([[project_v2x_spat_takeover_decoupled]]와 유사).

**GPS 전원 분리 2대 버그 + 수정 (2026-06-16, ioniq5_hmi_dev):**
- (버그1) `cpt7_diagnostic_pub.cpp`는 `GPS_INS_AliveCnt`를 navpvt가 아닌 **진단 노드 자체 100ms 타이머**로 증가시킴(`alive_cnt++`). GPS 전원 끊겨 `/ublox/navpvt` 멈춰도 AliveCnt는 계속 증가 + `/diagnostic/cpt7_gps`도 계속 발행 → stat_display AliveCnt 정지 감지·pyqt miss_cnt 둘 다 안 걸림. RTK code/std는 navpvt 콜백에서만 갱신돼 마지막 정상값 freeze → **두 HMI 영원히 녹색(실차 재현됨)**. `GPS_INS_SolutionStat`(0x01=no data)은 아무 소비자도 안 봄.
  - 수정: navpvt 연속 미수신 3 tick(300ms)이면 `GPSRTK_StatCode=0` 강제 → 기존 `RTK<2→status=2` 로직 그대로 타서 고장 표출. navpvt 20Hz라 정상 시 오판 없음.
- (버그2) `Take_Over_Request`가 `pose_2d_cb` 안에서만 세팅·발행됨. pose_2d_gps는 navpvt 파생이라 GPS 끊기면 콜백이 죽어 to_control_team 전체가 freeze → takeoverreq=1이 영영 못 나감.
  - 수정: 발행을 `publish_timer_cb`(20Hz 독립 타이머)로 이전. pose_2d_cb는 `self.last_p` 캐시만, 타이머가 매 tick TOR 재적용 후 발행. GPS 단절 무관하게 TOR 전달.

관련: [[project_control_fault_not_displayed]], [[project_pyqt_hmi_primary]], [[project_altitude_can_flow]]

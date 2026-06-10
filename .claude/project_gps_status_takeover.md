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

관련: [[project_control_fault_not_displayed]], [[project_pyqt_hmi_primary]]

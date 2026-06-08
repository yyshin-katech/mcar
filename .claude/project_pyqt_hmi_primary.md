---
name: pyqt_hmi is primary monitoring UI, mirror rviz features there
description: 모니터링 화면은 pyqt_hmi가 메인. rviz의 시각화 기능은 pyqt_hmi에도 동일하게 반영
type: project
originSessionId: 7bc83e9b-1c21-49ec-9db9-d586acbc5f2f
---
차량 모니터링 UI는 **`pyqt_hmi`가 메인**이다. `stat_display`(rviz overlay)는 보조이고, 신규 진단/표시 기능은 둘 다에 추가하거나 pyqt_hmi 우선으로 추가한다.

**Why:** 사용자가 "모니터링 화면은 rviz말고 pyqt_hmi 쓸거야"라고 명시. rviz의 GPS std 표시(`stat_display::GPS_STD_Text_Gen`)도 pyqt_hmi로 옮겨달라고 직접 요청.

**How to apply:**
- **pyqt_hmi는 2개 빌드로 분기됨 (2026-06 기준) — 진단/표시 로직 변경 시 둘 다 반영해야 함:**
  - 기본: `main_display.py` → `widgets/main_window.py` (진단 판정 메서드 `_evaluate_diag_status`)
  - A-1: `main_display_a1.py` → `utils/hmi_state.py`(`HmiStateController._evaluate_diag`) + `widgets_a1/`
  - 한쪽만 고치면 다른 빌드에서 누락됨. 같은 변경을 양쪽에 적용.
- stat_display(rviz overlay)는 보조 — 진단 색상/팝업/사운드 로직이 pyqt_hmi와 매칭. 한쪽 바꾸면 정합성 확인.
- 진단 status 색: 0=정상(green #28a745), 1=경고(orange #ff8c00), 2=에러(red #dc3545). (`widgets/status_indicator.py`)
- 진단 상태 결정 규약: (a) 메시지 끊김(miss_cnt>threshold) → error(2), (b) StatCode 도메인 조건 → warning(1), (c) 정상(0).
- **예외(2026-06-02): VCU는 `VCU_StatCode==1`(life_count 결손)을 device fault로 보고 error(2)로 표시** — 일반 규약(도메인=warning 1)을 깸. 두 빌드 모두 `return 2`. life_count 판정은 vcu_diagnostic_pub가 VCU Info 6종 life_count staleness(0.5s)로 수행. 관련 [[project_active_branch_map]].
- 상응 위젯 위치: `create_left_panel`의 `System Status`(상태 인디케이터), `GPS Information`(GPS std 등), `Speed Information`, `Traffic Light`. 시각화는 `create_vehicle_view` 내 `VehicleViewWidget`.

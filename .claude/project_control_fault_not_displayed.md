---
name: project_control_fault_not_displayed
description: 슈퍼바이저/횡/종 제어 고장 error_code는 어떤 진단/HMI도 평가하지 않아 stat_display 팝업에 표출 안 됨
metadata: 
  node_type: memory
  type: project
  originSessionId: c33bd427-3676-488b-95e5-e46292e27002
---

stat_display rviz 팝업(`/rviz/jsk/popup`)은 제어 고장(슈퍼바이저/횡/종)을 표출하지 못한다. 2단 구조 때문:

1. **진단 노드가 error_code를 안 본다**
   - ADCU 진단(`chassis_CAN_reader.cpp` adcu_diag_timer_callback) = `brain_life_count` 정지(staleness)만 판정. `ioniq5_ad_can.error_code`(슈퍼바이저 고장값) 무시.
   - VCU 진단(`vcu_diagnostic_pub.cpp`) = 6개 life_count staleness만 판정. `v_can.lateral_error_code`/`longitudinal_error_code`(=4) 무시. 횡·종이 VCU 단일 status로 합쳐져 구분 불가.
2. **팝업이 진단 status(0/1/2)만 본다**
   - `stat_display.cpp` system_status_check(): 9개 status 루프에서 abnormal_sensor를 덮어써 count==1이면 마지막 1개, count≥2면 "시스템 고장 (N개)" 통합 표출.

따라서 error_code/brain_status만 바뀌는 고장 주입은 표출 경로가 없다(life_count가 안 멈추므로 status=0 유지). pyqt_hmi/web_hmi도 이 제어 error_code를 소비하지 않는다(grep 0건).

**검증 근거**: bag 2026-06-09-09-13-43...0.bag — ADCU 진단 전 구간 stale(=1), VCU 29.7s·58.2s 정지. 팝업은 전부 staleness 기반이고 주입한 횡/종/슈퍼바이저 error_code와 무관. (2026-06-09 분석, 코딩은 보류)

연결하려면 ① 진단 노드가 error_code를 평가해 status 반영, 또는 ② 팝업이 error_code 직접 구독 필요. 관련: [[project_pyqt_hmi_primary]]

**2026-06-09 업데이트 (①의 일부 적용)**: `vcu_diagnostic`을 AutonomousState(ID 16) 기반으로 교체함. 구독 `/sensors/ioniq5_ad_can`, `autonomous_life_count` staleness(0.5s) OR `error_code != 0` → `VCU_StatCode=1`. 따라서 슈퍼바이저(A, 침묵)·종(B)·횡(C, error_code≠0) 고장이 이제 `/diagnostic/vcu`→stat_display 팝업의 **VCU 슬롯**으로 표출된다. 단 횡/종 구분은 여전히 안 됨(둘 다 supervisor error_code≠0 → VCU 하나로 합쳐짐). 검증 하네스: [[reference_diag_replay_harness]] make_adcan_bag.py.

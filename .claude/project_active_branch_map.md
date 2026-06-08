---
name: project_active_branch_map
description: "활성 브랜치 ioniq5_hmi_dev + K_CITY_20260608 맵, to_control_team_demo가 mat 값을 코드에서 덮어쓴다는 gotcha"
metadata: 
  node_type: memory
  type: project
  originSessionId: c8669ca3-39b3-47d6-8d04-3b1f205a2769
---

2026-06 기준 활성 작업 상태와 주의점.

- **브랜치**: `ioniq5_hmi_dev`
- **활성 맵**: `katech_test.launch`가 `MAPFILE_PATH = mapfiles/K_CITY_20260608` 사용 (rosparam). `local_test.launch`는 `K_CITY_2025`. 맵은 `target_roads[k] = link_{k+1}.mat` 로 로드 (1-based LINK_ID → 0-based 인덱스).

**Why/gotcha:** `to_control_team_demo.py`는 mat 파일의 `Speed_Limit`·신호정보(`look_at_signalGroupID`/`IntersectionID`)·`is_stop_line`을 일단 읽은 뒤, `pose_2d_cb`에서 **LINK_ID별 하드코딩 분기로 덮어쓴다**. 즉 mat 값만 바꿔서는 거동이 안 바뀌는 경우가 많음.

**How to apply:**
- 속도제한/신호정보 변경 요청 시 mat 파일이 아니라 `to_control_team_demo.py`의 LINK_ID 분기를 먼저 확인. Speed_Limit은 여러 하드코딩 리스트로 덮어씀(예: [39~51]→40, [65,76~79]→10, else→30). 이력: else 기본 30(2026-06-01 15→30), link 5/8/12/... 15 강제 제거(2026-06-02), **link 12를 속도10 리스트에서 제거→else 30(2026-06-08, mat도 30)**.
- 링크 매칭은 frenet min-|d| 방식(`compute_my_lane_cy`). 물리적으로 겹치는 링크(예: 78이 끝부분 s≈27,36~37m에서 79와 겹침)는 조기 전환되므로, old_lane_id 기반 hysteresis로 처리(2026-06-02 78→79 적용).
- 관련 UI: [[project_pyqt_hmi_primary]]

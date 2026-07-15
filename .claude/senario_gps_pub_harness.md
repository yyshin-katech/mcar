---
name: senario-gps-pub harness
description: senario 시나리오 HTML 의 link 시퀀스 → 40 km/h 일정 GPS 시뮬레이션 publisher 신규 개발 파이프라인 (.claude/agents 3개 + skills/senario-gps-pub)
type: project
originSessionId: 8099a2fb-c590-474a-b3ff-3eb9f9dff9be
---
# senario-gps-pub 하네스

**목적:** `src/localization/gps_system_localizer/mapfiles/senario/mat_viewer_senario_*.html` 의 주행 link 시퀀스를 따라 GPS 시뮬레이션 데이터를 발행하는 ROS publisher 를 신규 작성.

**Why:** 기존 `test_senario3_publisher.py` 는 senario3 디렉토리/30 km/h 전용. 새 시나리오(senario, 40 km/h, 간단 동역학) 마다 매번 손으로 복붙하는 비용을 줄이고, 사양·코드·검증을 분리해 외과적 변경을 강제.

**How to apply:** "senario gps publisher 만들어", "mat 시나리오 시뮬", "40km/h GPS 시뮬", "route 바꿔서 다시" 등 요청 시 `senario-gps-pub` 스킬을 호출. 스킬은 analyst → coder → verifier 3-에이전트 파이프라인을 자동 실행하고, 신규 publisher .py 1개를 `gps_system_localizer/src/` 에 생성.

**구성요소:**
- agents: senario-sim-analyst (HTML 파싱 + 기존 publisher 패턴 분석), senario-sim-coder (사양서대로 .py 작성), senario-sim-verifier (py_compile + import dry-run + roscore + rostopic echo)
- skill: `.claude/skills/senario-gps-pub/SKILL.md` (오케스트레이터)
- 작업 디렉토리: `_workspace/01_analyst_spec.md` → `_workspace/02_coder_report.md` → `_workspace/03_verifier_report.md`

**메시지/토픽 (기존 publisher 와 동일):** `/localization/pose_2d_gps` (mmc_msgs/localization2D_msg), EPSG=5179.

**사용자 메모:** 사용자가 "NavPVT 등" 이라 표현해도 실제 코드 패턴은 localization2D_msg. 코드 패턴이 사용자 표현보다 우선.

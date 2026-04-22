---
name: ros-agent-team-execution
description: ROS 에이전트팀 실행/종료 패턴 — 백그라운드 실행, tmux 분할 불필요, 종료 시 kill 필요
type: feedback
---

ROS 에이전트팀 실행 시 tmux pane 분할하지 말 것. 백그라운드 프로세스로 실행하면 충분.

**Why:** 사용자가 pane 분할을 요청한 적 없었고, 실제로 불필요하다고 피드백함.
**How to apply:**
- roscore, launch, publisher, monitor 등을 `run_in_background`로 실행. tmux pane/window 생성 안 함.
- 실행 순서: roscore(즉시) → launch파일(+3~5s) → publisher(+10s) → monitor(+12s)
- 종료 시 `rosnode kill -a` + `killall roscore rosmaster rosout`만으로 부족할 수 있음.
  tmux pane이나 백그라운드에서 실행한 프로세스가 좀비로 남을 수 있으므로, `ps aux | grep ros` 확인 후 `kill` 필요.
- 에이전트팀 종료 요청 시: ROS 노드 종료 → 잔여 프로세스 확인 → 강제 kill까지 완료해야 함.

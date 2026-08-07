---
name: bag replay dual-publisher trap for web_hmi
description: 시흥/오이도 bag에 /hmi/* 출력 토픽까지 녹화돼 있어 web_hmi.launch와 함께 재생 시 dual-publisher 발생. live 브리지의 필터/수정이 안 먹는 것처럼 보이고 REC 표시가 자동으로 켜진다. /hmi/*:=/sink/* remap 필수.
type: project
originSessionId: 2732ef96-e301-44d6-abdd-0935ca63a907
---
`bag_data/20260508/2026-05-08-11-19-30_*` 5분할 bag에는 `/hmi/state` (1385 msg) `/hmi/bag` (1, latched) `/hmi/threejs/tracks` (214) `/hmi/threejs/map` (1, latched) `/hmi/diagnostics` `/hmi/objects` `/hmi/popup` `/hmi/topic_hz` 등 web HMI **출력** 토픽까지 통째로 녹화돼 있다. 그대로 `rosbag play` + `roslaunch web_hmi web_hmi.launch` 동시 실행하면 모든 `/hmi/*`에 publisher 2개 (`/web_hmi_bridge` + `/play_*`) → 브라우저는 마지막 도착 메시지로 표시.

**Why:** dual publisher 상황에서 (1) 새로 추가한 브리지측 필터(예: `PERCEPT_MIN_CONFIDENCE=0.9`)가 안 먹는 것처럼 보임 — 실제로는 정상이지만 bag의 옛날 ghost 트랙이 동시에 publish돼서 차단 효과가 안 보임. (2) `/hmi/bag` latched의 `recording:true`가 그대로 표시돼 web HMI가 자동으로 REC 상태로 시작하는 것처럼 보임. 2026-05-10 세션에서 ghost 트랙 깜빡임 디버깅 1시간 + 자동 REC 표시 디버깅 추가 손해. qt_hmi 케이스(`project_qt_hmi.md`)는 "bag와 web_hmi.launch 함께 띄우지 말기"가 답이지만 **web_hmi 자체를 bag로 검증할 때는 remap이 답**.

**How to apply:** web_hmi 변경을 bag로 검증할 때 다음 remap 풀세트 사용:
```
rosbag play --clock --loop <bags> \
  /hmi/bag:=/sink/bag \
  /hmi/state:=/sink/state \
  /hmi/diagnostics:=/sink/diagnostics \
  /hmi/objects:=/sink/objects \
  /hmi/popup:=/sink/popup \
  /hmi/topic_hz:=/sink/topic_hz \
  /hmi/map:=/sink/map \
  /hmi/threejs/map:=/sink/threejs_map \
  /hmi/threejs/tracks:=/sink/threejs_tracks \
  /hmi/threejs/free_space:=/sink/threejs_free_space \
  /hmi/threejs/traffic_lights:=/sink/threejs_traffic_lights \
  /hmi/ego_pose:=/sink/ego_pose
```
검증 첫 단계: `rostopic info /hmi/threejs/tracks` 등에서 publisher가 `/web_hmi_bridge*`만 남는지 확인. `/play_*`가 보이면 remap 누락. SIGKILL된 rosbag 등록이 ROS Master에 stale로 남으면 `rosnode cleanup`으로 정리 (안 하면 rostopic info에 죽은 publisher가 계속 보임).

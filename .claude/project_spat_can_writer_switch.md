---
name: project-spat-can-writer-switch
description: "spat_CAN_writer.cpp의 교차로→CAN 매핑 switch 구조 — temp_intersection_id_msg는 dead store, 실제 (교차로,그룹) 발화는 look_at 값으로 결정"
metadata: 
  node_type: memory
  type: project
  originSessionId: a9dcef0e-428b-4662-aca7-bdb02e5d68c6
---

`src/sensing/can/src/spat_CAN_writer.cpp` CALLBACK_SPAT 의 교차로 처리 구조 (2026-06-17 확인, ioniq5_hmi_dev).

- **매칭**: `g_intersection_id`/`g_signalGroup_id`(= `to_control_team` 의 `look_at_IntersectionID`/`look_at_signalGroupID`)와 수신 SPaT의 `IntersectionID`/`SignalGroupID`가 둘 다 같아야 V2X_SPaT_1 CAN 송신. 즉 (교차로,그룹) 쌍 정의의 원천은 이 파일이 아니라 `to_control_team_demo.py` 의 look_at (맵 target_roads + LINK_ID 하드코딩 오버라이드, 예: LINK_ID==20 → 1300/3).
- **switch(intersection_id)**: 매칭 성공 시 `temp_intersection_id_msg = 교차로ID/100` 규칙으로 값 설정 (200→2,300→3,400→4,610→6,700→7). **이 변수는 어디서도 읽히지 않는 dead store** — switch는 cosmetic. signalGroup은 switch에 안 들어감.
- 2026-06-17 요청으로 `case(100)→1`, `case(1500)→15` 추가. signalGroup 16/10은 switch 무관, look_at 경유로 매칭됨. to_control_team 쪽 look_at 발행은 미연결(요청 범위 밖).

**How to apply:** "교차로 N에 그룹 M 처리"류 요청은 switch만으로 발화 안 됨 — to_control_team이 그 시점에 look_at_IntersectionID/signalGroupID를 보내야 실제 CAN 송신됨. 발화 미동작 디버깅 시 look_at 발행부부터 확인.

참고: [[project-v2x-spat-topic]], [[project-v2x-spat-takeover-decoupled]]

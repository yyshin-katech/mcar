---
name: feedback-can-frozen-additive
description: 차량 CAN으로 나가는 토픽은 수정 대신 additive 별도 토픽으로 우회(CAN 완전 동결) 선호
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f42f36a4-15cf-4556-8a41-a5d1d1e540f8
---

새 기능이 차량 CAN(예: `Pedestrian_Stat`)으로 발행되는 토픽/메시지(`/katech_msg/crosswalk_detection` 등)의 데이터를 필요로 할 때, 사용자는 그 토픽/메시지를 수정(필드 추가·잠재버그 수정 포함)하는 대신 **기존 CAN 경로를 한 글자도 안 건드리고 additive 신규 토픽을 추가**하는 방식(선택지 "CAN 완전 동결")을 선호한다.

**Why:** CAN 변경은 차량 거동에 직결되고, 잠재버그 수정조차 특정 케이스(예: 동시 2~4명 검출)에서 CAN 바이트를 바꿀 수 있어 CANoe 재검증 부담이 생긴다. 정확성 개선이라도 "무손상"이 우선.

**How to apply:** CAN-feeding 토픽에서 파생 데이터가 필요하면 그 토픽을 건드리지 말고, 정확 계산한 신규 토픽을 별도 발행해 소비한다. 무변경 증명은 `git diff ∅` / sha256 동일. 예: tim-pedes-display 하네스에서 `/katech_msg/crosswalk_occupancy` 를 신설(검출노드 additive)하고 퓨전 노드가 이를 구독. [[canoe_tool]] [[feedback_can_request_signals]]

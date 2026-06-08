---
name: reference_spat_timestamp
description: "SPaT 송신시각 필드 - OBU는 MOY+DSecond 있으나 9시간 오프셋, MQTT는 MOY=0"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 8a0b5080-1af5-413b-8513-86051c70a511
---

v2x_msgs/intersection_msg 의 송신시각 필드는 `MinuteOfTheYear`(MOY, 올해 1/1 00:00 UTC 기준 경과 분) + `DSecond`(그 분 내 ms, 0~60999). 둘을 합쳐야 절대 송신시각.

2026-05-29 OBU(`/siheung_spat`) vs MQTT(`/siheung_v2x/mqtt_spat`) 라이브 비교 결과:

- **OBU**: MOY 채워짐(0% zero). 단 표준대로 UTC 환산 시 **실제 시각보다 정확히 -540분(-9.0시간)** 어긋남 = KST↔UTC 타임존 오류(컨트롤러/OBU 시계가 KST를 UTC로 표기). DSecond(분 내 ms)는 정상. 절대 송신시각 쓰려면 +9h 보정 필요.
- **MQTT**: MOY **항상 0**(204/204). DSecond(ms)만 옴 → 분/일자 복원 불가, 절대 송신시각 없음. 클라우드 V2N 경로에서 MOY 누락.
- **top-level `msg.time`**: 송신시각 아님 — 디코더 노드(siheung_v2x_node / mqtt_spat_rx_node)가 수신 시점에 찍는 ROS 로컬시각.

같은 IID 509 비교 시 남은시간(TimeChangeDetails)은 두 토픽 동일(±1초, MQTT가 ~3Hz로 OBU ~28Hz보다 느려 stale). 단 MovementPhaseStatus 는 SG50/70 에서 OBU=5(permissive) vs MQTT=6(protected) 로 표기 다름. 관련 [[v2x_package]] [[project_v2x_spat_topic]].

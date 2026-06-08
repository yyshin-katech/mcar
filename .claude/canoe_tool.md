---
name: CANoe Tool
description: CAN 송신 검증/분석에 Vector CANoe를 사용한다 — rate/시점 이슈 디버깅 시 CANoe 로그 기반으로 보고함
type: reference
originSessionId: a6eb69e2-ca85-40c8-af59-d781c0bd2015
---
- 사용자는 차량 CAN 신호를 받아서 분석할 때 Vector CANoe를 사용한다.
- "canoe에서 받아서 변환해보면 ..." 같은 보고는 CANoe의 측정/디코드 결과 기반.
- CAN writer(track_CAN_writer 등)의 송신 주기/timing 이상은 CANoe에서 확인되는 경우가 많음.
- DBC: `src/sensing/can/dbc/CANdb_IONIQev_PCAN2.dbc` (track_CAN_writer_no_grid가 사용)
- 디버깅 시 CANoe 화면/로그를 사용자가 직접 보고 알려주는 경우가 많으므로, "CANoe에서 어떻게 보이는지"를 기준으로 가설을 세우는 게 유효.

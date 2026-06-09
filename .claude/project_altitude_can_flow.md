---
name: project_altitude_can_flow
description: 해발고도(hMSL)를 CAN으로 송신하는 흐름과 DBC v7 신설 (2026-06)
metadata: 
  node_type: memory
  type: project
  originSessionId: a7f0eb77-93e1-44ec-a5d2-d5e8b4600adc
---

2026-06-10 추가. ublox 해발고도(MSL)를 X/Y와 동일 경로로 CAN까지 전송.

흐름: `/ublox/navpvt` hMSL(mm)
→ `gps_world_tf_node_send_host_gps.cpp`: `localization2D_msg.altitude = hMSL*1e-3` (m, MSL)
→ `to_control_team_demo.py` pose_2d_cb: `p.host_altitude = msg.altitude`
→ `local_CAN_writer.cpp`: `CAR_EGO_A_Ex.ALTITUDE` 로 송신.

CAN 시그널: 메시지 `CAR_EGO_A_Ex` (id **1830 / 0x726**), DLC **4→8** 확장. 신규 시그널 `ALTITUDE` = `39|32@0-` **32비트 IEEE float**(SIG_VALTYPE_ 필요), 단위 m, byte 4-7. 기존 X_High/Y_High는 byte 0-3 유지. X/Y/YAW와 동일한 float 시그널 방식.

**비자명 gotcha:**
- DBC 버전 = **파일별 분리** 관례 (v3~v7 모두 별도 파일 보존). 버전업 = 새 파일 생성 후 코드의 `strcpy(filename, ".../v7.dbc")` 경로 변경.
- CLAUDE.md/README는 chassis_CAN_reader=v4 라고 하지만 **실제 코드는 v6 사용 중이었고 → 2026-06 양쪽 모두 v7로 변경**. `local_CAN_writer.cpp`(송신)+`chassis_CAN_reader.cpp`(수신) 둘 다 v7.
- `local_CAN_writer`는 `dlc=8` 고정이라 DBC DLC와 무관하게 **항상 8바이트 송신** (그래서 CAR_EGO_A_Ex byte4-7이 비어있어도 버스엔 나감).
- 제어팀이 고도를 수신하려면 제어팀 측 DBC에도 v7 CAR_EGO_A_Ex 반영 필요 (이번 작업은 차량 송신부만).

검증: cantools 라운드트립 통과 + can/gps_world_tf/mmc_msgs 빌드 OK. 실차/CANoe rate·timing 검증은 미실시. [[canoe_tool]] [[project_active_branch_map]]

---
name: ioniq5 브랜치 2026-04 작업 이력
description: ioniq5 브랜치 주요 변경사항 — DBC v5 통합, DTG_CAN_writer, IONIQ5_CAN_reader 등
type: project
originSessionId: a15bb8d9-1d28-417f-befe-6f4494f4b54e
---
## 2026-04-16 작업

### DBC v5 통합
- CANdb_IONIQ5_AD_CAN_v5.dbc 추가 (v4 대비: WHL_SPD11/SAS11/ESP12 제거, MOTOR_RPM 원복, MD_AD_Req 추가)
- 전체 CAN 노드(8개 파일) DBC 참조를 v5로 통일

### 신규/변경 모듈
- **DTG_CAN_writer.cpp** 신규 생성 (ch3, 2gen-2ch-C_IoniqEV_v2.dbc)
  - ESP12, SAS11, CGW_PC4, WHL_SPD11 메시지 전송
  - /sensors/v_can 구독, CYL_PRES/SAS_Speed는 0 전송
- **IONIQ_CAN_reader → IONIQ5_CAN_reader** 파일명 변경
- **local_CAN_writer**: MD_AD_Req 시그널 추가 (On_ODD_Stat), /vehicle/mode_command 구독
- **vspd_CAN_writer**: launch에서 주석 처리

### 작업 목록
- claude_work_list/work_20260416.md에 작업 내용 및 완료 시간 기록

**Why:** IONIQ5 차량 CAN 통신 체계 재정비, DTG 데이터 전송 기능 추가
**How to apply:** CAN 관련 작업 시 v5 DBC 기준으로 진행. DTG_CAN_writer는 ch3 전용.

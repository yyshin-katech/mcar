---
name: CAN 채널 할당표
description: IONIQ5 CAN 노드별 채널, DBC, R/W 매핑 (ioniq5 브랜치 기준)
type: project
originSessionId: a15bb8d9-1d28-417f-befe-6f4494f4b54e
---
CAN 채널 할당표 (ioniq5 브랜치, 2026-04-16 기준)

| 채널 | 노드 | DBC | R/W | Launch |
|------|------|-----|-----|--------|
| ch0 | chassis_CAN_reader | AD CAN v5 | Read | katech_test |
| ch0 | local_CAN_writer | AD CAN v5 | Write | katech_test |
| ch0 | spat_CAN_writer | AD CAN v5 | Write | katech_test |
| ch0 | katech_ped_detector_can_writer | AD CAN v5 | Write | katech_test |
| ch0 | diagnostic_CAN_writer | AD CAN v5 | Write | diagnostic_only |
| ch0 | gps_diagnostic_CAN_writer | AD CAN v5 | Write | (미사용) |
| ch0 | vspd_CAN_writer | AD CAN v5 | Write | 주석 처리됨 |
| ch0 | cpt7_CAN_writer | AD CAN v5 | Write | 주석 처리됨 |
| ch1 | track_CAN_writer_no_grid | PCAN2 DBC | Write | katech_test |
| ch2 | IONIQ5_CAN_reader | V_CAN_Release.dbc | Read | katech_test |
| ch3 | DTG_CAN_writer | 2gen-2ch-C_IoniqEV_v2.dbc | Write | katech_test |

**Why:** 채널 충돌 방지 및 DBC 버전 추적 필요. ch0에 다수 노드 집중.
**How to apply:** CAN 노드 추가/수정 시 채널 겹침 확인. 모든 AD CAN 노드는 v5 DBC 통일됨.

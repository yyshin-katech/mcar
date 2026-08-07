# CAN 패키지 상세

## DBC 파일 매핑
| DBC 파일 | 용도 | 사용처 |
|----------|------|--------|
| `CANdb_IONIQ5_AD_CAN_v3.dbc` | AD CAN (ch0) | chassis_CAN_reader, local/spat/diagnostic/cpt7/gps_diagnostic CAN writer, katech_ped_detector |
| `CANdb_IONIQev_PCAN2.dbc` | PCAN2 | track_CAN_writer_no_grid |
| `V_CAN_Release.dbc` | V-CAN (ch2, CAN FD) | IONIQ_CAN_reader |

## CAN FD 설정 (IONIQ_CAN_reader)
- Arbitration: 500K (`canBITRATE_500K`)
- Data phase: 1M (`canFD_BITRATE_1M_80P`)
- `canOPEN_CAN_FD` 플래그 필수
- 데이터 버퍼: 64바이트 (`can_data[64]`)
- WheelInfo (ID 113, 16bytes), DynamicInfo (ID 112, 12bytes)가 CAN FD

## 커스텀 메시지
| msg 파일 | 토픽 | 노드 |
|----------|------|------|
| `ioniq5_ad_can_msg.msg` | `/sensors/ioniq5_ad_can` | chassis_CAN_reader |
| `v_can_msg.msg` | `/sensors/v_can` | IONIQ_CAN_reader |

## V_CAN_Release.dbc 메시지 (14개)
### Info (차량→AD)
- GearInfo (117), TurnSignalInfo (116), LongitudinalInfo (115)
- SteeringInfo (114), WheelInfo (113, FD), DynamicInfo (112, FD)

### Control (AD→차량)
- TurnSignalControl (84), LateralControl (83), GearControl (82), LongitudinalControl (81)

### State (피드백)
- GearState (21), TurnSignalState (20), LongitudinalState (19), LateralState (18)

## CANdb_IONIQ5_AD_CAN_v3.dbc 주요 메시지 (chassis_CAN_reader)
- OperationControl (17): operation_sw, autonomous_sw, emergency_sw
- LateralControl (83): steering_control_mode, target_steering_angle
- GearControl (82): gear_control_mode, target_gear
- BrainState (33): life_count, brain_status
- AutonomousState (16): operation_mode, autonomous_mode, error_code, warning_code

## chassis_CAN_reader 추가 기능
- `/sensors/ioniq5_ad_can` 자체 구독 → BrainState life_count 추출
- 0.1초 타이머로 life_count 변화 감시 → `/diagnostic/adcu` 퍼블리시 (k_adcu_diagnostic_msg)

## launch 노드 (katech_test.launch) - 활성화 상태
- CAN_channel_initializer, chassis_CAN_reader, percept_topic_matcher
- local_CAN_writer, IONIQ_CAN_reader
- track_CAN_writer_no_grid, spat_CAN_writer
- katri_v2x_node, katech_ped_detector, ped_detector_can_writer
- 주석 처리: cpt7_topic_matcher, cpt7_CAN_writer

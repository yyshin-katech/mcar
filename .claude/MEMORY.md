# mcar_v13 프로젝트 메모리

## 프로젝트 개요
- ROS (noetic) 기반 자율주행 시스템
- 경로: `/home/ads/mcar_v13`
- 메인 브랜치: `main`, 작업 브랜치: `ioniq5`
- GitHub: `yyshin-katech/mcar`

## 주요 패키지 구조
- `src/sensing/can/` - CAN 통신 (Kvaser canlib/kvaDbLib)
- `src/msgs/katech_custom_msgs/` - 커스텀 메시지 정의
- `src/msgs/mmc_msgs/` - 기존 메시지 정의
- `src/localization/` - GPS/localization
- `src/visualization/` - HMI, rviz
- `src/diagnostic/` - 각종 진단 노드
- 상세: [can_package.md](can_package.md)

## 설정
- `.claude/settings.json`: `bypassPermissions` 모드
- 사용자 언어: 한국어

## 커밋/푸시 규칙
- 커밋·푸시 시 메모리 파일도 프로젝트 `.claude/`에 동기화하여 함께 커밋
- 메모리 원본: `/home/ads/.claude/projects/-home-ads-mcar-v13/memory/`
- 복사 대상: `/home/ads/mcar_v13/.claude/`
- 대상 파일: `MEMORY.md`, `can_package.md` (메모리 파일 추가 시 갱신)

## 최근 작업 이력
- DBC 파일을 `CANdb_IONIQev_PCAN1.dbc` → `CANdb_IONIQ5_AD_CAN_v3.dbc`로 변경 (7개 src 파일)
- `IONIQ_CAN_reader.cpp`: `V_CAN_Release.dbc` 사용, CAN FD 지원 (500K/1M), 14개 전체 메시지 수신
- `CHASSIS_CAN_READER`: 기존 VCU 메시지 → IONIQ5 AD CAN 메시지로 교체
- launch에서 `vision_CAN_reader`, `front_RADAR_CAN_reader` 제거
- `vcu_diagnostic`: `/sensors/v_can` 구독, GearInfo `life_count`로 VCU 상태 체크
- `chassis_CAN_reader`: BrainState `life_count`로 ADCU diagnostic 추가 (`/diagnostic/adcu` 퍼블리시)
- launch: `chassis_CAN_reader`, `IONIQ_CAN_reader` 활성화
- `cpt7_gps_diagnostic`: Novatel → ublox NavPVT(`/ublox/navpvt`) 구독으로 변경
- `stat_display`: GPS 색상 판단을 NavPVT fixType 기반으로 변경
- `stat_display`: LIDAR `lidar_status=2` 강제 덮어쓰기 버그 수정, CAM/RADAR 항상 정상 처리

## Diagnostic 구조
| 토픽 | 메시지 타입 | 소스 노드 | 판단 기준 |
|------|-------------|-----------|-----------|
| `/diagnostic/vcu` | `vcu_diagnostic_msg` | vcu_diagnostic | V_CAN GearInfo life_count |
| `/diagnostic/adcu` | `k_adcu_diagnostic_msg` | chassis_CAN_reader | AD_CAN BrainState life_count |
| `/diagnostic/cpt7_gps` | `cpt7_gps_diagnostic_msg` | cpt7_gps_diagnostic | ublox NavPVT fixType |
- 공통 패턴: 콜백에서 msg_received 플래그 설정, 타이머에서 플래그 확인 후 리셋
- GPS fixType: 0=NO_FIX, 2=2D, 3=3D(정상), 4=GNSS+DR(정상)
- stat_display GPS 색상: fixType>=3 초록, <3 주황, 통신끊김 빨강

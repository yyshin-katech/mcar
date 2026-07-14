---
name: Project Build & Branch Status
description: mcar_v13 ROS 워크스페이스 빌드 환경 및 브랜치 현황
type: project
originSessionId: 2c8d51a4-b376-4ecd-aff2-58a2c295179f
---
빌드 명령: `catkin_make` (working dir: `/home/katech/mcar_v13`)

**Why:** CLAUDE.md에 `/home/ads/mcar_v13`으로 기재되어 있으나 실제 경로는 `/home/katech/mcar_v13`
**How to apply:** 빌드 시 항상 `/home/katech/mcar_v13`에서 실행

## 브랜치 구조 (2026-04-17 기준)

| 브랜치 | 용도 |
|--------|------|
| `ioniq5` | 현재 주 작업 브랜치 |
| `dev` | 개발 통합 브랜치 |
| `main` | 메인 브랜치 |
| `siheung_dev` | 시흥 테스트 브랜치 |
| `katech_aeb` | AEB 기능 브랜치 (로컬 없음) |
| `koras`, `251001_kcity_test` | 원격 전용 브랜치 |

## 브랜치 전환 직후 첫 빌드 실패 함정 (2026-07-10)

브랜치를 전환하고 바로 `catkin_make` 하면 **첫 빌드가 msg 헤더 경합으로 실패**할 수 있다. 재빌드하면 통과.

- **증상 예**: `siheung_dev` ↔ `ioniq5_hmi_dev` 전환 후 `mqtt_spat_rx_node.cpp:102: error: 'to_control_team_from_local_msg' has no member named 'MANUAVER'`
- **원인**: 두 브랜치의 `.msg` 정의가 다른데(`MANUAVER` 필드는 siheung_dev 전용, ioniq5_hmi_dev엔 없음) `devel/`는 git-ignore라 브랜치 간 공유됨. 전환 후 `make -j8` 병렬 빌드에서 `mmc_msgs` 헤더 **재생성이 끝나기 전에** 소비 노드(siheung_v2x)가 옛 브랜치 헤더로 먼저 컴파일 → transient 실패.
- **How to apply**: 코드 버그로 오인 말 것. 생성 헤더(`devel/include/<pkg>/<Msg>.h`)에 해당 필드가 이미 있고 mtime이 `.msg`보다 최신이면 경합 확정 → `catkin_make` 한 번 더 실행하면 EXIT=0. (근본 회피는 `catkin_make <msg_pkg>` 먼저 또는 `-j1`이지만, 재실행이 가장 싸다.)

## 최근 변경사항 (2026-04-17 풀 기준)

- DBC v5 추가: `CANdb_IONIQ5_AD_CAN_v5.dbc`
- `DTG_CAN_writer.cpp` 신규 추가
- `IONIQ_CAN_reader.cpp` → `IONIQ5_CAN_reader.cpp` 리네임
- HMI (`main_window.py`, `vehicle_view.py`) 대규모 수정
- `CLAUDE.md` 프로젝트 루트에 추가됨

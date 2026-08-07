---
name: Project Build & Branch Status
description: mcar_v13 ROS 워크스페이스 빌드 환경 및 브랜치 현황
type: project
originSessionId: 2c8d51a4-b376-4ecd-aff2-58a2c295179f
modified: 2026-08-07T04:34:30.248Z
---
빌드 명령: `catkin_make` (working dir: `/home/katech/mcar_v13`)

**Why:** CLAUDE.md에 `/home/ads/mcar_v13`으로 기재되어 있으나 실제 경로는 `/home/katech/mcar_v13`
**How to apply:** 빌드 시 항상 `/home/katech/mcar_v13`에서 실행

## 브랜치 = 차량 플랫폼 구분 (2026-08-07 확정)

**브랜치 계열이 곧 차량 버전이다.** 코드/git 이력만으로는 알 수 없는 사실.

| 계열 | 차량 |
|------|------|
| `siheung_*` (`siheung_dev`, `siheung_release`) | **아이오닉 EV** |
| `ioniq5_*` (`ioniq5_release`, `ioniq5_kcity_tested`, `ioniq5_siheung_dev`) | **IONIQ 5** |

**Why:** 두 계열은 차량 자체가 달라서 DBC·CAN 시그널·메시지 정의(`.msg`)·맵이 갈라진다. CLAUDE.md 상단에는 "IONIQ 5"로만 적혀 있어 `siheung_*` 도 IONIQ 5 인 줄 오해하기 쉽다.
**How to apply:** 브랜치 간 코드를 옮기거나 비교할 때 차량이 다르다는 전제로 볼 것. 특히 CAN/DBC·`.msg` 필드 차이는 버그가 아니라 플랫폼 차이일 수 있다. 두 계열의 공통 조상은 `main`(`cbe9d3c`).

## 브랜치 구조 (2026-08-07 기준)

| 브랜치 | 계열 | 용도 |
|--------|------|------|
| `ioniq5_siheung_dev` | IONIQ 5 | 개발 (2026-08-07 `ioniq5_release` 에서 분기) |
| `ioniq5_release` | IONIQ 5 | 릴리스 (구 `ioniq5`) |
| `ioniq5_kcity_tested` | IONIQ 5 | K-City 검증 완료 스냅샷 (구 `ioniq5_hmi_dev`) |
| `siheung_dev` | 아이오닉 EV | 시흥 시범운행 개발 |
| `siheung_release` | 아이오닉 EV | 시흥 릴리스 |
| `main` | — | 메인 (두 계열 공통 조상) |
| `dev` | — | 개발 통합 |
| `katech_aeb`, `koras`, `251001_kcity_test` | — | 원격 전용 |

2026-08-07 리네임: `ioniq5`→`ioniq5_release`, `ioniq5_hmi_dev`→`ioniq5_kcity_tested` (로컬·원격 모두, 구 원격 브랜치 삭제됨). 아래 함정 항목의 `ioniq5_hmi_dev` 는 현 `ioniq5_kcity_tested`.

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

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

## 최근 변경사항 (2026-04-17 풀 기준)

- DBC v5 추가: `CANdb_IONIQ5_AD_CAN_v5.dbc`
- `DTG_CAN_writer.cpp` 신규 추가
- `IONIQ_CAN_reader.cpp` → `IONIQ5_CAN_reader.cpp` 리네임
- HMI (`main_window.py`, `vehicle_view.py`) 대규모 수정
- `CLAUDE.md` 프로젝트 루트에 추가됨

---
name: Map Data
description: senario / senario3 mat 파일 구조, MANUAVER 라벨 관리 원칙
type: project
originSessionId: 02987e96-3bce-4338-b524-4cc3596fca14
---
## senario3 mat 파일 (239개)
경로: `src/localization/gps_system_localizer/mapfiles/senario3/`
- `MANUAVER` 필드 있음, `guard_zone` 없음
- 신호-bearing(signal/intID != 0): 68개, MANUAVER LEFT 11 / RIGHT 7
- SHP 원본: `src/localization/gps_system_localizer/src/shp_map/senario3/TB_senario_map_senario3.shp` (fid=LINK_ID, mc_MANEUVE 매핑)

## senario mat 파일 (308개, 2026-05-18 정리 후)
경로: `src/localization/gps_system_localizer/mapfiles/senario/`
- `MANUAVER` 필드 추가(2026-05-18), `guard_zone` 필드 존재하지만 **현재 모두 0** (이전에 8개 1이었으나 일괄 0 처리)
- 신호-bearing: 113개, MANUAVER LEFT 9 / RIGHT 3
- 회전차선 라벨 원천: `claude_work_list/curve_lane.md` (사용자 관리)
  - LEFT(-1): 417, 442, 499, 548, 946, 973, 1878, 2540, 2541
  - RIGHT(+1): 1205, 1239, 1242
  - 나머지는 0 (STRAIGHT)
- 2026-05-18 정리 작업 입력: `claude_work_list/20260518_mat_file_update.md`
  - 완전 제거 14개, 앞/뒤 트림 12개(정확히 N m, 보간 사용), NEXT_LINK_ID 변경 다수
  - 완전 제거된 link_id를 NEXT/RIGHT/LEFT로 참조하던 다른 mat 파일은 자동으로 0 처리

## senario1 mat 파일
- MANUAVER 필드 **없음** (64개 신호-bearing 링크 전부 누락) — 의도적으로 미작업

## launch 기본값 (`launch/katech_test.launch`)
- `mat_scenario` default: `senario` (2026-05-18 기준)
- `shp_scenario` default: `HDMap_Oido_New`
- 시나리오 전환: `roslaunch ... mat_scenario:=senario3 shp_scenario:=senario3`

**Why:** 신호등 매칭은 to_control_team_demo.py가 mat의 look_at_signalGroupID/look_at_IntersectionID/MANUAVER를 SPaT 필터로 전달. MANUAVER가 누락되면 default 0(STRAIGHT)로 떨어져 좌/우회전 차선에서 신호 매칭 실패.

**How to apply:**
- senario에 mat 파일 추가/변경 시 MANUAVER 필드 보존
- 회전차선 변경은 `curve_lane.md` 갱신 후 `~/temp/harness_to_control_team/add_manuaver.py` 재실행
- senario1 사용 가능성이 생기면 동일 규칙으로 작업 필요

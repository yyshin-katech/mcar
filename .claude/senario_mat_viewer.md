---
name: senario mat 뷰어 재생성 하네스
description: senario mat 변경 시 mat_viewer HTML을 재생성하는 하네스 위치/사용법
type: reference
originSessionId: 02987e96-3bce-4338-b524-4cc3596fca14
---
## 하네스 위치
`/home/yuyeong/temp/harness_to_control_team/` (repo 외부)

- `add_manuaver.py` — senario 322개 mat에 MANUAVER 필드 추가/덮어쓰기 (curve_lane.md 기반)
- `regen_mat_viewer.py` — senario mat 데이터로 HTML 뷰어 재생성

## 대상 HTML
`src/localization/gps_system_localizer/mapfiles/senario/mat_viewer_senario_260514c 1.html`
- Line 815의 `var DATA = {...}` 1줄(GeoJSON) 만 교체
- renderDetail() 신호 섹션에 MANUAVER 행 자동 추가 (이미 있으면 skip)
- WAYPOINTS 배열 등 나머지 JS는 보존

## 좌표 변환
- mat 파일: EPSG:5179 (Korea TM, east/north)
- HTML: WGS84 lon/lat
- 이 환경은 pyproj 미설치 → `cs2cs` CLI subprocess 사용
- TM 파라미터: `+proj=tmerc +lat_0=38 +lon_0=127.5 +k=0.9996 +x_0=1000000 +y_0=2000000 +ellps=GRS80`

## 검증 체크포인트
- features 개수 (현재 322)
- signal-bearing 개수 (현재 117)
- MANUAVER -1/+1 카운트
- 샘플 LINK_ID(예: 499)의 좌표가 시흥 영역(lon~126.72, lat~37.35)인지

**Why:** mat 데이터 변경하면 뷰어 HTML도 같이 갱신해야 화면이 새 라벨(MANUAVER) 표시함. 손으로 GeoJSON 만들기엔 322 × 평균 25점이라 무리.

**How to apply:** mat 변경 후 `python3 ~/temp/harness_to_control_team/regen_mat_viewer.py` 실행 → HTML 자동 갱신.

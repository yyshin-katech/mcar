---
name: mat-stopline-adj
description: senario mat 링크를 B2_SURFACELINEMARK 정지선까지 연장 + 다음 링크 트림하는 하네스/알고리즘. mat 편집·뷰어 패치 패턴
metadata: 
  node_type: memory
  type: project
  originSessionId: c2a74392-f1b6-49d8-91ad-e78f7a6568f9
---

senario mat 도로링크 끝을 노면선표시(B2_SURFACELINEMARK) 정지선까지 연장하고 다음 링크를 교차점부터 트림. 하네스 `stopline-adj`(2026-07-15, siheung_dev). 상세 [[senario-mat-viewer]], 데이터 [[reference_a2_link_shp]].

## 1회 작업 (stop_line_adj_20260715)
링크 **871/870/877**(IID 509, 직진, stop=1) 끝을 정지선 **B2209W001421**(Kind 530=정지선, B2_SURFACELINEMARK)까지 **+2.9m 연장**(66→~68.9m, 34→36점). 다음 링크 **3871/3870/3877**(`_T` 트림링크) 앞을 잘라 교차점부터 시작(116→115점, ~229→226.3m). 정지선 수직거리 0.000000m.

## 편집 알고리즘 (per 세트 L, N=NEXT, 교차점 P, k=교차 세그 0-based index)
- **L 연장**: `new_L = L.pts + N.pts[1:k+1] + [P]` (N.start==L.end 공유라 N[0] 스킵, 끝에 P 1회).
- **N 트림**: `new_N = [P] + N.pts[k+1:]` (앞에 P 1회). → L.end==N.start==P 공유정점 유지.
- **station**: L·N 둘 다 0부터 유클리드 누적 재계산. **별도 length 필드 없음 → station[-1]이 길이.**
- **보존**(불변): LINK_ID/NEXT/LEFT/RIGHT_LINK_ID/look_at_Intersection·signalGroupID/MANUAVER/Speed_Limit/lane-change 4종/guard_zone/LINK_ID_string. **is_stop_line: 연장 L=1 유지 / 트림 N=0 유지.**

## mat 스키마 / savemat 정합
`east/north/station` = float64 **(1,N)**, 스칼라 15개 각 고정 dtype, `LINK_ID_string` = **(1,) `<U12`/`<U18`**. `scipy.io.savemat` 시 east/north/station만 `(1,-1)` reshape 대입, 나머지 원본 무손상, `__*` 키 제거. round-trip 으로 dtype/shape/키집합 백업과 동일 검증 필수.

## 좌표계 / 교차 계산
mat = **EPSG:5179** (+proj=tmerc lat_0=38 lon_0=127.5 k=0.9996 x_0=1e6 y_0=2e6 GRS80). B2_SURFACELINEMARK shp = **EPSG:32652**. 정지선을 pyproj 32652→5179 변환 후 next 링크 폴리라인과 **선분-선분 교차**로 P·k 산출. (pyproj/scipy/pyshp 이 환경 설치됨)

## HTML 뷰어 패치
`mat_viewer_senario_260514c 1.html` `var DATA`(GeoJSON 293 feature, `properties.LINK_ID` 매칭, 좌표 **[lon,lat] WGS84 7자리**, per-feature `station_start/end`·`length_m`·`point_count`, **per-point station 없음**). 편집 6 feature만 화이트리스트 필드 갱신 → `json.dumps(ensure_ascii=False)` 재직렬화(287 feature byte 동일). 5179→WGS84는 **pyproj EPSG:5179→4326** 이 기존 뷰어 좌표와 오차 0. **`regen_mat_viewer.py`(harness_to_control_team)는 repo 외부·이 환경에 없음** → 직접 패치.

## 안전/검증
mat = 주행 경로/정지선(차량 거동 직결). 원본 백업 필수(`_stopline_adj_workspace/mat_backup/` + 293 mat sha256 스냅샷). verifier 독립 재계산: 연속성·정지선 도달(<0.1m)·station 단조·위상/필드 보존·**6개 외 무변경(287 불변, git tracked 수정=6 mat+1 HTML)**·뷰어 반영.

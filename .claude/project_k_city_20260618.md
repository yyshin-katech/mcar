---
name: project-k-city-20260618
description: "K_CITY_20260618 맵 생성 이력 — shp→mat 변환 규칙, 신규 링크 구성"
metadata: 
  node_type: memory
  type: project
  originSessionId: 75e36484-068a-4b93-87f0-e653085a596c
---

K_CITY_20260618 은 K_CITY_20260608 을 복제하여 link_61 교체 + link_86/87/88 신규 추가한 맵.

**Why:** ~/shp_file/A2_LINK shp 파일(A2256W000085 등 4개 링크)을 기존 경로 구간에 삽입하여 교차로 진입 구간을 세분화.

**How to apply:** 이후 shp→mat 추가 변환 시 동일 규칙 적용.

## shp 변환 규칙

- shp 파일 좌표계: **EPSG:32652** (UTM zone 52N) → mat 저장 좌표계: **EPSG:5179** (Korean TM)
- 변환 스크립트: `mapfiles/K_CITY_20260618/convert_links.py`
- 포인트 간격: 2m (시작·종점 항상 포함)
- 라이브러리: `pyshp(shapefile)` + `pyproj Transformer`

## 링크 구성 (chain: 88→86→87→61→62)

| 파일 | shp ID | NEXT | is_stop | IID | SGID | 비고 |
|------|--------|------|---------|-----|------|------|
| link_61.mat | A2256W000085 | 62 | 0 | 0 | 0 | 기존 61 교체 |
| link_86.mat | A2256W000229 | 87 | 0 | 0 | 0 | 신규 |
| link_87.mat | A2256W000231 | 61 | 1 | 1500 | 10 | 신규, 교차로 정지선 |
| link_88.mat | A2256W000327 | 86 | 1 | 100 | 16 | 신규, 교차로 정지선 |

## 연동 파일 수정

- `launch/katech_test.launch`: MAPFILE_PATH → `K_CITY_20260618`
- `to_control_team_demo.py`: `MAX_LANE_ID = 88` (구 85)

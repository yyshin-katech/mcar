---
name: lanelet-marker-fiona-geometry
description: fiona shapefile geometry 타입별 coordinates 구조 차이 + lanelet_marker.py POINTZ 크래시/A2_LINK 전용화
metadata: 
  node_type: memory
  type: reference
  originSessionId: c2a74392-f1b6-49d8-91ad-e78f7a6568f9
---

`src/visualization/.../lanelet_marker.py` (rviz 차선 LINE_STRIP 마커) 관련 shapefile 처리 함정. (2026-05-18, siheung_dev)

## POINTZ 크래시 수정
증상: `'float' object is not subscriptable`. 원인: fiona 가 Point geometry 를 `coordinates = (x,y,z)` **단일 튜플**로 반환하는데 기존 코드가 LineString/Polygon 처럼 좌표 목록으로 가정 → `[coord[0] for coord in coords]` 에서 float 인덱싱. 해결: `geom_type in ('Point','MultiPoint')` 면 continue (LINE_STRIP 은 ≥2 점 필요하므로 어차피 못 그림). HDMap_Oido_New 의 A1_NODE/B1_SAFETYSIGN/C1_TRAFFICLIGHT 등 POINTZ 레이어가 원인.

## A2_LINK 전용화
`load_multiple_shapefiles(path, "*.shp")` → `"A2_LINK.shp"` (`lanelet_marker.py:271`). 전체 11 레이어 로드 시 마커 9226개 → RViz 성능 경고(임계 5000). 차선만 그리면 충분. 다른 레이어 시각화는 `/hmi/threejs/map`(web_hmi) 담당.

## 일반 패턴 — fiona geometry 분기
`coordinates` 구조가 타입마다 다름: Point=`(x,y,z)` 단일 튜플 / LineString=`[(x,y,z),...]` / Polygon=`[ring1, ring2, ...]`(ring 이 LineString 형태). `len(coordinates)==0` 체크만으로 Point 못 거름(튜플 len=3). **geom_type 분기 필수.**

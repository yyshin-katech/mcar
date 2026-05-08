---
name: siheung_dev 활성 맵
description: siheung_dev 브랜치의 활성 shapefile 맵 (위치/좌표계/layer 구조) — web_hmi 좌표 변환 + LAYERS_ALL 근거
type: project
originSessionId: 3ea36247-9ca8-4af1-aa8b-d69a44426fbf
---
siheung_dev 활성 맵 = `src/localization/gps_system_localizer/src/shp_map/` 루트 두 .shp:
- `TB_senario_map.shp` — POLYLINEZ (shapeType=13), 1770 features — 도로망
- `TB_senario_surfaceMARK.shp` — POLYGONZ (shapeType=15), 372 features — 노면 표시

좌표계: .prj는 `WGS_1984_UTM_Zone_52N` (EPSG:32652). .dbf에 X_5179/Y_5179 컬럼이 있으나 실 geometry는 UTM Zone 52N. web_hmi의 reproject 분기로 EPSG:32652→5179 변환되어 표시됨.

**Why:** ioniq5_hmi_dev web_hmi는 EPSG:5179(K-City 13 layer)를 가정. siheung_dev는 좌표계·layer 구조가 다 달라 그대로 쓰면 좌표 어긋남 + LAYERS_ALL 매칭 0건. 2026-05-07 web_hmi에 .prj 자동 감지(UTM_Zone_52N → EPSG:32652→5179 reproject) 추가. 2026-05-08 LAYERS_ALL을 shp_map 루트 두 layer로 확장. 1차 어댑트(senario3 sub-dir 단일 layer)는 화면에 지도가 안 떠서 폐기.

**How to apply:**
- siheung_dev에서 web_hmi 좌표 이상 신고가 오면 .prj가 UTM_Zone_52N인지 + `web_hmi_bridge.py`/`web_hmi_threejs_bridge.py` Transformer 분기가 살아있는지 확인.
- 새 .shp 추가 시 좌표계가 EPSG:5179가 아니면 reproject 분기에 WKT 키워드(예: "UTM_Zone_52N") 추가.
- **shp_map 실제 영역**: WGS84 중심 ~37.367°N, 126.725°E (시흥시 배곧신도시/정왕동), EPSG:5179 origin 약 [931391, 1929714]. 시흥 자율주행 시뮬용 맵.
- senario1/, senario3/ 서브디렉토리는 옛 단일 시나리오 맵. 활성 맵은 루트의 두 .shp.

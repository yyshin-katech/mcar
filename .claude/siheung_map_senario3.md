---
name: siheung_dev senario3 map
description: siheung_dev 브랜치의 활성 맵 senario3 좌표계·구조 (web_hmi 좌표 변환 근거)
type: project
originSessionId: 3ea36247-9ca8-4af1-aa8b-d69a44426fbf
---
siheung_dev 브랜치의 활성 맵 = `src/localization/gps_system_localizer/src/shp_map/senario3/TB_senario_map_senario3.shp` (단일 layer, POLYLINEZ, 242 features, EPSG:32652 / UTM Zone 52N).

**Why:** ioniq5_hmi_dev의 web_hmi는 EPSG:5179(K-City 13 layer)를 가정. siheung_dev senario3는 좌표계도 다르고 layer 구조도 단일이라 그대로 사용 시 좌표 어긋남 + LAYERS_ALL 매칭 0건. 2026-05-07 web_hmi에 .prj 자동 감지(UTM_Zone_52N → EPSG:32652→5179 reproject) + LAYERS_ALL을 senario3 단일 항목으로 좁힘.

**How to apply:**
- siheung_dev에서 web_hmi 좌표 이상 신고가 오면 senario3.prj가 UTM_Zone_52N 인지 + `web_hmi_bridge.py` Transformer 분기가 살아있는지 확인.
- 새 .shp 추가 시 좌표계가 EPSG:5179가 아니면 reproject 분기에 WKT 키워드(예: "UTM_Zone_52N") 추가.
- **senario3 실제 영역**: WGS84 중심 37.364°N, 126.726°E (시흥시 배곧신도시 / 정왕동 일대), 약 2km×1km. EPSG:5179 east 930440~932548, north 1929304~1930206. siheung_dev에서 사용되는 자율주행 시뮬용 맵.

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
- bag 재생 좌표 (host_east≈935713, host_north≈1916422 → 위경도 37.244°N, 126.775°E ≈ 화성 송산면 / 시화호 남단)가 senario3 영역과 매칭됨.

---
name: reference-a2-link-shp
description: web_hmi 지도/경로 원천 A2_LINK.shp(senario_shp_20260623) 구조 — 좌표계(UTM52N), 필드, 좌/중/우 3차선, ITSLinkID 1:N, 다음 링크 위상 조회
metadata:
  type: reference
---

web_hmi 지도 링크 원천: `src/localization/gps_system_localizer/src/shp_map/senario_shp_20260623/A2_LINK.shp` (HD 정밀도로지도 A2_LINK 레이어, 3014 링크). `web_hmi.launch` 의 `map_shp`/`threejs_mapdir` 기본값.

- **좌표계 함정**: `.prj` = `WGS_1984_UTM_Zone_52N` = **EPSG:32652**. shp 원좌표(예 `298797, 4136679`)를 EPSG:5179 로 착각 금지. `web_hmi_bridge`/`web_hmi_threejs_bridge` 가 로드 시 `.prj` 감지 → **EPSG:5179 로 reproject**(`Transformer.from_crs("EPSG:32652","EPSG:5179",always_xy=True)`). 프론트/route JSON 은 전부 5179 절대좌표.
- **필드**: `ID`(문자열 링크 ID, 예 `A222BF785188`) · `FromNodeID`/`ToNodeID`(위상 노드) · `R_LinkID`/`L_LinkID`(우/좌 인접 차선) · `ITSLinkID`(간선 구간 ID, **1:N** — 하나가 5~41개 세부 링크 포함, 순서 미상) · `Length` · geometry(폴리라인 점열).
- **차선 구조**: 같은 구간이 좌/중/우 **3개 병렬 차선**. R/L_LinkID 로 상호 참조. **중앙 = L_LinkID·R_LinkID 둘 다 채워진 링크**(예 `785093`[L→785092,R→785094]=중앙). 네비/경로 대표선은 중앙 차선.
- **위상 진행**: 링크A.`ToNodeID` == 링크B.`FromNodeID` 가 다음 링크. **ID 번호 순서(785188→785190)는 병렬 차선 관계일 뿐 진행 순서 아님.**
- **다음 링크 조회**: `ToNodeID` 를 `FromNodeID` 로 갖는 링크. 분기면 복수 → 폴리라인 끝/시작 접선(heading) 비교로 직진(Δ작음)/회전(Δ큼) 판정. 실측 예: `785188`.ToNode(`A122BF785074`) → `785058`(직진 Δ0.2°) / `785057`(좌분기 Δ71.8°).
- **빈 구간**: 명시 링크열이 위상 불연속이면 `FromNodeID` 인접그래프 BFS 로 중간 링크 삽입(예 `785104↔785110` 사이 = `A222BF785105` 유일).

용도: [[project_global_nav_hmi]] 경로 정의, web_hmi 지도 레이어. mat 기반 신호등/링크(`link_*.mat`)는 [[map_data]] 와 별개 — 이건 shp 원천.

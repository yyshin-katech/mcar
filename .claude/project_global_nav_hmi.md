---
name: project-global-nav-hmi
description: global-nav-hmi 하네스 — web_hmi 지도에 주행 예정 경로(중앙차선 굵은 리본) 표시 + OBU TIM 트리거 시 기존→신규 경로 latch 전환. 795117 절단(후반 ITS 미구현)
metadata:
  type: project
---

web_hmi(threejs_f1) 지도에 주행 예정 경로를 자차 앞 굵은 리본으로 표시하고, `on_block_link && do_not_go_forward` 성립 시 기존→신규(우회) 경로로 전환·latch 하는 하네스. 원천 `claude_work_list/global_nav_hmi.md`. 확정 결정 4건: 중앙차선 대표 1선 · 오프라인 JSON · 795 구간까지 · latch 전환.

**하네스**: `skills/global-nav-hmi` + agents 3 (`global-nav-{analyst,coder,verifier}`). 워크스페이스 `_global_nav_workspace/`(00_constraints→01_spec→02_impl_a/b→03_verify, git 미추적). 서브 에이전트 파이프라인.

**경로 데이터** (A2_LINK 중앙차선, [[reference_a2_link_shp]]):
- OLD(직진) 10링크 `785093→785104→785105→785110→785189→785190→785059→785065→785203→785208`, 299점.
- NEW(좌회전 우회) 11링크 = 앞 6링크(785093~785190) OLD 와 **동일 공유**, 분기 후 `785057→785082→795017→795018→795117`, 246점.
- 분기점 = 좌차선 `785188`.ToNode(`A122BF785074`): OLD `785058`(직진)/NEW `785057`(좌분기). 공유 prefix 161점(분기 index 160).
- 빈구간 `785104↔785110` = `A222BF785105`(BFS 유일, 이음매 0.00 m). 커넥터 2곳(785190→785057 3.11 m, 785082→795017 3.21 m) 외 이음매 0.00 m.
- **795117 에서 절단** — md 후반 ITSLinkID 구간(수십 링크, 순서 미상)은 미구현(다음 단계: analyst 에 ITS→ID 위상정렬 위임).

**구현** (전부 additive, CAN 무손상, git diff 48+/0-):
- 신규 `scripts/extract_route.py`(오프라인 추출: 하드코딩 링크열·위상정렬·중복점dedup·BFS옵션) → `scripts/route_senario_20260623.json`(12.9 KB, 커밋). 런타임 재추출 없음.
- `web_hmi_threejs_bridge.py`: `/hmi/threejs/route`(latch String) additive 발행(`~route_json` param). `/hmi/threejs/map` origin 발행 옆.
- 신규 `web/threejs/RouteLayer.jsx`: 지면 삼각스트립 리본(BlockZones 재질 계승, ROUTE_HALF_W=1.6). 전환 = `useRosState().on_block_link===1 && do_not_go_forward===1` 최초 성립 시 old→new latch(신규 토픽 없음, 기존 `/hmi/state` 재사용). 페이드 = `useEgoPose()` nearest-index slice 로 지나간 구간 미표시.
- `index_threejs_f1.html`/`ThreejsF1Screen.jsx` 등록·마운트.
- route JSON 좌표 = EPSG:5179 절대, 프론트가 `map.origin`(=`/hmi/threejs/map`) 빼서 delta(+X=east,+Z=north) 렌더 — MapLayers/BlockZones 규약.

전환 트리거는 [[hmi_block]] 계열(`on_block_link` LINK_ID∈{548,550,552,417,419,420}, `do_not_go_forward`=TIM `/v2x/tim_message/can_go_status` 2.0s staleness)와 공유. 실행 확인은 브라우저 육안(verifier 는 토픽까지). [[feedback_can_frozen_additive]]

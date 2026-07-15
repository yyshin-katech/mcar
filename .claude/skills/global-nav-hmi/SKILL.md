---
name: global-nav-hmi
description: web_hmi(Three.js) 지도에 차량의 주행 예정 경로를 네비게이션처럼 자차 앞 굵은 선으로 표시하고, OBU TIM(do_not_go_forward) 팝업/배너 트리거 시 기존 경로를 삭제하고 신규(우회) 경로로 전환하는 하네스. "주행 예정 경로", "글로벌 내비", "global_nav", "경로 표시", "경로 전환", "우회 경로 안내", "네비 라인", "지나간 경로 페이드", "경로 다시", "경로 하네스" 요청 시 반드시 이 스킬로 orchestrate. 원천: claude_work_list/global_nav_hmi.md. 단순 경로/링크 조회는 직접 응답.
---

# global-nav-hmi — 주행 예정 경로 표시/전환 (Orchestrator)

`claude_work_list/global_nav_hmi.md` 요구: web_hmi 지도에 주행 예정 경로(A2_LINK.shp 링크 시퀀스)를 자차 앞 굵은 선으로 표시. 차량이 특정 위치 진입 → OBU TIM 수신 → 팝업/배너(`do_not_go_forward`) 뜨면 기존 경로 삭제하고 신규 경로로 안내. 지나간 구간은 페이드/미표시.

## 확정 결정 (00_constraints.md — 재논의 금지)
- 경로 선 = **중앙 차선 대표 1선** 굵은 선 (좌/중/우 3차선 중 중앙).
- 데이터 = **오프라인 사전 추출 JSON** (빌드타임 1회).
- 범위 = 분기 직후 **795116/795118 구간까지** 우선 (후반 ITSLinkID 구간은 다음 단계).
- 전환 = **on_block_link AND do_not_go_forward** 시 old→new, 이후 **latch**.

## 트리거 정의 (불변)
- 경로 전환 = `/hmi/state.on_block_link===1 && do_not_go_forward===1` (기존 web_hmi_bridge 발행, hmi-block 하네스와 공유 — 새로 만들지 않음).
- 분기점 = 좌차선 `A222BF785188`.ToNode(`A122BF785074`) → old `A222BF785058`(직진) / new `A222BF785057`(좌분기).

## 실행 모드
**서브 에이전트 파이프라인**: analyst → coder(PART A 데이터/백엔드 → PART B 프론트) → verifier. 기존 web_hmi 하네스(hmi-block-display, tim-pedes-display)와 동일 패턴. 모든 Agent 호출에 `model: "opus"` 명시.

## Phase 0: 컨텍스트 확인
- `_global_nav_workspace/` 없음 → 초기 실행 (analyst부터).
- 있음 + 부분 수정 요청 → 해당 에이전트만 재호출.
- 있음 + 새 입력(경로 변경 등) → 기존을 `_global_nav_workspace_prev/`로 옮기고 새 실행.

## Phase 1: 분석 (global-nav-analyst)
`00_constraints.md`를 전제로 A2_LINK 위상·중앙차선 추출·빈구간 BFS·분기·전환·렌더링을 분석 → `_global_nav_workspace/01_spec.md`(route JSON 스키마 + 추출 스크립트 사양 + PART A/B 파일:라인 명세 + 검증 포인트).

## Phase 2: 구현 (global-nav-coder)
01_spec 대로 **PART A 먼저**(추출 스크립트 + web_hmi_threejs_bridge `/hmi/threejs/route` 발행 → route JSON 실제 생성), **이어서 PART B**(RouteLayer.jsx 굵은 선·전환·페이드 + index/ThreejsF1Screen 마운트). PART A의 JSON이 PART B의 입력이므로 순서 의존. 산출 `02_impl_a.md`/`02_impl_b.md`.

## Phase 3: 검증 (global-nav-verifier)
route JSON 연결성·분기·795절단, 스크립트 재현, bridge py_compile+토픽 광고, RouteLayer babel+마운트, 전환 latch 로직, CAN 무영향 → `_global_nav_workspace/03_verify.md`. FAIL 시 해당 PART 코더 1회 재호출 후 진행(누락은 보고서에 명시).

## 대상 파일
- 신규: `scripts/extract_route.py`(또는 사양 지정), `web/threejs/RouteLayer.jsx`, route JSON.
- 수정(additive): `scripts/web_hmi_threejs_bridge.py`, `web/index_threejs_f1.html`, `web/threejs_f1/ThreejsF1Screen.jsx`.

## 원칙
추가만(기존 토픽/키/콜백/레이아웃/컴포넌트 보존), CAN 무손상, PART 격리(다른 패키지 비침범), 실제 링크 ID 사용, 전환 트리거는 기존 `/hmi/state` 재사용.

## 데이터 흐름
```
extract_route.py (shp→route JSON)  ─파일→  web_hmi_threejs_bridge.py ─/hmi/threejs/route(latch)→ RouteLayer.jsx
/hmi/threejs/map(origin) ──────────────────────────────────────────────────────────────────────→ RouteLayer.jsx
/hmi/state(on_block_link,do_not_go_forward) ─전환 트리거→ RouteLayer.jsx (old→new latch)
/hmi/ego_pose ─progress→ RouteLayer.jsx (지나간 구간 페이드)
```

## 테스트 시나리오
- **정상**: 초기 old 경로 굵은 선 표시 → ego 진행에 따라 지나간 구간 페이드 → 분기점서 트리거 성립 → new 경로로 전환·latch.
- **에러**: route JSON 미생성/빈 배열 → verifier FAIL → coder PART A 재호출. RouteLayer 마운트 누락 → 화면 무표시 → verifier가 마운트 체인 지목.

## 에러 핸들링
1회 재시도 후 재실패 시 해당 결과 없이 진행(03_verify.md에 누락 명시). 상충 데이터(빈구간 연결 후보 복수 등)는 삭제하지 않고 출처 병기.

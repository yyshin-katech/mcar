---
name: hmi-block-display
description: 특정 타이밍(차량이 link 548/550/552/417 위 또는 /v2x/tim_message/can_go_status 의 do_not_go_forward=True 수신)에 web_hmi 지도와 rviz(stat_display)에 "전방 직진 주행 금지" 블로킹 표시(붉은 30% 반투명 박스 + 텍스트 + 좌/우 진행 화살표)를 띄운다. 사용자가 "hmi 블로킹", "직진 금지 표시", "hmi_block", "전방 직진 주행 금지", "블로킹 박스", "주행 경로 블로킹", "블로킹 다시" 등을 요청하면 반드시 이 스킬을 사용. 단순 코드 질문은 직접 응답.
---

# hmi-block-display — 주행 경로 HMI 블로킹 표시 (Orchestrator)

`_work_item/hmi_block.md` 요구: 차량이 지정 링크(548/550/552/417) 위에 있거나 V2X can_go_status
의 `do_not_go_forward=True` 수신 시, web_hmi 지도 + rviz 화면에 "전방 직진 주행 금지" 표시.

## 실행 모드
**에이전트 팀(파이프라인)**: analyst → coder(PART1 web_hmi / PART2 stat_display 병렬) → verifier.
다른 하네스(spat-viewer-build, vehicle-tracker)와 동일 패턴.

## 산출물 위치
`_hmi_block_workspace/` : `01_spec.md`(사양) / `02_impl_*.md`(구현) / `03_verify.md`(검증).

## 트리거 정의 (불변)
- `on_block_link` = LINK_ID ∈ {548, 550, 552, 417}  (출처: /hmi/state.link_id / local_msg.LINK_ID, 실제 링크 ID)
- `do_not_go_forward` = /v2x/tim_message/can_go_status True 수신 후 2.0s staleness 내
- 붉은 반투명 박스(opacity 0.3) + 좌/우 화살표 = `on_block_link`
- "전방 직진 주행 금지" 텍스트/팝업 = `on_block_link OR do_not_go_forward`

## 블로킹 박스 좌표 (EPSG:5179)
Box A: (931706.402,1928787.276)(931709.078,1928785.213)(931703.929,1928789.172)
Box B: (931741.676,1928832.478)(931739.247,1928834.373)(931744.369,1928830.448)
(원천 WGS84 lat/long 1e7 7점 중 region7=region6 중복 제거 → 2 폴리곤)

## 대상
- web_hmi: `scripts/web_hmi_bridge.py`, `web/threejs/BlockZones.jsx`(신규), `web/index_threejs_f1.html`
- stat_display: `include/stat_display.h`, `lib/stat_display.cpp`, `package.xml`/`CMakeLists.txt`, `workspace_config/ioniq_statdisplay.rviz`

## 원칙
추가만(기존 토픽/키/콜백/팝업/레이아웃 보존), PART 격리(다른 패키지 비침범), 실제 링크 ID 사용.

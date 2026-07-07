---
name: tim-pedes-display
description: TIM(V2X) 보행자(/obu/v2x_pedes_assistance) 와 차량 자체 횡단보도 판단(/katech_msg/crosswalk_detection) 을 퓨전(소스 필드 포함)하여 새 토픽으로 발행하고, web_hmi 지도의 횡단보도 #1/#2 위에 보행자 유무를 붉은 점멸 + "전방 보행자 주의" 팝업(소스별 색상)으로 표시한다. 사용자가 "tim pedes", "보행자 퓨전", "횡단보도 보행자 표시", "tim_pedes_display", "보행자 퓨전 다시", "횡단보도 hmi" 등을 요청하면 반드시 이 스킬을 사용. 단순 코드 질문은 직접 응답.
---

# tim-pedes-display — TIM 보행자 퓨전 + HMI 표시 (Orchestrator)

원천 요구: `claude_work_list/tim_pedes_display.md`.
차량 자체 횡단보도 보행자 판단 + OBU(V2X) 보행자 신호를 퓨전 → web_hmi 지도에 횡단보도별 보행자 표시.

## 실행 모드
**에이전트 팀(파이프라인)**: analyst → coder(PART A 백엔드 / PART B web_hmi) → verifier.
(현 세션에서 신규 에이전트 타입이 아직 로드 안 됐으면 general-purpose 로 프롬프트에 역할·workspace 문서를 실어 실행.)

## 산출물 위치
`_tim_pedes_workspace/` : `00_constraints.md`(확정 제약·계약) / `01_spec.md`(사양) / `02_impl_*.md`(구현) / `03_verify.md`(검증).

## 확정 결정 (불변 — 00_constraints.md)
- 횡단보도 ↔ OBU pedes: **#1=south_pedes, #2=east_pedes**.
- 아키텍처: **새 퓨전 노드(CAN 무손상)** — 검출 노드는 additive `crosswalk_id` 만.
- 팝업 색상: 자체=주황 #ff9800 / OBU=빨강 #ff3030 / 둘다=자홍 #ff30ff.

## 트리거/매핑 (불변)
- active_crosswalk: LINK_ID ∈ {1239,1238}→#1, ==1205→#2, else none. (출처 `/localization/to_control_team`)
- own_present: 자체검출 중 `crosswalk_id==active && on_crosswalk==1`.
- obu_present: #1→south_pedes, #2→east_pedes.
- present = own OR obu. source = own&obu:3 / obu:2 / own:1 / none:0.
- "link 1205 & #2 없고 #1만 있으면 무시" = active=#2 일 때 #2 필드만 평가(자연 충족).

## 네이밍 계약
- msg: `ped_crosswalk_check_msg += uint8 crosswalk_id`, 신규 `crosswalk_ped_fusion_msg`(time/active_crosswalk_id/pedestrian_present/source).
- 노드: `crosswalk_ped_fusion.py`(pkg can) → `/katech_msg/crosswalk_ped_fusion`. `launch/katech_test.launch` 등록.
- web_hmi: `web/threejs/CrosswalkZones.jsx`(신규) + `web_hmi_bridge.py` state 키 `crosswalk_ped_active/present/source` + threejs HTML 진입점 마운트.

## 원칙
추가만(기존 토픽/키/콜백/CAN 전송 보존), PART 격리(다른 패키지 비침범), 좌표 EPSG:5179 단일 원천(검출 노드 다각형), 값 추측 금지.

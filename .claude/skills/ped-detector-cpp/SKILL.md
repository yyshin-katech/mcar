---
name: ped-detector-cpp
description: katech_ped_detector.py(횡단보도 보행자 on_crosswalk 검출)를 C++ ROS 노드로 포팅하고 크기 게이트 + 방향 게이트(횡단축 vs 절대 이동방향)를 추가하는 하네스. 순서 타입→크기→멤버십→방향. "ped detector cpp", "보행자 검출 cpp 포팅", "크기 게이트", "방향 게이트", "on_crosswalk cpp", "횡단보도 검출 cpp" 요청 시 이 스킬로 orchestrate. 확정사실: _ped_detector_cpp_workspace/00_constraints.md. 단순 조회는 직접 응답.
---

# ped-detector-cpp — 보행자 검출 C++ 포팅 + 크기·방향 게이트 (Orchestrator)

`katech_ped_detector.py` 의 `on_crosswalk` 검출을 C++ 노드로 포팅 + 크기/방향 게이트 추가. 순서 **타입→크기→멤버십(LINK)→방향**. 원천: `_ped_detector_cpp_workspace/00_constraints.md`.

## 실행 모드
**에이전트 팀(파이프라인)**: analyst → coder → verifier. `Agent` 직접 호출, `model: opus`. 산출 `_ped_detector_cpp_workspace/`.
※ ped-detector-cpp-* 에이전트가 레지스트리에 없으면 `general-purpose` 로 실행하되 각 .md 역할·범위를 프롬프트에 주입.

## Phase 0: 컨텍스트
- workspace 존재 + 부분 재작업 → 해당 에이전트만 재호출.
- 새 요청 → 오케스트레이터가 사전조사(현행 Python 로직/토픽/msg, 속도 프레임, crosswalk_data/CW_LINKS) + **사용자 결정 확인**(정지 처리, 임계값) 후 analyst.

## Phase 1: 분석 (analyst) → `01_spec.md`
포팅 등가 로직 + 신규 게이트(크기·방향) + 절대속도(ego 가산) + PCA 축 + C++ 필드경로 + CMake/launch 명세.

## Phase 2: 구현 (coder) → 신규 C++ 노드 + CMake + launch 교체 + `catkin_make` → `02_impl.md`.

## Phase 3: 검증 (verifier) → 빌드·토픽등가·값일치·게이트·절대속도·정지처리·무변경·dual-pub 없음 → `03_verify.md`. FAIL 시 coder 재호출.

## 확정 불변 (재논의 금지)
- **동작 등가**: 토픽/메시지/crosswalk_data(1~9)/CW_LINKS/ray-casting/LINK 게이팅/occupancy(occupied_ids+crosswalk1/2) 보존. 신규 = size·direction 게이트만.
- **객체 vx,vy = ego-상대** → 절대속도 `R(yaw)·(vx,vy)+v_ego_map` (ego 가산 필수).
- 순서 타입{2,3}→크기≤2.0m→멤버십→방향(이동 sp≥0.3m/s & 각≤40°, 정지 스킵=타입+크기 유지). 상수 튜닝가능.
- CAN writer(katech_ped_detector_can_writer.cpp)·fusion(crosswalk_ped_fusion.py)·msg 정의 불침범. launch dual-publisher 금지(Python↔C++ 택1).

## 에러 핸들링
verifier FAIL → 원인 분류 후 coder 1회 재호출. 재실패 시 사용자 보고.

## 테스트 시나리오
- 정상: 빌드 EXIT0, 토픽/값 등가, 게이트 순서·절대속도·정지처리 확인.
- 엣지: 링크셋 disjoint, PCA 축 9개, dual-publisher 부재, 정지 보행자 유지·정차 차량 배제.

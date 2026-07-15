---
name: spat-dir-match
description: MQTT SPaT 신호를 ego 진행방향(local MANUAVER -1/0/1)에 맞는 movement(MovementStateName LEFT/STR/RIGHT)로 매칭하도록 수정/반영하는 하네스. "spat 방향 매칭", "신호등 방향 매칭", "MANUAVER 매칭", "MovementStateName 매칭", "좌회전 신호 매칭 안됨", "방향 신호등 잘못 나옴", "spat 방향 다시" 요청 시 반드시 이 스킬로 orchestrate. 원천 확정사실: _spat_dir_workspace/00_constraints.md. 단순 조회는 직접 응답.
---

# spat-dir-match — SPaT 방향 매칭 (Orchestrator)

MQTT SPaT movement 방향(문자열 STR/LEFT/RIGHT)을 ego 의 local MANUAVER(-1/0/1)에 매칭해, **ego 진행방향에
해당하는 신호등만** CAN/HMI 에 반영. 원천: `_spat_dir_workspace/00_constraints.md`.

## 실행 모드
**에이전트 팀(파이프라인)**: analyst → coder(PART A CAN / PART B HMI, 지정 PART만) → verifier.
서브 에이전트 `Agent` 직접 호출, `model: opus`. 산출물은 파일 기반(`_spat_dir_workspace/`).

## Phase 0: 컨텍스트 확인
- `_spat_dir_workspace/` 존재 + 부분 수정 요청 → 해당 에이전트만 재호출.
- 새 요청 → analyst 부터. `00_constraints.md` 없으면 오케스트레이터가 먼저 사전조사(코드+bag)로 작성.

## Phase 1: 분석 (spat-dir-analyst)
`00_constraints.md` 전제 → 소비자별 신호 선택 로직 규명 → `01_spec.md`(진단+PART별 수정계획+엣지+검증계획).
**CAN 수정이 포함되면 coder 진행 전 오케스트레이터가 사용자에게 스코프·진단 확인.**

## Phase 2: 구현 (spat-dir-coder)
`01_spec.md` 대로 PART 별 외과적 구현. CAN 무손상(선택 로직만). 빌드/문법 확인. → `02_impl_<part>.md`.

## Phase 3: 검증 (spat-dir-verifier)
빌드 + 정적 + CAN diff 무손상 + bag(2026-06-24) 드라이런으로 (302/70/LEFT) 방향 매칭 확인 → `03_verify.md`.
FAIL 시 원인 분류해 coder(PART)/analyst 재호출.

## 데이터 전달
파일 기반: `00_constraints.md`(입력) → `01_spec.md` → `02_impl_*.md` → `03_verify.md`.
최종 요약만 사용자에게. 중간 산출물 보존.

## 확정 불변 (재논의 금지)
- 방향 매핑 **-1→LEFT / 0→STR / 1→RIGHT**.
- PED/BUS/BYC 차량 방향 매칭 제외.
- signalGroup 단독 매칭 금지(같은 SG 다방향) — 방향 문자열이 최종 판별자.
- **CAN 무손상**: spat_CAN_writer 는 선택 로직만, 프레임/시그널/ID/주기 불변.

## 에러 핸들링
1회 재시도 후 재실패면 그 PART 없이 진행하고 보고서에 누락 명시. 상충 데이터는 삭제 말고 출처 병기.

## 테스트 시나리오
- 정상: (302, SG70, MANUAVER-1) → MQTT (302,70,LEFT) 선택, STR/PED 아님.
- 엣지: 매칭 없음(517 LEFT 부재) → 안전 동작(신호 미표시/0), CAN 프레임 불변.

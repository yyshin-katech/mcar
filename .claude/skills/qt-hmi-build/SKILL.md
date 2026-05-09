---
name: qt-hmi-build
description: Qt6/C++ 네이티브 HMI를 web_hmi와 동등한 기능 (F1 dashboard + 3D scene + ControlPanel)으로 신규 개발/확장. 사용자가 "qt hmi 개발", "네이티브 hmi", "qt hmi 다음 단계", "qt hmi 마일스톤", "qt hmi 빌드", "qt hmi 검증" 등을 요청하면 반드시 이 스킬을 사용. 작업물은 `claude_work_list/hmi_design/` 아래.
---

# qt-hmi-build 오케스트레이터

새 Qt6/C++ HMI 패키지(`src/visualization/qt_hmi/`)를 web_hmi 기능 동등 수준으로 점진 개발. 3단계 파이프라인 (설계 → 구현 → 검증). web-hmi-adapt와 동일한 디자인 패턴.

## 실행 모드

**파이프라인 패턴 (서브 에이전트 순차)** — 각 단계는 직전 산출물을 입력으로 사용. 병렬화 불가.

## 작업 디렉토리

- 산출물 루트: 워크스페이스 `claude_work_list/hmi_design/`
- 신규 코드: 워크스페이스 `src/visualization/qt_hmi/`
- 워크스페이스 루트는 환경마다 다름 (`/home/sim/mcar/`, `/home/ads/mcar_v13/`, `/home/katech/mcar_v13/`). 오케스트레이터는 `git rev-parse --show-toplevel`으로 검출 후 프롬프트에서 절대 경로로 변환.

## Phase 0: 컨텍스트 확인

호출 즉시:

1. `git rev-parse --show-toplevel`로 워크스페이스 루트 결정.
2. `claude_work_list/hmi_design/01_design.md` 존재 여부 확인.
3. 마일스톤 진행 상태 확인 (`02_impl_M*.md`로 마일스톤별 분리 저장).
4. 사용자 의도 분기:
   - **초기 실행** (설계 문서 없음) → Phase A부터.
   - **다음 마일스톤** ("M2 진행") → Phase A skip, Phase B를 다음 M으로 호출.
   - **재검증** ("다시 검증") → Phase C만 호출.
   - **설계 갱신** ("설계 다시") → Phase A 재호출, 기존 설계는 `01_design_prev.md`로 백업.

## Phase A: design-architect

`Agent` 도구로 1회 호출. `subagent_type='general-purpose'`, `model: 'opus'`.

prompt 필수 항목:
- "먼저 `${WORKSPACE}/.claude/agents/design-architect.md`를 Read하고 그 정의를 따라 작업하라."
- 기존 분석 대상: `src/visualization/web_hmi/` (기능 사양 원본), `src/visualization/pyqt_hmi/` (ROS↔Qt 패턴 참고).
- 출력: `claude_work_list/hmi_design/01_design.md`
- 마일스톤 분할 명시 — M1: 스켈레톤+ROS bridge+빌드, M2: F1 telemetry, M3: 3D map scene, M4: ControlPanel.
- 코드 변경 금지 (분석/설계만).

**대기**: design-architect 완료 보고 대기.

## Phase B: impl-coder

design 문서 Read 후 `Agent` 1회 호출. `subagent_type='general-purpose'`, `model: 'opus'`.

prompt:
- "먼저 `${WORKSPACE}/.claude/agents/impl-coder.md`를 Read하고 그 정의를 따라 작업하라."
- 입력: `claude_work_list/hmi_design/01_design.md`
- 처리할 마일스톤: 호출 시점에 명시 (1차 호출은 M1).
- 출력 코드: `src/visualization/qt_hmi/` (신규 패키지)
- 출력 보고: `claude_work_list/hmi_design/02_impl_M{n}.md`
- 외과적 구현 — 마일스톤 외 코드 작성 금지. 다른 마일스톤은 인터페이스만 정의(stub) 또는 TODO.

**대기**: impl-coder 완료 보고 대기.

## Phase C: impl-verifier

`Agent` 1회 호출. `subagent_type='general-purpose'`, `model: 'opus'`.

prompt:
- "먼저 `${WORKSPACE}/.claude/agents/impl-verifier.md`를 Read하고 그 정의를 따라 작업하라."
- 입력: 직전 마일스톤 산출물 (`02_impl_M{n}.md` + 변경된 파일들)
- 출력: `claude_work_list/hmi_design/03_verify_M{n}.md`
- 검증 항목:
  - `catkin_make --pkg qt_hmi` 결과 (빌드 PASS 필수)
  - 헤더 include 정합성, Qt6 모듈 누락 여부
  - ROS 토픽 구독 시그니처 매칭 (.msg 정의와)
  - 라이브 가능 시 `rostopic info` / `rostopic echo` 로 위젯-토픽 흐름 확인

## Phase D: 통합 보고

3개 산출물 Read 후 사용자에게 직접 표시:

```markdown
## qt_hmi 마일스톤 M{n} 결과 요약

### 설계 → 구현 → 검증

| 단계 | 결과 | 산출물 |

### 구현된 기능
- ...

### 빌드/검증 PASS / FAIL
- ...

### 다음 마일스톤 제안
- M{n+1}: ...
```

`claude_work_list/hmi_design/00_consolidated.md`에 동시 저장 (마일스톤 누적).

## Phase E: 후속 처리

- impl-coder가 "설계 모호" 보고 → 사용자에게 추가 지시 요청 또는 Phase A 재호출.
- impl-verifier FAIL → 1회 impl-coder 재호출 (수정만), 그래도 FAIL이면 사용자 개입.
- M{n} PASS → "M{n+1} 진행할까요?" 사용자 확인 후 다음 단계.
- 모든 마일스톤 PASS → 커밋 여부 사용자 확인.

## 데이터 흐름 / 에러 핸들링

- 단계 간 데이터: `claude_work_list/hmi_design/` 파일 기반.
- design-architect 실패 → 환경 문제(파일 못 찾음 등) 보고 후 중단.
- impl-coder 실패 → 신규 파일 롤백(`git clean -fd src/visualization/qt_hmi/`은 사용자 확인 후), 설계 재검토.
- impl-verifier FAIL → 빌드 에러 분류(헤더 누락/심볼 미정의/CMake 등) 후 1회 impl-coder 재호출.

## 테스트 시나리오

### 정상 흐름 (1차)
1. 사용자 "qt hmi 만들어줘".
2. `claude_work_list/hmi_design/` 비어있음 → Phase A부터.
3. design-architect → impl-coder(M1) → impl-verifier(M1) 순차 실행.
4. M1 PASS → "M2 진행할까요?" 확인.

### 정상 흐름 (M2 이상)
1. 사용자 "qt hmi M2 진행".
2. `01_design.md` 존재 → Phase A skip.
3. impl-coder(M2) → impl-verifier(M2) 실행.

### 재검증
1. 사용자 "qt hmi 다시 검증".
2. impl-verifier만 재호출 (현재 마일스톤 기준).

### 에러 흐름
1. impl-verifier FAIL → 빌드 로그 첨부해 impl-coder 1회 재호출.
2. 재시도도 FAIL이면 사용자에게 로그 제시 + 추가 지시 요청.

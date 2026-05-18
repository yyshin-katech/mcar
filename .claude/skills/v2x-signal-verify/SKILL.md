---
name: v2x-signal-verify
description: senario 경로 mat 파일을 사용할 때 v2x SPaT(신호등) 정보가 디코딩→매칭→to_control_team 발행까지 제대로 흘러가는지 정합성 검증. 사용자가 "v2x 신호등 검증", "senario 신호등 매칭 확인", "SPaT 파싱 검증", "신호등 디코딩 흐름 점검" 등을 요청하면 반드시 이 스킬을 사용. 코드 변경 없음(정적 분석 + 정적 검증). 패치가 필요하면 별도 하네스로 위임 권장.
---

# v2x-signal-verify 오케스트레이터

목적: `mapfiles/senario/*.mat` 의 신호등 매핑 ↔ `siheung_v2x` 의 SPaT 디코딩(`/katri_v2x_node/katri_spat` 등) ↔ `to_control_team_demo.py` 의 신호등 매칭 로직 사이 정합성을 코드 변경 없이 검증.

라이브 ROS 토픽 echo 는 SKIP (사용자 정책). 정적 분석 위주.

## 실행 모드

2단계 파이프라인 (analyst → verifier). 병렬 불가 (verifier 가 analyst 산출물을 입력으로 받음).

## Phase 0: 컨텍스트 확인

호출 즉시:

1. `_v2x_verify_workspace/` 존재 여부 확인.
2. 존재하면:
   - 사용자가 부분 재실행 ("분석만 다시", "검증만") → 해당 단계만 재호출.
   - 사용자가 새 검증 → 기존 `_v2x_verify_workspace/` 를 `_v2x_verify_workspace_prev/` 로 이동.
3. 없으면 초기 실행.
4. 사용자가 맵 경로/시나리오를 명시했으면 (예: `senario`, `senario3`) analyst 에게 전달.

`_v2x_verify_workspace/` 는 워크스페이스 루트 (`/home/yuyeong/mcar/`) 하위.

## Phase A: v2x-signal-analyst

`Agent` 1회 호출. `subagent_type='general-purpose'`, `model: 'opus'`.

prompt 필수 항목:
- "먼저 `/home/yuyeong/mcar/.claude/agents/v2x-signal-analyst.md` 를 Read 하고 그 정의를 따라 작업하라."
- 출력: `_v2x_verify_workspace/01_analysis_report.md`
- 검증 대상 시나리오 디렉토리 (기본: `src/localization/gps_system_localizer/mapfiles/senario/`)
- 사용자가 명시한 한정 범위 (예: "특정 link만") 가 있으면 전달.

대기: analyst 완료 보고 대기. 보고 없이 다음 단계 진행 금지.

## Phase B: v2x-signal-verifier

analyst 보고서 Read 후 `Agent` 1회 호출. `subagent_type='general-purpose'`, `model: 'opus'`.

prompt:
- "먼저 `/home/yuyeong/mcar/.claude/agents/v2x-signal-verifier.md` 를 Read 하고 그 정의를 따라 작업하라."
- 입력: `_v2x_verify_workspace/01_analysis_report.md` 의 "검증 체크리스트" 섹션
- 출력: `_v2x_verify_workspace/02_verify_report.md`
- 정적 검증만 (라이브 토픽 echo SKIP — 사용자 정책)

대기: verifier 완료 보고 대기.

## Phase C: 통합 보고

두 보고서를 Read 후 사용자에게 직접 표시:

```markdown
## v2x 신호등 파싱 검증 결과 요약

### 분석 항목 N건 → 검증 결과 (PASS/WARN/FAIL)

| 분류 | 항목 | 결과 | 비고 |

### 발견 정합성 이슈
- (FAIL/WARN 만)

### 다음 단계 제안
- (라이브 검증으로 더 확인할 항목, 패치가 필요하면 후속 하네스 호출 권장)
```

`_v2x_verify_workspace/00_consolidated.md` 로 동시 저장.

## Phase D: 후속 처리

- analyst 가 "사양 모호" 보고 → 사용자에게 추가 지시 요청.
- verifier 가 FAIL 보고 → analyst 1회 재호출 (무한 루프 방지 위해 1회 한정).
- 모두 PASS → 추가 라이브 검증이 필요한지 사용자에게 제안만, 코드 변경 금지.
- 패치가 필요하면 별도 하네스로 위임 권장 (사용자 동의 후).

## 데이터 흐름 / 에러 핸들링

- 단계 간 데이터: `_v2x_verify_workspace/` 파일 기반.
- analyst 실패 → 환경 문제 (예: mat 파일 파싱 불가) 보고 후 중단.
- verifier 실패 → analyst 1회 재호출, 그래도 실패면 사용자 개입.

## 작업 원칙

- **코드 변경 금지** (정적 분석/검증만).
- mat 파일은 SciPy `scipy.io.loadmat` 으로 읽되, 가용하지 않으면 `octave-cli` / `h5py` 폴백 또는 사양만 기록.
- 라이브 ROS 토픽 검증은 사용자가 별도 요청하지 않으면 SKIP.
- 보고서는 갭/이슈가 0건이어도 그 결과를 명시. 무리한 트집 금지.

## 테스트 시나리오

### 정상 흐름
1. 사용자 "senario 신호등 매칭 검증해줘".
2. `_v2x_verify_workspace/` 없음 → 초기 실행.
3. analyst → verifier 순차 실행.
4. 모두 PASS → 라이브 검증 권장만 제안.

### 부분 재실행
1. 1차 검증 후 "mat 파일 갱신했으니 다시 분석" 요청.
2. `_v2x_verify_workspace/` 보존, analyst 만 재호출.
3. 새 이슈 있으면 verifier 이어 실행.

### FAIL 흐름
1. analyst 가 신호등 mat 누락/typo 발견.
2. verifier 가 정적 검증에서 FAIL 보고.
3. 오케스트레이터가 사용자에게 패치 방향을 보고하고 별도 하네스/직접 수정 동의 요청.

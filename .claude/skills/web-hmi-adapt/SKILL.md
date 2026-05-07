---
name: web-hmi-adapt
description: web_hmi가 ioniq5_hmi_dev 브랜치 기준으로 작성되어 현재 브랜치(siheung_dev 등)에서 동작 안 할 때, 토픽·메시지·launch·맵 차이를 매칭해 어댑팅. 사용자가 "web_hmi 데이터 매칭", "web_hmi 어댑트", "브랜치 매칭", "web_hmi가 안 도는데", "다시 어댑트", "어댑트 갱신" 등을 요청하면 반드시 이 스킬을 사용. 맵 경로 변경(senario3 등) 요청도 포함.
---

# web-hmi-adapt 오케스트레이터

`web_hmi`(`scripts/`, `launch/`, `web/`)를 현재 브랜치의 데이터 구조에 맞춰 어댑팅. 3단계 파이프라인 (분석 → 적용 → 검증).

## 실행 모드

**파이프라인 패턴 (서브 에이전트 순차)** — 각 단계는 직전 산출물을 입력으로 사용. 병렬화 불가능 (의존 관계).

## Phase 0: 컨텍스트 확인

호출 즉시:

1. `_adapt_workspace/` 존재 여부 확인.
2. 존재하면:
   - 사용자가 **부분 재실행** ("매칭만 다시", "검증만") → 해당 단계만 재호출.
   - 사용자가 **새 어댑트** → 기존 `_adapt_workspace/`를 `_adapt_workspace_prev/`로 이동.
3. 없으면 **초기 실행**.
4. 사용자가 맵 경로를 명시했으면(`senario3` 등) Phase A의 detective에게 전달.

`_adapt_workspace/`는 워크스페이스 루트(`/home/katech/mcar_v13/`) 하위.

## Phase A: match-detective

`Agent` 도구로 1회 호출. `subagent_type='general-purpose'`, `model: 'opus'`.

prompt 필수 항목:
- "먼저 `/home/katech/mcar_v13/.claude/agents/match-detective.md`를 Read하고 그 정의를 따라 작업하라."
- 출력 파일: `_adapt_workspace/01_match_report.md`
- 사용자가 명시한 맵 경로 (예: `localization/gps_system_localizer/src/shp_map/senario3`)
- 현재 브랜치명 (e.g. siheung_dev)
- "발견 0건이면 그렇게 보고. 무리한 트집 금지."

**대기**: detective 완료 보고 대기. 보고 없이 다음 단계 진행 금지.

## Phase B: bridge-adapter

detective 보고서 Read 후 `Agent` 1회 호출. `subagent_type='general-purpose'`, `model: 'opus'`.

prompt:
- "먼저 `/home/katech/mcar_v13/.claude/agents/bridge-adapter.md`를 Read하고 그 정의를 따라 작업하라."
- 입력: `_adapt_workspace/01_match_report.md`의 "권장 패치 사양" 섹션
- 출력: `_adapt_workspace/02_adapter_changes.md`
- 외과적 변경 원칙 강조: 사양 외 변경 금지

**대기**: adapter 완료 보고 대기.

## Phase C: adapt-verifier

`Agent` 1회 호출. `subagent_type='general-purpose'`, `model: 'opus'`.

prompt:
- "먼저 `/home/katech/mcar_v13/.claude/agents/adapt-verifier.md`를 Read하고 그 정의를 따라 작업하라."
- 입력: bridge-adapter가 변경한 파일들 (02_adapter_changes.md 참조)
- 출력: `_adapt_workspace/03_verify_report.md`

## Phase D: 통합 보고

3개 보고서 모두 Read 후 사용자에게 직접 표시:

```markdown
## web_hmi 어댑트 결과 요약

### 분석된 갭 N건 → 적용 패치 M건 → 검증 결과

| 단계 | 결과 | 산출물 |

### 변경 파일
| 파일 | 라인 | 변경 요약 |

### 검증 PASS / FAIL
- ...

### 다음 단계 제안
- 라이브 실행 시 추가 확인 필요한 토픽/페이로드
```

`_adapt_workspace/00_consolidated.md` 로 동시 저장.

## Phase E: 후속 처리

- adapter가 "사양 모호" 보고하면 사용자에게 추가 지시 요청.
- verifier가 FAIL이면 detective에게 재분석 요청 (1회만, 무한 루프 방지).
- 모두 PASS면 커밋 여부 사용자 확인.

## 데이터 흐름 / 에러 핸들링

- 단계 간 데이터: `_adapt_workspace/` 파일 기반.
- detective 실패 → 환경 문제(.msg 못 찾음 등) 보고 후 중단.
- adapter 실패 → 변경 롤백(git checkout -- ...) 후 사양 재검토.
- verifier FAIL → detective 1회 재호출, 그래도 FAIL이면 사용자 개입.

## 테스트 시나리오

### 정상 흐름
1. 사용자 "web_hmi 데이터 매칭 해줘 (맵: senario3)".
2. `_adapt_workspace/` 없음 → 초기 실행.
3. detective → adapter → verifier 순차 실행.
4. 모두 PASS → 커밋 제안.

### 부분 재실행
1. 1차 어댑트 후 ".msg 새로 추가됐는데 다시 매칭" 요청.
2. `_adapt_workspace/` 보존, detective만 재호출.
3. 새 갭이 있으면 adapter → verifier 이어 실행.

### 에러 흐름
1. detective가 권장 패치 작성하지만 adapter가 "사양 모호" 보고.
2. 오케스트레이터가 사용자에게 추가 지시 요청.
3. 사용자 추가 지시로 adapter 1회 재호출.

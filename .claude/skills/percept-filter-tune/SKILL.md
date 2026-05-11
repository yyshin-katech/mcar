---
name: percept-filter-tune
description: src/sensing/can/src/percept_topic_matcher.cpp 의 perception 오브젝트 필터링 정책을 튜닝. 자차 기준 우선순위 (전방 100m → 좌/우 → 14개 cap 유지) 를 외과적으로 적용/재적용한다. 사용자가 "perception 필터 튜닝", "오브젝트 필터", "percept_topic_matcher 수정", "전방 100m 우선", "오브젝트 14개 유지", "object_filter.md 작업" 등을 요청하면 반드시 이 스킬을 사용. 단순 코드 질문은 직접 응답.
---

# percept-filter-tune 오케스트레이터

`percept_topic_matcher.cpp` 의 오브젝트 필터 정책을 사용자 사양(`claude_work_list/object_filter.md` 등)대로 조정한다. 3 단계 파이프라인 (분석 → 구현 → 검증).

## 실행 모드

**파이프라인 패턴 (서브 에이전트 순차)** — `Agent` 도구로 1회씩 호출. 각 단계는 직전 산출물(`_filter_workspace/*.md`)을 입력으로 사용. 병렬화 불가능. `model: "opus"` 명시.

## Phase 0: 컨텍스트 확인

호출 즉시:

1. `_filter_workspace/` 존재 여부 확인.
2. 존재하면:
   - 사용자가 **부분 재실행** ("분석만 다시", "검증만", "다시 빌드") → 해당 단계 에이전트만 재호출.
   - 사용자가 **새 튜닝** (사양 변경, 다른 cpp 파일 지정) → 기존 `_filter_workspace/` 를 `_filter_workspace_prev/` 로 이동.
3. 없으면 **초기 실행**.
4. 사용자가 사양 파일 경로(`claude_work_list/object_filter.md` 등)를 명시했으면 그 내용 본문을 analyst 에게 그대로 전달.

`_filter_workspace/` 는 워크스페이스 루트(`/home/ads/mcar_v13/`) 하위.

## Phase A: percept-filter-analyst

`Agent` 1회 호출. `subagent_type='percept-filter-analyst'` (정의 파일이 있으면) 또는 `subagent_type='general-purpose'`. `model: 'opus'`.

prompt 필수 항목:
- "먼저 `/home/ads/mcar_v13/.claude/agents/percept-filter-analyst.md` 를 Read 하고 그 정의를 따라 작업하라."
- 출력 파일: `_filter_workspace/01_filter_report.md`
- 사용자 사양 본문 (object_filter.md 등) 인용
- 대상 cpp 후보: `src/sensing/can/src/percept_topic_matcher.cpp` (메인), `src/sensing/can/src/percept_topic_only_front_lidar.cpp` (보조)
- "발견 0건이면 그렇게 보고. 무리한 트집 금지."

**대기**: analyst 완료 보고 대기. 보고 없이 다음 단계 진행 금지.

## Phase B: percept-filter-coder

analyst 보고서 Read 후 `Agent` 1회 호출. `subagent_type='percept-filter-coder'` 또는 `'general-purpose'`. `model: 'opus'`.

prompt:
- "먼저 `/home/ads/mcar_v13/.claude/agents/percept-filter-coder.md` 를 Read 하고 그 정의를 따라 작업하라."
- 입력: `_filter_workspace/01_filter_report.md` 의 "권장 패치 사양" 섹션
- 출력: `_filter_workspace/02_coder_changes.md`
- 외과적 변경 원칙 강조: 사양 외 변경 금지
- 빌드 명령: `cd /home/ads/mcar_v13 && catkin_make --pkg can`

**대기**: coder 완료 보고 대기. 빌드 PASS 까지 확인.

## Phase C: percept-filter-verifier

`Agent` 1회 호출. `subagent_type='percept-filter-verifier'` 또는 `'general-purpose'`. `model: 'opus'`.

prompt:
- "먼저 `/home/ads/mcar_v13/.claude/agents/percept-filter-verifier.md` 를 Read 하고 그 정의를 따라 작업하라."
- 입력: coder 가 변경한 파일들(`02_coder_changes.md` 참조) + 사양서(`01_filter_report.md`)
- 출력: `_filter_workspace/03_verify_report.md`

## Phase D: 통합 보고

3 개 보고서 모두 Read 후 사용자에게 직접 표시:

```markdown
## perception 필터 튜닝 결과 요약

### 분석 → 구현 → 검증
| 단계 | 결과 | 산출물 |

### 변경 파일
| 파일 | 라인 | 변경 요약 |

### 검증 PASS / FAIL
- 빌드:
- 출력 객체 수 == 14:
- 전방 100m 우선:

### 다음 단계 제안
- 라이브 실행 시 추가 확인 필요한 토픽/페이로드
```

`_filter_workspace/00_consolidated.md` 로 동시 저장.

## Phase E: 후속 처리

- coder 가 "사양 모호" 보고 → 사용자에게 추가 지시 요청.
- verifier FAIL (빌드/사양 불충족) → coder 1 회만 재호출 (무한 루프 방지). 그래도 FAIL 이면 사용자 개입.
- verifier FAIL (사양 자체 문제) → analyst 1 회 재호출 후 coder 재실행.
- 모두 PASS 면 커밋 여부 사용자 확인.

## 데이터 흐름 / 에러 핸들링

- 단계 간 데이터: `_filter_workspace/` 파일 기반.
- analyst 실패 (cpp 못 찾음, msg 정의 없음) → 환경 문제 보고 후 중단.
- coder 실패 (빌드 깨짐) → 변경 롤백(`git checkout -- <file>`) 후 사양 재검토.
- verifier FAIL → 위 분류대로 분기.

## 테스트 시나리오

### 정상 흐름
1. 사용자: "claude_work_list/object_filter.md 내용으로 perception 필터 튜닝 해줘".
2. `_filter_workspace/` 없음 → 초기 실행.
3. analyst → coder → verifier 순차. 모두 PASS → 통합 보고 + 커밋 제안.

### 부분 재실행
1. 1 차 튜닝 후 사용자: "거리 임계값 100 → 80 으로 다시".
2. `_filter_workspace/` 보존, analyst 만 재호출 (사양 갱신) → coder/verifier 재실행.

### 에러 흐름
1. coder 가 "사양 모호" 보고 → 오케스트레이터가 사용자에게 명확화 요청.
2. 사용자 답변으로 analyst 재호출 → 사양서 갱신 → coder 재실행.

---
name: web-hmi-review
description: src/visualization/web_hmi 패키지(launch + scripts + web/)의 정합성·무결성을 4개 영역(launch, bridge, frontend, variant)으로 다각도 검토한다. 사용자가 "web_hmi 검토", "web_hmi 점검", "HMI 감사", "HMI 코드 리뷰", "web_hmi 상태 확인", "다시 검토", "재실행" 등을 요청하면 반드시 이 스킬을 사용. 발견 항목별 (severity, 위치 file:line, 권장 조치) 표를 사용자에게 제출.
---

# web-hmi-review 오케스트레이터

`src/visualization/web_hmi` 검토를 4명의 전문 auditor (launch / bridge / frontend / variant)가 병렬로 수행하고, 결과를 통합 보고서로 합친다.

## 실행 모드

**서브 에이전트 패턴 (병렬 4개)** — 각 auditor는 독립 영역을 담당하므로 팀 통신 불필요. 결과는 파일로 수집해 메인이 통합한다.

## Phase 0: 컨텍스트 확인

호출 즉시 다음을 확인:

1. `src/visualization/web_hmi/_review_workspace/` 존재 여부 확인.
2. 존재하면:
   - 사용자가 **부분 재실행** ("frontend만 다시", "bridge만 갱신") → 해당 auditor 1명만 호출, 나머지 보고서는 그대로 둠.
   - 사용자가 **새 검토** → 기존 `_review_workspace/`를 `_review_workspace_prev/` 로 이동 후 새로 시작.
   - 사용자가 모호하면 어떤 모드인지 묻는다.
3. 없으면 **초기 실행**.

`_review_workspace/`는 워크스페이스 루트 (`/home/katech/mcar_v13/`) 하위에 생성한다.

## Phase 1: 4 auditor 병렬 호출

`Agent` 도구를 한 메시지에 4번 호출. 각 호출에 `model: "opus"`, `subagent_type` 지정.

| auditor | subagent_type | 출력 |
|---------|--------------|------|
| launch-auditor | launch-auditor | `_review_workspace/01_launch_audit.md` |
| bridge-auditor | bridge-auditor | `_review_workspace/02_bridge_audit.md` |
| frontend-auditor | frontend-auditor | `_review_workspace/03_frontend_audit.md` |
| variant-auditor | variant-auditor | `_review_workspace/04_variant_audit.md` |

각 prompt에 다음을 포함:
- 점검 대상 디렉토리 절대 경로
- 출력 파일 경로
- 다른 auditor와 영역 중복 없음을 명시 (예: "JSX 토픽 구독 키 정합성은 frontend-auditor 담당이지만, 발행 측 정합성은 bridge-auditor와 교차 검증 필요")
- "발견 0건이면 그렇게 보고" — 무리한 트집 금지

## Phase 2: 통합 + 교차 정합성 검증

4개 보고서를 모두 Read하고:

1. **교차 정합성 비교**:
   - bridge-auditor가 발행한다고 보고한 토픽/JSON 키 ↔ frontend-auditor가 구독한다고 보고한 키
   - launch-auditor가 등록한 노드 ↔ bridge-auditor가 점검한 스크립트
   - variant-auditor가 본 HTML 진입점 ↔ frontend-auditor의 컴포넌트 인벤토리

2. **통합 보고서 출력** (사용자에게 직접 표시):

```markdown
## web_hmi 검토 결과 요약

총 발견 N건 (critical X · major Y · minor Z · info W)

### Critical / Major (즉시 조치)
| # | 영역 | 위치 | 발견 | 권장 조치 |

### Minor / Info (선택 개선)
| # | 영역 | 위치 | 발견 | 권장 조치 |

### 교차 검증 PASS
- ...

### 다음 단계 제안
- ...
```

3. 통합 보고서 본문 자체도 `_review_workspace/00_consolidated.md` 로 저장.

## Phase 3: 후속 처리 제안

발견 항목 중 critical/major 가 있으면 사용자에게 묻는다:
- "이 항목을 지금 수정할까요? (예: #3 토픽 키 오타 → bridge-auditor 결과 따라 수정)"
- 사용자가 수정 요청하면 별도 implementer 에이전트 호출 가능 (현재 하네스에는 미정의; 필요 시 일반 메인 도구로 직접 수정).

## 데이터 흐름 / 에러 핸들링

- 한 auditor가 실패하면 1회 재시도. 재실패 시 그 영역은 `(검토 실패)` 표시 후 진행.
- 4개 모두 실패하면 워크플로우 중단.
- 산출물은 사후 검증을 위해 `_review_workspace/`에 보존.

## 테스트 시나리오

### 정상 흐름
1. 사용자가 "web_hmi 검토" 요청.
2. `_review_workspace/` 없음 → 초기 실행.
3. 4 auditor 병렬 호출, 모두 보고서 생성.
4. 메인이 교차 검증 후 통합 보고서 출력.

### 부분 재실행
1. 1차 검토 후 사용자가 "frontend만 다시" 요청.
2. 기존 `_review_workspace/` 유지, `frontend-auditor` 만 재호출.
3. 통합 보고서 갱신.

### 에러 흐름
1. bridge-auditor가 `mmc_msgs` import 실패로 메시지 구조 분석 못 함.
2. 1회 재시도 후 `(부분 검토)` 표시 + 사용자에게 환경 확인 요청.

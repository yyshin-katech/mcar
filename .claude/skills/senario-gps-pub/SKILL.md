---
name: senario-gps-pub
description: mat_viewer_senario_*.html 의 주행 link 시퀀스를 따라 GPS 데이터를 시뮬레이션 발행하는 ROS publisher 파이썬 노드를 신규 개발한다. 기존 test_senario3_publisher.py 와 동일한 토픽/메시지 패턴을 따르되, 새 시나리오 (senario 디렉토리, 40 km/h 일정, 간단 동역학) 에 맞춰 분석→코딩→검증 파이프라인으로 산출한다. 사용자가 "senario gps publisher 만들어", "mat 시나리오 시뮬", "senario_260514c 주행 시뮬", "40km/h GPS 시뮬", "senario gps pub 다시 만들어", "publisher 갱신", "route 바꿔서 다시" 등을 요청하면 반드시 이 스킬을 사용한다. 단순 코드 질문은 직접 응답.
---

# senario-gps-pub — GPS Senario Simulator Publisher (Orchestrator)

senario 시나리오의 주행 link 시퀀스를 따라 EPSG:5179 GPS 데이터를 시뮬레이션 발행하는 신규 publisher 노드를 만든다. 기존 `test_senario3_publisher.py` 와 동일한 토픽 (`/localization/pose_2d_gps`) / 메시지 (`mmc_msgs/localization2D_msg`) 패턴을 그대로 재사용한다.

## 실행 모드
**에이전트 팀 (파이프라인)**: analyst → coder → verifier. 3명이라 팀 통신 오버헤드가 가볍고, coder 가 사양 모호점을 analyst 에게 즉시 질문할 수 있어 품질이 높다.

## Phase 0: 컨텍스트 확인

작업 시작 전 `_workspace/` 상태로 실행 모드 결정.

| 상태 | 모드 |
|------|------|
| `_workspace/` 없음 | **초기 실행** — 전체 파이프라인 |
| `_workspace/` 존재 + 사용자가 부분 수정 요청 (예: "속도만 50km/h로", "route 끝에 link 추가") | **부분 재실행** — 해당 에이전트만 재호출 |
| `_workspace/` 존재 + 사용자가 새 입력 제공 (다른 HTML, 다른 시나리오) | **새 실행** — 기존을 `_workspace_prev/` 로 mv 후 처음부터 |

## Phase 1: 분석 (senario-sim-analyst)

목표: HTML 에서 link 시퀀스 추출 + 기존 publisher 패턴 분석 → 사양서 1개.

핵심 입력:
- `src/localization/gps_system_localizer/mapfiles/senario/mat_viewer_senario_260514c 1.html` (파일명에 공백 주의 — 따옴표/이스케이프)
- `src/localization/gps_system_localizer/src/test_senario3_publisher.py`

산출물: `_workspace/01_analyst_spec.md`

## Phase 2: 코딩 (senario-sim-coder)

목표: 사양서 그대로 publisher .py 1개 작성.

산출물:
- 사양서의 신규 파일 경로 (예: `src/localization/gps_system_localizer/src/test_senario_260514c_publisher.py`)
- `_workspace/02_coder_report.md`

## Phase 3: 검증 (senario-sim-verifier)

목표: py_compile → import dry-run → (가능 시) roscore + rostopic echo.

산출물: `_workspace/03_verifier_report.md` + PASS/FAIL verdict.

FAIL 시 coder 재호출. 2회 연속 FAIL 이면 사용자에게 보고 후 중단.

## 데이터 흐름

```
analyst ──spec──> coder ──publisher.py──> verifier ──verdict──> 사용자
                            │                    │
                            └── (FAIL 시 재작업) ─┘
```

전달 방식: 파일 기반 (`_workspace/`) + 메시지 기반 (SendMessage 로 알림). 사양서/보고서는 보존.

## 차량 동역학 가이드라인 (사양서 작성 시 분석가가 결정)

- **기본 권장**: `s += v*dt` 누적 station + 경로 위 (e, n, yaw) 보간 — 기존 publisher 와 같은 구조, 40 km/h 일정에 충분.
- **대안**: kinematic bicycle (x, y, yaw) 상태 + pure pursuit. 단순 시뮬에서는 over-engineering 위험.
- 둘 중 무엇이든 사양서에 의사코드 2~5줄로 명시. 가감속 없음 (40 km/h 상수).

## 에러 핸들링

| 단계 | 실패 유형 | 대응 |
|------|----------|------|
| 분석 | HTML 에서 link 시퀀스 못 찾음 | 사양서 작성 중단, 사용자 확인 요청. senario3 시퀀스 복붙 금지. |
| 분석 | 기존 publisher 가 NavPVT 가 아닌 localization2D_msg 발행 | 사용자가 "NavPVT 등" 이라 했어도 코드 패턴 우선. 사양서에 명시. |
| 코딩 | 사양서 모호 | analyst 에 SendMessage 1회. 응답 없으면 사양서 그대로 + 보고서에 표기. |
| 검증 | mat 파일 missing | "ROUTE_LINK_IDS 가 mapfiles/senario/ 와 어긋남" 보고. 코더는 사양서의 missing 정책 적용. |
| 검증 | roscore 불가 | 1~4 단계만 수행, 5 단계 skip 명시. 결과는 부분 PASS 로 처리. |

## 에이전트 호출 패턴

오케스트레이터는 다음 순서로 Agent 도구를 호출한다. 모두 `model: "opus"` 명시.

```
Phase 1:
  Agent(subagent_type="general-purpose", name="senario-sim-analyst",
        model="opus", prompt="<에이전트 정의 + 사용자 요청 + Phase 0 결과>")

Phase 2 (Phase 1 완료 후):
  Agent(subagent_type="general-purpose", name="senario-sim-coder",
        model="opus", prompt="<에이전트 정의 + 사양서 경로>")

Phase 3 (Phase 2 완료 후):
  Agent(subagent_type="general-purpose", name="senario-sim-verifier",
        model="opus", prompt="<에이전트 정의 + 산출 파일 경로 + 실행 명령>")
```

에이전트 정의는 각자 `.claude/agents/<name>.md` 에서 읽어 전달. 사양서/코드/보고서는 `_workspace/` 에 저장하므로 다음 에이전트는 파일 경로만 받으면 충분.

## 테스트 시나리오

### 정상 흐름
1. 사용자: "senario_260514c 주행 시뮬 publisher 만들어줘, 40km/h"
2. analyst: HTML 에서 link 시퀀스 N개 추출 → `_workspace/01_analyst_spec.md`
3. coder: 사양서대로 `test_senario_260514c_publisher.py` 작성 → `_workspace/02_coder_report.md`
4. verifier: py_compile OK, roscore + rostopic echo → 1 메시지 (east/north/yaw) 확인 → PASS
5. 사용자에게 최종 보고 (파일 경로 + 실행 명령 + 보고서 위치)

### 에러 흐름 (HTML 파싱 실패)
1. 사용자 요청 동일
2. analyst: HTML 에 link id 가 leaflet 라이브러리 외엔 안 보임 → "파싱 실패, 사용자 확인 필요" 보고
3. 오케스트레이터: 사용자에게 "HTML 구조 확인 필요. ROUTE_LINK_IDS 직접 제공 가능?" 질문
4. 사용자 응답 후 재실행 (Phase 0 의 부분 재실행 모드)

## 작업 디렉토리

루트: 프로젝트 루트 (`/home/katech/mcar_v13` 또는 `/home/ads/mcar_v13`).
- `_workspace/01_analyst_spec.md`
- `_workspace/02_coder_report.md`
- `_workspace/03_verifier_report.md`
- 신규 publisher: `src/localization/gps_system_localizer/src/<analyst가 명명>.py`

## 산출물 체크리스트 (오케스트레이터가 사용자에게 보고 시)

- [ ] 신규 publisher .py 파일 경로
- [ ] ROUTE_LINK_IDS 길이 + 총 주행 거리 (m) + ETA (s)
- [ ] 검증 단계별 결과 (PASS/SKIP/FAIL)
- [ ] 실행 명령: `rosrun gps_system_localizer <name>.py` 또는 `python3.8 <path>`
- [ ] `_workspace/` 보고서 3개 위치

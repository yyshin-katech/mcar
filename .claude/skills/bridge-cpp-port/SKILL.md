---
name: bridge-cpp-port
description: src/visualization/web_hmi/scripts/web_hmi_threejs_bridge.py 등 web_hmi 의 Python ROS 브리지를 동등 기능의 C++ ROS 노드로 포팅. /hmi/threejs/tracks 가 콜백 지연으로 1 Hz 까지 떨어져 트랙 박스가 1초마다 점프하는 문제를 C++ 로 재작성해 percept 10 Hz 를 따라가게 만든다. 사용자가 "브리지 cpp 포팅", "브릿지 파이썬을 cpp 로", "web_hmi_threejs_bridge cpp", "bridge cpp 다시 적용", "트랙 박스 1초 점프 해결" 등을 요청하면 반드시 이 스킬을 사용. 단순 코드 질문은 직접 응답.
---

# bridge-cpp-port 오케스트레이터

`src/visualization/web_hmi/scripts/web_hmi_threejs_bridge.py` 의 트랙 발행 경로(`/hmi/threejs/tracks`)를 동등 기능의 C++ ROS 노드로 포팅한다. 3 단계 파이프라인 (분석 → 구현 → 검증).

## 실행 모드

**파이프라인 패턴 (서브 에이전트 순차)** — `Agent` 도구로 1 회씩 호출. 각 단계는 직전 산출물(`_bridge_cpp_workspace/*.md`)을 입력으로 사용. 병렬화 불가능. `model: "opus"` 명시.

## Phase 0: 컨텍스트 확인

호출 즉시:

1. `_bridge_cpp_workspace/` 존재 여부 확인.
2. 존재하면:
   - 사용자가 **부분 재실행** ("분석만 다시", "구현만", "검증만 다시", "빌드만") → 해당 단계 에이전트만 재호출.
   - 사용자가 **새 포팅 시작** (사양 변경, 다른 파이썬 노드 지정) → 기존 `_bridge_cpp_workspace/` 를 `_bridge_cpp_workspace_prev/` 로 이동.
3. 없으면 **초기 실행**.
4. 사용자가 추가 사양 (예: 옵션 1 전체 포팅 명시) 을 알려주면 analyst 에게 그대로 전달.

`_bridge_cpp_workspace/` 는 워크스페이스 루트(`/home/ads/mcar_v13/`) 하위.

## Phase A: bridge-port-analyst

`Agent` 1 회 호출. `subagent_type='bridge-port-analyst'` (정의 파일이 있으면) 또는 `subagent_type='general-purpose'`. `model: 'opus'`.

prompt 필수 항목:
- "먼저 `/home/ads/mcar_v13/.claude/agents/bridge-port-analyst.md` 를 Read 하고 그 정의를 따라 작업하라."
- 출력 파일: `_bridge_cpp_workspace/01_port_spec.md`
- 대상 Python: `src/visualization/web_hmi/scripts/web_hmi_threejs_bridge.py`
- 사용자 추가 지시 (옵션 1/2 강제 등) 인용
- "발견 0건이면 그렇게 보고. 무리한 트집 금지."

**대기**: analyst 완료 보고 대기. 보고 없이 다음 단계 진행 금지.

## Phase B: bridge-port-coder

analyst 보고서 Read 후 `Agent` 1 회 호출. `subagent_type='bridge-port-coder'` 또는 `'general-purpose'`. `model: 'opus'`.

prompt:
- "먼저 `/home/ads/mcar_v13/.claude/agents/bridge-port-coder.md` 를 Read 하고 그 정의를 따라 작업하라."
- 입력: `_bridge_cpp_workspace/01_port_spec.md` 전문
- 출력: `_bridge_cpp_workspace/02_coder_changes.md`
- 외과적 변경 원칙: 사양 외 변경 금지, Python 파일 삭제 금지
- 빌드 명령: `cd /home/ads/mcar_v13 && catkin_make --pkg web_hmi`

**대기**: coder 완료 보고 대기. 빌드 PASS 까지 확인. PASS 못하면 verifier 진입 금지.

## Phase C: bridge-port-verifier

`Agent` 1 회 호출. `subagent_type='bridge-port-verifier'` 또는 `'general-purpose'`. `model: 'opus'`.

prompt:
- "먼저 `/home/ads/mcar_v13/.claude/agents/bridge-port-verifier.md` 를 Read 하고 그 정의를 따라 작업하라."
- 입력: coder 가 변경한 파일들(`02_coder_changes.md` 참조) + 사양서(`01_port_spec.md`)
- 출력: `_bridge_cpp_workspace/03_verify_report.md`
- 라이브 환경(`rostopic list`) 있으면 hz/bw 측정, 없으면 정적 검증만.

## Phase D: 통합 보고

3 개 보고서 모두 Read 후 사용자에게 직접 표시:

```markdown
## web_hmi 브리지 cpp 포팅 결과 요약

### 분석 → 구현 → 검증
| 단계 | 결과 | 산출물 |

### 변경 파일
| 파일 | 라인 | 변경 요약 |

### 검증 PASS / FAIL
- 빌드 (catkin_make --pkg web_hmi):
- /hmi/threejs/tracks hz:
- 페이로드 키 호환:

### 다음 단계 제안
- 사용자가 실행해야 할 launch 변경 / 노드 재시작
```

`_bridge_cpp_workspace/00_consolidated.md` 로 동시 저장.

## Phase E: 후속 처리

- coder 가 "사양 모호" 보고 → 사용자에게 추가 지시 요청.
- verifier FAIL B (사양 위배) → coder 1 회만 재호출 (무한 루프 방지). 그래도 FAIL 이면 사용자 개입.
- verifier FAIL C (사양 자체 문제) → analyst 1 회 재호출 후 coder 재실행.
- verifier FAIL A (환경) / FAIL D (라이브 미준비) → 사용자 개입 안내.
- 모두 PASS 면 launch 변경/노드 재시작 가이드를 사용자에게 제시하고 커밋 여부 확인.

## 데이터 흐름 / 에러 핸들링

- 단계 간 데이터: `_bridge_cpp_workspace/` 파일 기반.
- analyst 실패 (Python 파일 못 찾음, msg 정의 없음) → 환경 문제 보고 후 중단.
- coder 실패 (빌드 깨짐) → 변경 롤백(`git checkout -- <file>`) 후 사양 재검토.
- verifier FAIL → 위 분류대로 분기.

## 테스트 시나리오

### 정상 흐름
1. 사용자: "web_hmi threejs bridge cpp 로 포팅해줘".
2. `_bridge_cpp_workspace/` 없음 → 초기 실행.
3. analyst (옵션 2 권장) → coder (web_hmi_threejs_tracks_node.cpp 추가, Python `_pub_tracks` 비활성화) → verifier (hz 9 Hz, 페이로드 일치) → 통합 보고 + launch 가이드.

### 부분 재실행
1. 1 차 포팅 후 사용자: "전체 포팅 (옵션 1) 으로 다시".
2. `_bridge_cpp_workspace/` 보존, analyst 만 재호출 (옵션 변경 사양 갱신) → coder/verifier 재실행.

### 에러 흐름
1. coder 가 "nlohmann/json 미설치, vendoring 위치 사양 모호" 보고 → 오케스트레이터가 사용자에게 명확화 요청.
2. 사용자 답변으로 analyst 재호출 → 사양서 갱신 → coder 재실행.

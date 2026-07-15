---
name: vehicle-tracker
description: BSM 기반 차량 위치 관제 시스템 신규 개발 오케스트레이터. AWS EC2 (13.209.88.22) 에 올릴 Node.js Express+ws 서버 + Leaflet 프론트엔드 + 차량 측 ROS Python forwarder 노드를 analyst → coder → verifier 3-phase 로 생성. 사용자가 "차량 관제", "vehicle tracker", "BSM 서버", "차량 위치 웹 관제", "관제 다시 만들어", "frontend 갱신", "forwarder 노드 수정" 등을 요청하면 반드시 이 스킬을 사용. 단순 질문은 직접 응답.
---

# vehicle-tracker 오케스트레이터

목적: 차량의 BSM(Basic Safety Message) 데이터를 JSON 으로 EC2 서버로 실시간 전송하고, 웹 지도(Leaflet)에서 차량 위치/속도/방향(heading)을 실시간 표시. 배포 대상: AWS EC2 t3.small (13.209.88.22).

## 결정 사항 (사용자 합의 완료)

- 서버: **Node.js Express + ws** (WebSocket)
- 차량→서버 전송: **WebSocket**
- 다중 차량: **vehicle_id 기준 지원**
- 저장: **메모리만 (최신 상태)** — 차량별 최신 BSM 만 dict 로 유지
- 지도: **Leaflet + OpenStreetMap 타일** (토큰 불요)
- 포트: **8080** (HTTP + WS, 동일 포트)

## 디렉토리

- `server/` — Node.js Express + ws 서버 (워크스페이스 루트 하위)
- `frontend/` — HTML + Leaflet + 순수 JS (워크스페이스 루트 하위)
- ROS forwarder 노드: analyst 가 위치 결정 (기존 ROS 패키지 신규 추가 또는 src 산하)
- `_vehicle_tracker_workspace/` — 사양/구현/검증 산출물 (gitignore 대상)

## 실행 모드

3-phase 파이프라인 (analyst → coder → verifier). 병렬 불가 (각 단계가 이전 단계 산출물을 입력으로 받음).

## Phase 0: 컨텍스트 확인

호출 즉시:

1. `_vehicle_tracker_workspace/` 존재 여부.
2. 존재하면:
   - 사용자가 부분 재실행 ("분석만 다시", "검증만") → 해당 단계만 재호출.
   - 사용자가 전체 재실행 → 기존 `_vehicle_tracker_workspace/` 를 `_vehicle_tracker_workspace_prev/` 로 이동.
3. 없으면 초기 실행.

## Phase A: vehicle-tracker-analyst

`Agent` 1회 호출. `subagent_type='general-purpose'`, `model: 'opus'`.

prompt 필수 항목:
- "먼저 `/home/sim/mcar/.claude/agents/vehicle-tracker-analyst.md` 를 Read 하고 그 정의를 따라 작업하라."
- 입력: BSM ROS 토픽/메시지 구조 (`src/v2x/siheung_v2x/src/{bsm_tx_node,mqtt_bsm_tx_node}.cpp`, `src/msgs/mmc_msgs/msg/`).
- 출력: `_vehicle_tracker_workspace/01_spec.md`
- 사양서 구조: BSM 필드 → JSON 스키마 매핑 / 서버 API (REST + WS) / 프론트엔드 UI / 디렉토리 트리 / 패키지 의존성 / 포트 정책 / 보안 처리(미적용 — 후속 단계).

대기: analyst 완료 보고 대기.

## Phase B: vehicle-tracker-coder

analyst 보고서 Read 후 `Agent` 1회 호출. `subagent_type='general-purpose'`, `model: 'opus'`.

prompt:
- "먼저 `/home/sim/mcar/.claude/agents/vehicle-tracker-coder.md` 를 Read 하고 그 정의를 따라 작업하라."
- 입력: `_vehicle_tracker_workspace/01_spec.md`
- 출력: `_vehicle_tracker_workspace/02_implementation.md` + 실제 `server/`, `frontend/`, ROS forwarder 노드 파일들.
- 외과적 구현: 사양서 범위 밖 파일 변경 금지.

대기: coder 완료 보고 대기.

## Phase C: vehicle-tracker-verifier

coder 보고서 Read 후 `Agent` 1회 호출. `subagent_type='general-purpose'`, `model: 'opus'`.

prompt:
- "먼저 `/home/sim/mcar/.claude/agents/vehicle-tracker-verifier.md` 를 Read 하고 그 정의를 따라 작업하라."
- 입력: `_vehicle_tracker_workspace/02_implementation.md` + 코드 파일들.
- 출력: `_vehicle_tracker_workspace/03_verify.md`
- 정적 검증 (Node syntax `node --check`, Python syntax `python -m py_compile`) + 가능 시 로컬 dry-run (`npm install --omit=optional`, `node server/index.js`, `wscat` 또는 `curl` 으로 WS 핸드셰이크 확인).
- ROS 측 forwarder 의 catkin_make 가능 여부 환경 의존 → SKIP 시 사유 명시.

대기: verifier 완료 보고 대기.

## Phase D: 통합 보고

세 보고서를 Read 후 사용자에게 직접 표시:

```markdown
## 차량 관제 시스템 신규 개발 결과 요약

### 산출 디렉토리
- server/ (파일 N개)
- frontend/ (파일 M개)
- ROS forwarder: <패키지 경로>

### 빌드/검증
- Node syntax/실행: PASS/FAIL
- Python syntax: PASS/FAIL
- Dry-run: PASS/SKIP (사유)

### 배포 준비
- 사용자가 EC2 배포 정보 (ssh key, user, 보안그룹) 제공하면 deploy 별도 안내.
```

`_vehicle_tracker_workspace/00_consolidated.md` 로 동시 저장.

## Phase E: 후속 처리

- analyst 가 "사양 모호" 보고 → 사용자에게 추가 지시 요청 (BSM 필드/UI 등).
- verifier 가 FAIL 보고 → coder 1회 재호출 (무한 루프 방지 위해 1회 한정).
- 모두 PASS → 사용자에게 다음 단계 (EC2 배포) 안내, 정보 수령 대기.

## 작업 원칙

- ROS 측 변경: 신규 forwarder 노드만 추가, 기존 BSM 노드 (`bsm_tx_node`, `mqtt_bsm_tx_node`) 는 손대지 않음.
- 보안: 초기 버전은 인증 없음 (사용자 추후 요청 시 별도 layer 추가).
- 로컬 LAN/사설망 가정 (HTTPS/WSS 는 EC2 배포 단계에서 nginx + Let's Encrypt 옵션 검토).
- 디렉토리 `server/`, `frontend/` 는 워크스페이스 루트 하위 (`/home/sim/mcar/`).
- 산출 보고서는 갭/이슈가 0건이어도 명시. 무리한 트집 금지.

## 테스트 시나리오

### 정상 흐름
1. 사용자 "차량 관제 만들어줘".
2. `_vehicle_tracker_workspace/` 없음 → 초기 실행.
3. analyst → coder → verifier 순차 실행.
4. 모두 PASS → 배포 정보 요청.

### 부분 재실행
1. 1차 완료 후 "프론트엔드만 다시" 요청.
2. coder 만 재호출, scope 를 frontend 로 한정.
3. verifier 재호출.

### FAIL 흐름
1. verifier 가 syntax FAIL 보고.
2. coder 1회 재호출.
3. 그래도 FAIL → 사용자 개입 요청.

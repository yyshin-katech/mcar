---
name: ioniq5-siheung-port
description: siheung_dev(아이오닉 EV) 브랜치의 지도·HMI·V2X·보행자검출을 ioniq5_siheung_dev(IONIQ 5 + ublox GPS) 브랜치로 이식해, IONIQ 5 시스템이 시흥 지역에서 아이오닉 EV 와 동일한 경로·미션을 수행하도록 만든다. 설계(fable5) → 코딩(sonnet5) → 검증(opus5) 3단계 파이프라인. 사용자가 "ioniq5 포팅", "시흥 이식", "브랜치 포팅", "siheung_v2x 이식", "DBC v8 통일", "포팅 계획", "포팅 검증", "이식 결과 확인", "claude_work_list 작업 시작", "ioniq5_job 작업", 또는 "다시 실행", "재실행", "이어서", "CAN 부분만 다시", "V2X만 다시", "검증만 다시", "계획 수정", "보완" 을 요청하면 반드시 이 스킬을 사용한다. 단일 파일 질문이나 단순 조회는 직접 응답.
---

# ioniq5-siheung-port 오케스트레이터

`claude_work_list/ioniq5_job_20260820.md` 작업지시서를 실행한다.

## 실행 모드

**하이브리드 (Phase 별 서브 에이전트 팬아웃 → 메인 통합)**

이 빌드에는 `TeamCreate` 가 없다. 에이전트 팀 대신 `Agent` 도구로 서브 에이전트를 병렬 스폰하고, 데이터는 `_ioniq5_port_workspace/` 파일로 주고받는다.

| Phase | 모드 | 모델 | 에이전트 |
|-------|------|------|---------|
| 1. 설계 | 팬아웃 4 (병렬) | **fable** | `port-designer` ×4 (can / v2x / percept / map-hmi) |
| 2. 코딩 | 팬아웃 3 (병렬) | **sonnet** | `can-dbc-porter`, `v2x-porter`, `percept-porter` |
| 3. 검증 | 팬아웃 4 (병렬) | **opus** | `port-verifier` ×4 |

모델은 작업지시서가 단계별로 지정한 값이다. `Agent` 호출 시 `model` 파라미터로 반드시 명시한다 (하네스 기본값 opus 를 덮어쓴다).

## Phase 0: 컨텍스트 확인 — 항상 여기부터

```bash
ls -la _ioniq5_port_workspace/ 2>/dev/null
git branch --show-current   # ioniq5_siheung_dev 여야 한다
```

| 상태 | 실행 모드 |
|------|----------|
| workspace 없음 | **초기 실행** — Phase 1부터 전체 |
| workspace 있음 + "CAN만/V2X만/검증만" 요청 | **부분 재실행** — 해당 에이전트만 재호출 (이전 산출물을 입력으로 전달) |
| workspace 있음 + 새 작업지시서 | **새 실행** — `_ioniq5_port_workspace/` → `_ioniq5_port_workspace_prev/` 이동 후 초기 실행 |
| workspace 있음 + 이어서 요청 | **이어하기** — 가장 마지막에 완료된 Phase 다음부터 |

브랜치가 `ioniq5_siheung_dev` 가 아니면 **멈추고 사용자에게 확인**한다. 다른 브랜치에서 이식하면 전부 무의미하다.

## Phase 1: 설계 (fable)

### 1-1. 범위 정의서 작성

메인이 직접 `_ioniq5_port_workspace/00_scope.md` 를 쓴다. 작업지시서 + 사용자가 확정한 범위 결정을 담는다. 4개 designer 가 공통으로 읽는 단일 진실원이다.

### 1-2. designer 4개 병렬 스폰

각 designer 는 `port-designer` 타입, `model: "fable"`, 담당 영역 하나. 프롬프트에 영역 경계를 못박는다.

| 영역 | 담당 경로 | 산출물 |
|------|----------|--------|
| `can` | `src/sensing/can/`, DBC, `launch/*.launch` 의 CAN 노드 | `01_design_can.md` |
| `v2x` | `src/v2x/`, `v2x_msgs`, `bsm_uploader`, `spat_viewer`, `siheung.launch` | `01_design_v2x.md` |
| `percept` | `katech_ped_detector*`, `crosswalk_ped_fusion`, `katech_custom_msgs`, `rviz_filter` | `01_design_percept.md` |
| `map-hmi` | `gps_system_localizer`, `web_hmi`, `pyqt_hmi`, `stat_display`, `mmc_msgs` — **VERIFY 위주** | `01_design_map_hmi.md` |

`map-hmi` 는 이미 이식된 영역이다(커밋 `dd2db43`). 작업지시서가 "수정되어 있는 부분은 검증 부분만 추가해서 검증" 이라 했으므로 이 designer 는 **PORT/ADAPT 항목을 만들지 말고 VERIFY 항목만** 만든다.

### 1-3. 계획 통합 및 사용자 승인 — 건너뛰지 않는다

4개 계획서를 읽고 `_ioniq5_port_workspace/01_design_MERGED.md` 로 합친다:
- 공유 파일(`launch/katech_test.launch`, `CMakeLists.txt`, msg 패키지) 충돌 항목을 한 곳에 모은다
- 영역 간 상충(같은 파일을 다르게 고치라는 지시)을 해소한다
- 모든 designer 의 **미해결 질문**을 모아 사용자에게 제출한다

**계획을 사용자에게 보고하고 승인받기 전에 Phase 2로 넘어가지 않는다.** 이 프로젝트의 변경은 차량 거동에 직결되고, 되돌리는 비용이 크다.

## Phase 2: 코딩 (sonnet)

승인된 `01_design_MERGED.md` 기준으로 porter 3개를 병렬 스폰한다 (`model: "sonnet"`).

### 공유 파일 충돌 회피

`launch/katech_test.launch`, `src/sensing/can/CMakeLists.txt` 는 여러 porter 가 건드린다. 락 규약:
```bash
# 편집 전
[ -f _ioniq5_port_workspace/LOCK_katech_test.launch ] && echo "대기" || echo "<내이름>" > _ioniq5_port_workspace/LOCK_katech_test.launch
# 편집 후
rm -f _ioniq5_port_workspace/LOCK_katech_test.launch
```
락 경합이 반복되면 메인이 해당 파일 편집을 직접 수행하고, porter 에게는 "무엇을 어떻게 바꿔달라" 는 지시만 받는다.

### msg 패키지 우선

`v2x_msgs`, `katech_custom_msgs`, `mmc_msgs` 를 건드리는 porter 가 있으면 **그 porter 를 먼저 단독 실행**한 뒤 나머지를 병렬로 돌린다. 헤더 경합으로 빌드가 깨진다.

## Phase 3: 검증 (opus)

`port-verifier` 4개를 병렬 스폰한다 (`model: "opus"`). 영역은 Phase 1과 동일 + 각 porter 의 "검증 요청 사항" 을 입력으로 준다.

**점진 검증**: porter 하나가 끝나면 전체를 기다리지 말고 해당 영역 검증을 바로 시작한다.

검증 결과에 `CONFIRMED` 가 있으면:
1. 해당 porter 를 1회 재호출해 수정
2. 재검증
3. 2회차에도 CONFIRMED 면 **고치지 말고 사용자에게 보고**한다 (설계 결함일 가능성)

## 데이터 전달 프로토콜

파일 기반. `_ioniq5_port_workspace/` 하위, 컨벤션 `{phase}_{종류}_{영역}.md`:

```
00_scope.md                 메인 작성, 전 에이전트 공통 입력
01_design_{영역}.md         designer 산출
01_design_MERGED.md         메인 통합 + 사용자 승인 대상
02_impl_{영역}.md           porter 산출
03_verify_{영역}.md         verifier 산출
04_FINAL_REPORT.md          메인 최종 통합
LOCK_*                      공유 파일 편집 락
```

중간 파일은 지우지 않는다 (사후 감사 추적용).

## 에러 핸들링

| 상황 | 조치 |
|------|------|
| 에이전트 실패 | 1회 재시도. 재실패 시 그 영역 없이 진행하되 **최종 보고서에 누락을 명시**한다. 조용히 넘기지 않는다. |
| 빌드 실패 | msg 를 건드렸으면 재빌드 1회는 정상(헤더 경합). 2회째 실패는 진짜 실패 — 되돌리고 오류 전문 기록. |
| 계획 ↔ 실제 코드 불일치 | **실제 코드가 기준.** 계획서를 갱신하고 진행. |
| 두 영역이 같은 줄을 다르게 수정 | 삭제하지 말고 양쪽 출처를 병기해 사용자에게 판단 요청. |
| DBC 시그널이 목표 버전에 없음 | **멈춘다.** 유사 시그널로 대체하지 않는다. 제어팀 동기화 사안. |
| 브랜치가 ioniq5_siheung_dev 아님 | 즉시 중단, 사용자 확인. |

## 완료 기준

`_ioniq5_port_workspace/04_FINAL_REPORT.md` 에 아래가 모두 채워졌을 때 완료다:
- [ ] `catkin_make` 전체 빌드 성공 (출력 인용)
- [ ] `launch/katech_test.launch` 의 모든 `type=` 실행파일이 `devel/lib/` 에 실재
- [ ] DBC 3종 외 참조 없음 — 남아 있으면 이유 명시
- [ ] CAN 채널 번호 변경 0건 (`git diff` 로 증명)
- [ ] novatel 잔존 의존 전수 목록 + 각각의 판정
- [ ] CONFIRMED 결함 0건 또는 사용자 인지된 잔여 항목
- [ ] 실기 확인 요청 목록 (CANoe / rostopic / 화면)

**빌드 성공은 완료가 아니다.** 실차 CAN 송신·토픽 주기·HMI 표시는 정적 분석으로 판정 불가하므로, 최종 보고서는 반드시 "사용자가 실기로 확인해야 할 항목" 을 남긴다.

## 테스트 시나리오

**정상 흐름**: 사용자 "ioniq5_job 작업 시작" → Phase 0 초기 실행 판정 → `00_scope.md` 작성 → designer 4개(fable) 병렬 → `01_design_MERGED.md` + 미해결 질문 사용자 제출 → 승인 → porter 3개(sonnet), msg 담당 먼저 → verifier 4개(opus) 점진 검증 → CONFIRMED 1건 발견 → 해당 porter 재호출 수정 → 재검증 OK → `04_FINAL_REPORT.md` + 실기 확인 요청 제출.

**에러 흐름**: v2x-porter 가 `siheung_v2x` 이식 후 `bsm_tx_node` 에서 novatel `/sensors/gps/inspva` 구독을 발견 → 설계서에 ublox 대체 지정 없음 → 코드를 고치지 않고 미처리 항목에 기록 → 메인이 사용자에게 "BSM 위치 소스를 `/ublox/navpvt` 로 대체할지, 필드 매핑(위경도/속도/헤딩)을 어떻게 할지" 질의 → 사용자 답변 후 부분 재실행으로 v2x-porter 만 재호출.

**부분 재실행 흐름**: 사용자 "CAN 부분만 다시" → Phase 0 이 workspace 존재 + 부분 요청 판정 → `01_design_can.md` 를 입력으로 `can-dbc-porter` 만 재호출 → `port-verifier`(can) 재실행 → 최종 보고서의 CAN 절만 갱신.

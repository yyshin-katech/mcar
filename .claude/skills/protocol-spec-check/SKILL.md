---
name: protocol-spec-check
description: ~/protocol/ 의 hwpx 규격 문서(경찰청 V2N 정보연계 규격)에서 MQTT 인터페이스 사양을 추출하여 현재 구현(mqtt_spat_rx_node, mqtt_bsm_tx_node, launch/siheung.launch)과 정합성을 검증. 사용자가 "프로토콜 규격 확인", "V2N 규격 검증", "hwpx 규격 비교", "MQTT 인터페이스 정합성 확인" 등을 요청하면 반드시 이 스킬을 사용. 코드 변경 없음(정적 분석 + 정적 검증). 패치가 필요하면 별도 하네스로 위임 권장.
---

# protocol-spec-check 오케스트레이터

목적: `~/protocol/[ITSK-00150-2]V2N규격-제2부 V2N정보연계-경찰청연구과제규격.hwpx` 의 MQTT 인터페이스 규격을 추출하여 `siheung_v2x` 의 `mqtt_spat_rx_node` / `mqtt_bsm_tx_node` / `launch/siheung.launch` 가 규격에 정합한지 코드 변경 없이 검증.

라이브 ROS 토픽 echo 는 SKIP (사용자 정책). 정적 분석 위주.

## 실행 모드

2단계 파이프라인 (analyst → verifier). 병렬 불가 (verifier 가 analyst 산출물을 입력으로 받음).

## Phase 0: 컨텍스트 확인

호출 즉시:

1. `~/protocol/` 의 hwpx 문서 존재 여부 (`ls /home/sim/protocol/*.hwpx`) 확인.
2. `_protocol_spec_workspace/` 존재 여부 확인.
3. 존재하면:
   - 사용자가 부분 재실행 ("분석만 다시", "검증만") → 해당 단계만 재호출.
   - 사용자가 새 검증 → 기존 `_protocol_spec_workspace/` 를 `_protocol_spec_workspace_prev/` 로 이동.
4. 없으면 초기 실행.

`_protocol_spec_workspace/` 는 워크스페이스 루트 (`/home/sim/mcar/`) 하위.

## Phase A: protocol-spec-analyst

`Agent` 1회 호출. `subagent_type='general-purpose'`, `model: 'opus'`.

prompt 필수 항목:
- "먼저 `/home/sim/mcar/.claude/agents/protocol-spec-analyst.md` 를 Read 하고 그 정의를 따라 작업하라."
- 입력: `~/protocol/[ITSK-00150-2]V2N규격-제2부 V2N정보연계-경찰청연구과제규격.hwpx`
- 출력: `_protocol_spec_workspace/01_spec_extracted.md`
- hwpx 압축 해제 → section XML 텍스트 추출 → MQTT 관련 사양 정리 → 검증 체크리스트 작성

대기: analyst 완료 보고 대기. 보고 없이 다음 단계 진행 금지.

## Phase B: protocol-spec-verifier

analyst 보고서 Read 후 `Agent` 1회 호출. `subagent_type='general-purpose'`, `model: 'opus'`.

prompt:
- "먼저 `/home/sim/mcar/.claude/agents/protocol-spec-verifier.md` 를 Read 하고 그 정의를 따라 작업하라."
- 입력: `_protocol_spec_workspace/01_spec_extracted.md` 의 "검증 체크리스트" 섹션
- 출력: `_protocol_spec_workspace/02_verify_report.md`
- 검증 대상: `src/v2x/siheung_v2x/src/mqtt_spat_rx_node.cpp`, `src/v2x/siheung_v2x/src/mqtt_bsm_tx_node.cpp`, `launch/siheung.launch`
- 정적 검증만 (라이브 토픽 echo SKIP — 사용자 정책)

대기: verifier 완료 보고 대기.

## Phase C: 통합 보고

두 보고서를 Read 후 사용자에게 직접 표시:

```markdown
## V2N MQTT 규격 정합성 검증 결과 요약

### 사양 항목 N건 → 검증 결과 (PASS/WARN/FAIL)

| 분류 | 항목 | 결과 | 비고 |

### 발견 정합성 이슈
- (FAIL/WARN 만)

### 다음 단계 제안
- (라이브 검증으로 더 확인할 항목, 패치가 필요하면 후속 하네스 호출 권장)
```

`_protocol_spec_workspace/00_consolidated.md` 로 동시 저장.

## Phase D: 후속 처리

- analyst 가 "사양 모호" 보고 → 사용자에게 추가 지시 요청.
- verifier 가 FAIL 보고 → analyst 1회 재호출 (무한 루프 방지 위해 1회 한정).
- 모두 PASS → 추가 라이브 검증 필요 항목 사용자에게 제안만, 코드 변경 금지.
- 패치 필요 시 별도 하네스로 위임 권장 (사용자 동의 후).

## 데이터 흐름 / 에러 핸들링

- 단계 간 데이터: `_protocol_spec_workspace/` 파일 기반.
- analyst 실패 (hwpx 압축 해제 불가, XML 파싱 실패) → 환경 문제 보고 후 중단.
- verifier 실패 → analyst 1회 재호출, 그래도 실패면 사용자 개입.

## 작업 원칙

- **코드 변경 금지** (정적 분석/검증만).
- hwpx 추출 시 HWPML XML 의 `<hp:t>` 태그 정규식이 표준 추출 방법. 인코딩은 UTF-8.
- 표/도식이 이미지로만 들어있어 텍스트 추출 불가능한 경우 → "이미지 N에 표/도식 포함됨, 텍스트 추출 불가" 명시. 사용자에게 추가 변환 요청.
- 라이브 ROS 토픽 검증은 사용자가 별도 요청하지 않으면 SKIP.
- 보고서는 갭/이슈가 0건이어도 그 결과 명시. 무리한 트집 금지.

## 테스트 시나리오

### 정상 흐름
1. 사용자 "프로토콜 규격 확인해줘".
2. `_protocol_spec_workspace/` 없음 → 초기 실행.
3. analyst → verifier 순차 실행.
4. 모두 PASS → 라이브 검증 권장만 제안.

### 부분 재실행
1. 1차 검증 후 "규격 문서 갱신했으니 다시 분석" 요청.
2. `_protocol_spec_workspace/` 보존, analyst 만 재호출.
3. 새 이슈 있으면 verifier 이어 실행.

### FAIL 흐름
1. analyst 가 규격 항목 N건 추출.
2. verifier 가 구현 측 mismatch FAIL 보고 (예: BSM 발행 주기 규격 10Hz vs 코드 5Hz).
3. 오케스트레이터가 사용자에게 패치 방향 보고하고 별도 하네스/직접 수정 동의 요청.

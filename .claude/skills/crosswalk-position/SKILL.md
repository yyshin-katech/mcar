---
name: crosswalk-position
description: claude_work_list/crosswalk_position.md 의 횡단보도 폴리곤(WGS84)을 katech_ped_detector.py 의 EPSG:5179 crosswalk_data 로 교체하고 senario mat 뷰어에 다각형(채움+토글+범례)으로 표시하는 하네스. "횡단보도 좌표", "crosswalk_position", "횡단보도 다각형", "ped_detector 좌표", "횡단보도 뷰어 표시", "횡단보도 좌표 다시" 요청 시 반드시 이 스킬로 orchestrate. 원천: claude_work_list/crosswalk_position.md, 확정사실: _crosswalk_position_workspace/00_constraints.md. 단순 조회는 직접 응답.
---

# crosswalk-position — 횡단보도 좌표 교체 + 뷰어 표시 (Orchestrator)

`crosswalk_position.md` 의 N개 횡단보도 폴리곤을 `katech_ped_detector.py` 의 EPSG:5179 `crosswalk_data` 로
교체(오프라인 pyproj 변환 후 리터럴)하고 senario mat 뷰어에 다각형 표시. 원천: `_crosswalk_position_workspace/00_constraints.md`.

## 실행 모드
**에이전트 팀(파이프라인)**: analyst → coder → verifier. `Agent` 직접 호출, `model: opus`. 파일 기반 산출(`_crosswalk_position_workspace/`).
※ crosswalk-position-* 에이전트가 레지스트리에 없으면 `general-purpose` 로 실행하되 각 에이전트 .md 의 역할·범위를 프롬프트에 그대로 주입.

## Phase 0: 컨텍스트 확인
- `_crosswalk_position_workspace/` 존재 + 부분 수정 요청 → 해당 에이전트만 재호출.
- 새 요청 → 오케스트레이터가 사전조사(현재 crosswalk_data, 변환 파라미터 일치, 점 개수, 뷰어 경로)로 `00_constraints.md` 작성 + **사용자 결정 확인(범위·뷰어 방식)** 후 analyst.

## Phase 1: 분석 (crosswalk-position-analyst)
`00_constraints.md` 전제 → PART A(.py crosswalk_data 교체) + PART B(뷰어 레이어 추가) 정밀 사양 → `01_spec.md`.

## Phase 2: 구현 (crosswalk-position-coder)
백업 후 PART A: 헬퍼로 WGS84→5179 변환, crosswalk_data 딕셔너리만 교체(py_compile). PART B: var CROSSWALK + 토글 + 범례 추가-온리. → `02_impl.md`.

## Phase 3: 검증 (crosswalk-position-verifier)
py_compile·키/점개수·독립 재변환 대조(<1e-3m)·1·2번 regression·뷰어 9폴리곤/토글·범위밖 무변경·2파일만 → `03_verify.md`. FAIL 시 coder 재호출.

## 확정 불변 (재논의 금지)
- **범위 = 좌표 + 뷰어 + 검증만.** occupancy_msg(crosswalk1·2_occupied)·CAN·HMI·tim-pedes **무변경**. 검출토픽은 순회로 자동 커버.
- 변환 = `pyproj EPSG:4326→5179 always_xy=True`, 입력 (lon,lat). .py 에 런타임 pyproj 의존 추가 금지(리터럴만).
- .py 는 crosswalk_data 딕셔너리 내용만 변경. 뷰어는 추가-온리(기존 DATA/LINEMARK/TYPE5 불변). 원본 백업.
- 수정 파일 = 정확히 2개(katech_ped_detector.py + mat_viewer HTML).

## 에러 핸들링
verifier FAIL → 원인 분류 후 coder 1회 재호출. 재실패면 백업 롤백 + 사용자 보고.

## 테스트 시나리오
- 정상: crosswalk_data 키 {1..9}, 3~9 신규 점 개수 일치, 1·2번 0m regression, 뷰어 9폴리곤 토글.
- 엣지: 비사각형 다각형 ray-casting, 좌표 자릿수, 대용량 HTML 추가-온리 byte diff.

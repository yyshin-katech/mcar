---
name: stopline-adj
description: senario mat 도로링크의 끝을 노면선표시(B2_SURFACELINEMARK) 정지선까지 연장하고 다음 링크를 교차점부터 트림하는 하네스. 웨이포인트/station(길이)/is_stop_line 갱신 + HTML mat 뷰어 반영. "링크 정지선 매칭", "정지선까지 연장", "링크 끝 정지선", "stop_line_adj", "링크 길이 수정 정지선", "mat 정지선 다시" 요청 시 반드시 이 스킬로 orchestrate. 원천: claude_work_list/stop_line_adj_*.md, 확정사실: _stopline_adj_workspace/00_constraints.md. 단순 조회는 직접 응답.
---

# stopline-adj — mat 링크 정지선 연장/트림 (Orchestrator)

senario mat 링크를 지정 정지선(B2_SURFACELINEMARK ID)까지 연장하고 다음 링크를 교차점부터 트림.
웨이포인트/station/is_stop_line 갱신 + HTML 뷰어 반영. 원천: `_stopline_adj_workspace/00_constraints.md`.

## 실행 모드
**에이전트 팀(파이프라인)**: analyst → coder → verifier. `Agent` 직접 호출, `model: opus`. 파일 기반 산출(`_stopline_adj_workspace/`).

## Phase 0: 컨텍스트 확인
- `_stopline_adj_workspace/` 존재 + 부분 수정 요청 → 해당 에이전트만 재호출.
- 새 요청 → 오케스트레이터가 먼저 사전조사(mat/shp 로드, 교차점 계산)로 `00_constraints.md` 작성 후 analyst.

## Phase 1: 분석 (stopline-adj-analyst)
`00_constraints.md` 전제 → 편집 알고리즘·필드 규칙·뷰어 패치·검증계획 정밀화 → `01_spec.md`.
**mat 은 주행 경로/정지선(안전 관련)이라 coder 진행 전 오케스트레이터가 사용자에게 분할점·규칙 확인 권장.**

## Phase 2: 구현 (stopline-adj-coder)
백업 후 6개 mat 편집(L 연장/N 트림/station 재계산/필드 보존) + HTML var DATA 6개 feature 패치. → `02_impl.md`.

## Phase 3: 검증 (stopline-adj-verifier)
연속성·정지선 도달·station 단조·위상/필드 보존·6개 외 무변경·뷰어 반영 → `03_verify.md`. FAIL 시 coder 재호출.

## 확정 불변 (재논의 금지)
- **6개 mat만** 편집. 위상(NEXT/L/R_LINK_ID)·IID/SG/MANUAVER/Speed_Limit 등 불변.
- **공유 정점** L.end==N.start==교차점 P. station 0부터 재계산.
- east/north/station shape (1,N)·키·dtype 원본 유지. is_stop_line: 연장링크=1 / 트림링크=0.
- 원본 백업 필수. 뷰어는 6개 feature만 패치.

## 에러 핸들링
1회 재시도 후 재실패면 그 세트 없이 진행하고 보고서에 누락 명시. 백업으로 롤백 가능.

## 테스트 시나리오
- 정상: L871 끝 → B2209W001421(931005.2,1929866.57) 도달(수직거리~0), 3871 시작=그 점, station 재계산, 위상 불변.
- 엣지: 교차 세그 파라미터 경계, station 마지막 짧은 세그, 6개 외 mat 해시 불변.

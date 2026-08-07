---
name: percept-object-filter-policy
description: "percept_topic_matcher.cpp + rviz_filter.cpp 의 인식 오브젝트 공간 필터 정책(3-zone OR, 14개 cap, 우측 컷)"
metadata: 
  node_type: memory
  type: project
  originSessionId: c2a74392-f1b6-49d8-91ad-e78f7a6568f9
---

CAN 측 perception 오브젝트 선별 필터. `src/sensing/can/src/percept_topic_matcher.cpp` (다운스트림 `track_CAN_writer_*` 슬롯 assign 순서 결정) + 시각화 `rviz_filter.cpp`. 두 파일 ROI 상수 **동일 값 인라인 — 동시 갱신 필수**. percept-filter-tune 하네스(`skills/percept-filter-tune`, agents 3, 워크스페이스 `_filter_workspace/`, 사양 `claude_work_list/object_filter.md`)로 관리.

## 현재 정책 (2026-05-11 전면 개편 이후)
인식 영역 = 3개 OR (자차 ego-frame, x=전방+/후방−, y=좌+/우−):
- **A) 박스**: `|y|≤5 && x∈[-40, 80]` — 자차 옆까지 그대로 포함.
- **B) 전방 측면**: `y>5 && x∈[10, 80]` (좌측만)
- **C) 후방 측면**: `y>5 && x∈[-40, -10]` (좌측만)
- 사각지대: 측면(`y>5`)에서 자차 주위 `x∈(-10,10)`.
- **우측(`y<-5`) 측면 전체 제거** — 측면 조건을 `abs_y>LATERAL` → `curr_y>LATERAL`(좌측만)로. 박스 A(`|y|≤5`)는 영향 없음(우측 5m 이내 포함).

상수: `FRONT_RANGE_M=80`, `REAR_RANGE_M=40`, `LATERAL_RANGE_M=5`, `FRONT_NEAR_X_M=10`, `REAR_NEAR_X_M=10`(측면 cut 전용, 박스 미적용).
정렬: 단일 `(int)sqrt(x²+y²)` ASC → `|y|` ASC. **14개 cap 유지**. sqrt 결과 정렬용 1회만(비교는 제곱).
시각화: `claude_work_list/percept_filter_zones.html`(radial gradient).

## rviz_filter.cpp ROI 마커
`percept_callback` 끝에 마커(`ns="roi"`, ego_frame, lifetime 1s): A=cyan 닫힌 사각형, B 전방측면=orange "ㄷ"자(안쪽 `y=+5` 라인 + 양 끝 2m 외향 tick, 외곽 y=∞ 표현). 우측 측면 마커(id 2,4)는 제거됨.

## 이력(초기→현재, 참고)
초기 `attention_type==1 우선 + priority_id ASC + 14 break` → (05-11a) FRONT(100m)/SIDE 분할 + attention 가중치, `FRONT_RANGE_M=100` → (05-11b) `LATERAL_RANGE_M=5` 컷 추가 → (05-11c) 위 3-zone OR 로 전면 개편(FRONT/SIDE·attention 정렬 폐기). 로그 문구 `Attention=1/0` → `Front/Side`.

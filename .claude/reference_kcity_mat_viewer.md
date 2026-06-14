---
name: reference_kcity_mat_viewer
description: "K_CITY 맵 mat 뷰어 재생성 하네스 위치/사용법 + mat east/north는 row (1,N) 규약"
metadata: 
  node_type: memory
  type: reference
  originSessionId: dcc247e8-3d20-4e27-835f-040773301873
---

K_CITY_20260608 맵(현 운영 맵, [[project_active_branch_map]]) mat 편집·뷰어 재생성.

## 뷰어 재생성 하네스 (senario용과 별개)
`/home/yuyeong/kcity_map/regen_kcity_viewer.py` (repo 외부). senario는 [[senario_mat_viewer]] 참고 — 환경/방식 다름.
- 대상 HTML: `…/mapfiles/K_CITY_20260608/mat_viewer_K_CITY_20260608.html`, Leaflet 기반, `var DATA = {GeoJSON}` **1줄**만 교체(나머지 JS 보존)
- 좌표변환: **pyproj** `Transformer.from_crs(5179,4326)` (이 kcity_map 환경엔 pyproj 설치됨; senario 쪽은 cs2cs CLI)
- 스크립트는 **append-only**(`NEW_LINKS`만 추가)지만 `if __name__` 가드 있어 import 가능. `build_feature(num, {})` 재사용 → 특정 링크 feature를 제거 후 재생성하면 **교체** 가능(79/83 트림 반영 시 이 방식 사용). NEXT_LINK_ID=0이면 existing_starts 빈 dict로 충분
- build_feature는 `np.asarray(m['east']).ravel()` → mat shape 무관하게 좌표 추출(shape만 바꾸면 뷰어 재생성 불필요)

## mat 길이/shape 규약
- 길이 의존 배열은 `east`,`north`,`station` 3개뿐. 별도 length 메타 필드 없음 → station 끝값이 곧 길이
- **east/north는 row (1,N)이어야 함**: `to_control_team_demo.py`가 `target_roads[i]['east'][0]`로 접근. column (N,1)이면 [0]이 1점만 반환 → 매칭 오동작. 2026-06-10 link_81/82/83이 column이라 row로 통일함
- 트림: `~/temp/harness_to_control_team/apply_20260518_update.py`의 `trim_link(lid,'front'|'back',m)`은 **정확히 Nm 보간 트림**. 단순 점절삭(station≤L-N 점만 유지, 보간X)은 점간격(~2m)만큼 오차 — 2026-06-10 link_79/83 끝40m는 단순절삭 선택(실제 ~41.98m 제거)

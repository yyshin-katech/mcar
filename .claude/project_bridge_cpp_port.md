---
name: web_hmi threejs bridge cpp 포팅
description: web_hmi_threejs_bridge.py 의 /hmi/threejs/tracks 발행을 C++ 노드로 분리 — Python 콜백 지연(0.78 Hz)을 10 Hz 로 회복한 패턴/함정 메모.
type: project
originSessionId: e1d57818-0ba3-42fc-8e4d-2f895e224b6e
---
# web_hmi threejs bridge cpp 포팅 (2026-05-12, siheung_dev)

## 배경 / 측정값
- `/percept_topic` 10 Hz 입력에 대해 Python `web_hmi_threejs_bridge.py` 의 `/hmi/threejs/tracks` 발행이 0.74–0.78 Hz 까지 떨어져 web_hmi 트랙 박스가 1초마다 점프.
- 페이로드 크기 47–89 KB (TRACKS_MAX_RENDERED=6 + cap 256 이미 적용된 상태) → rosbridge JSON throttle 시나리오 아님.
- 시그니처: `queue_size=1` + 콜백 처리 시간 > 100ms → 9 msg/sec 드롭. 원인은 Python ROS msg deserialize + `cloud_indices[k].data` 멤버 access 비용 누적.

## 채택 옵션
- **옵션 2 (분리 포팅)**: tracks 발행만 신규 C++ 노드 `web_hmi_threejs_tracks_cpp` 로 옮김. shapefile → `/hmi/threejs/map` latched 발행은 Python 잔존 (GDAL/OGR/PROJ 도입 회피).
- 옵션 1 (전체 포팅) 은 빌드 표면 큼 — 성능 이슈가 tracks 한정이므로 보류.

## 결과
- `catkin_make --pkg web_hmi` PASS → `devel/lib/web_hmi/web_hmi_threejs_tracks_cpp` (1.06 MB).
- `/hmi/threejs/tracks` hz: **0.78 → 9.98 Hz** (~13×), bw 51 KB/msg × 10 Hz = 503 KB/s.
- Publisher 단일 (cpp 노드만), Python 측 `Subscriptions: None` 으로 가드 정상.

## 핵심 패턴
- **JSON lib**: jsoncpp (`libjsoncpp-dev` 시스템 패키지), `StreamWriterBuilder` indentation="" 로 압축. nlohmann/json vendoring 회피.
- **PointCloud2 zero-copy**: `ConstPtr` 보관 + `reinterpret_cast<const float*>(msg->data.data())` + `point_step/sizeof(float)` 스트라이드. 복사 없이 인덱싱.
- **cloud_indices 비용 차이**: Python 에서 dominant cost (`std_msgs/Int32 .data` attribute access) 였던 부분이 C++ 직접 멤버 access 로 사실상 0. cap 먼저(`indices[:256]`) 원칙은 유지 (페이로드 크기 한도용).
- **정렬·슬라이싱 순서 (불변)**: confidence 컷 → candidate 수집(메타데이터만) → `std::sort` dist² ASC (cap 없음, 초기엔 `partial_sort` top 6 였으나 2026-05-12 cap 제거) → 생존자만 `slicePoints` (점군 cap 256 먼저) → JSON. 점군 슬라이싱은 정렬 후에 — 자르기 전에 슬라이싱하면 99% 낭비.
- **transport hints**: 3개 구독 모두 `transport_hints().tcpNoDelay()`.
- **단일 스레드 `ros::spin()`**: mutex 불필요. 멀티스레드 spinner 도입 시 cloud/percept/ego 캐시 보호 필수.

## dual-publisher 회피 패턴 (외과적 분리 일반화)
- 같은 토픽을 두 노드가 advertise 하면 마지막 advertise 가 이김 + queue 충돌 — **반드시 한쪽 비활성화**.
- 권장: 기존 Python 노드에 `~publish_tracks` ROS param (기본 False) 가드를 추가. False 면 `_pub_tracks` advertise + 관련 구독(`_sub_local`/`_sub_cloud`/`_sub_percept`) + 로그 타이머 전부 스킵. 다른 책임(map latched 발행)은 그대로 유지.
- launch 에서 Python 노드에 명시적으로 `<param name="publish_tracks" value="false"/>` 주입, 옆에 cpp 노드 `<node>` 블록 추가. variant 가드(`if="$(eval arg('variant') in ('threejs', 'threejs_f1'))"`)로 사용 안 하는 variant 에서는 cpp 도 안 띄움.
- 검증 순서: (1) `rostopic info <topic>` Publishers 단일 확인 → (2) Python 노드 `rosnode info` 의 Subscriptions 가 비었는지 → (3) `rostopic hz` 측정.

## TRACKS_MAX_RENDERED=6 cap 제거 (2026-05-12 후속)
- cpp 포팅으로 콜백 여유가 충분해진 뒤 사용자 요청으로 일괄 제거.
- `web_hmi_threejs_tracks_node.cpp`: 상수 + `if (size > 6) partial_sort/resize else sort` 분기 → 단순 `std::sort(dist² ASC)`. 주석 동기화.
- `web_hmi_threejs_bridge.py`: 동일 상수 + `candidates[:TRACKS_MAX_RENDERED]` 라인 제거 (가드 비활성이지만 일관성).
- `web_hmi_bridge.py`: `OBJECTS_MAX=6` + cap 분기 제거. `_publish_objects` 정렬은 유지 (가까운 객체 먼저 발행).
- 상한 책임: `percept_topic_matcher.cpp`의 `max_objects_to_publish=14`. web_hmi 측은 더 이상 cap 안 함.
- 점군 cap 256은 유지 (페이로드 크기 안정성).
- 라이브 적용은 launch 재기동 필요 (`web_hmi.launch`의 cpp 노드 `required="true"`).
- `TrackBoxes.jsx`의 `TRACK_MISS_GRACE=4`는 cap 6 가정 하에서 줄인 값이라 cap 풀린 뒤엔 더 길게 둬도 무방 — 회귀 보이면 그 쪽 우선 점검.

## 메시지 오타 필드 (perception_ros_msg)
포팅 시 반드시 그대로 유지:
- `CoreInfo.trakcer_id` (sic — tracker 아님)
- `Object.hassupplmentinfo` (sic — supplement 아님)
- `SupplementInfo.cloud_indices[k].data` — Int32 객체 배열, 각 원소 `.data` access

## 페이로드 키 (TrackBoxes.jsx 호환 필수)
```
{ stamp, tracks: [{ id, tid, type, x, y, vx, vy, size_x, size_y, orientation, confidence, points? }], ego_at_emit?: { east, north, yaw } }
```
- `type`: `1 → "pedestrian"`, else `"car"`.
- `points`: cloud_indices 슬라이싱 결과가 비지 않을 때만 키 추가, mm round (`std::round(v*1000)/1000`).
- `ego_at_emit`: `/localization/to_control_team` 한 번이라도 수신했을 때만.
- `orientation = atan2(direction.y, direction.x)` (순서 주의 — atan2(sin, cos)).

## 하네스 / 변경 파일
- 하네스: `.claude/skills/bridge-cpp-port/` + agents 3 (bridge-port-analyst/coder/verifier). 워크스페이스 `_bridge_cpp_workspace/`.
- 변경 파일:
  - `src/visualization/web_hmi/src/web_hmi_threejs_tracks_node.cpp` (신규 339 lines)
  - `src/visualization/web_hmi/CMakeLists.txt` (roscpp/sensor_msgs/perception_ros_msg/mmc_msgs + jsoncpp + add_executable)
  - `src/visualization/web_hmi/package.xml` (build/exec depend 추가)
  - `src/visualization/web_hmi/launch/web_hmi.launch` (Python 노드에 publish_tracks=false 주입 + cpp `<node>` 블록 추가)
  - `src/visualization/web_hmi/scripts/web_hmi_threejs_bridge.py` (publish_tracks 가드 추가, 다른 로직 미수정)

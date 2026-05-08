# launch-auditor 보고서

## 요약
- 점검 파일:
  - `src/visualization/web_hmi/launch/web_hmi.launch` (76 줄)
  - `src/visualization/web_hmi/scripts/web_hmi_bridge.py` (295 줄)
  - `src/visualization/web_hmi/scripts/web_hmi_threejs_bridge.py` (380 줄)
  - `src/visualization/web_hmi/scripts/web_server.py` (50 줄)
  - `src/visualization/web_hmi/scripts/open_browser.py` (39 줄)
  - `src/visualization/web_hmi/scripts/_demo_publisher.py` (111 줄)
  - `src/visualization/web_hmi/CMakeLists.txt`, `package.xml`
- 발견 0건: critical 0, major 0, minor 0, info 0

## 발견 항목

발견 없음. launch 파일과 스크립트, 외부 경로 모두 정합성 확인.

## 검증한 항목 (이상 없음)

### A. launch 파일 자체
- `<arg name="variant" default="threejs_f1">` 의 default 값이 `web/index_threejs_f1.html` 로 매핑되어 실재 (`web_hmi.launch:22-23`).
- `$(eval {...}.get(arg('variant'), 'index.html'))` page 매핑(launch:23) 4개 키 → 모두 실파일 존재:
  - `f1` → `web/index_f1.html` (실재)
  - `default` → `web/index.html` (실재)
  - `threejs` → `web/index_threejs.html` (실재)
  - `threejs_f1` → `web/index_threejs_f1.html` (실재)
  - 알 수 없는 variant → `index.html` 로 fallback (안전).
- `<node>` 4개 모두 `scripts/`에 실제 파일 존재 + `catkin_install_python` 등록됨 (`CMakeLists.txt:11-18`):
  - `web_hmi_bridge` → `web_hmi_bridge.py` (실재, 실행권한 +x)
  - `web_hmi_threejs_bridge` → `web_hmi_threejs_bridge.py` (실재, 실행권한 +x)
  - `web_hmi_server` → `web_server.py` (실재, 실행권한 +x)
  - `web_hmi_open_browser` → `open_browser.py` (실재, 실행권한 +x)
- `_demo_publisher.py`는 launch에 등록되지 않으나 docstring 상 `rosrun web_hmi _demo_publisher.py`로 호출하는 디버깅용으로 의도된 누락.
- `<node>` 노드 이름과 스크립트 `rospy.init_node(...)` 일치 (launch는 `__name:=` 으로 override 하므로 큰 영향 없음, 그래도 모두 일치):
  - `web_hmi_bridge` ↔ `init_node('web_hmi_bridge')` (`web_hmi_bridge.py:282`)
  - `web_hmi_threejs_bridge` ↔ `init_node("web_hmi_threejs_bridge")` (`web_hmi_threejs_bridge.py:373`)
  - `web_hmi_server` ↔ `init_node('web_hmi_server')` (`web_server.py:17`)
  - `web_hmi_open_browser` ↔ `init_node('web_hmi_open_browser', anonymous=True)` (`open_browser.py:13`, anonymous라 launch가 정한 이름 사용됨 — 정상)
- `<node if="$(eval ... in ('threejs','threejs_f1'))">`로 threejs bridge가 조건부 실행 — `default`/`f1` variant에서는 `/hmi/threejs/*` 토픽이 발행되지 않음. variant 전제와 일치.
- 토픽 발행 중복 없음:
  - `/hmi/state`, `/hmi/diagnostics`, `/hmi/objects`, `/hmi/popup`, `/hmi/traffic`, `/hmi/bag`, `/hmi/topic_hz`, `/hmi/map`은 `web_hmi_bridge.py`만 발행.
  - `/hmi/threejs/map`, `/hmi/threejs/tracks`는 `web_hmi_threejs_bridge.py`만 발행.
  - `_demo_publisher.py`도 `/hmi/state` 등을 발행하나 launch에 미등록이라 동시 실행되지 않음. 사용자가 수동 `rosrun`으로 띄우는 경우만 충돌이 가능 — 의도된 디버그용 시나리오.

### B. 노드 ↔ 파라미터 정합성
- `web_hmi_bridge`:
  - `<param name="bag_dir">` (launch:42) ↔ `rospy.get_param('~bag_dir', None)` (`web_hmi_bridge.py:112`) — 일치.
  - `<param name="map_shp">` (launch:43) ↔ `rospy.get_param('~map_shp', '')` (`web_hmi_bridge.py:239`) — 일치.
- `web_hmi_threejs_bridge`:
  - `<param name="mapdir">` (launch:53) ↔ `rospy.get_param("~mapdir", ...)` (`web_hmi_threejs_bridge.py:88-89`) — 일치 (launch arg는 `threejs_mapdir`이고 노드 param 은 `mapdir`로 의도적 단축).
- `web_hmi_server`:
  - `<param name="port">` (launch:62) ↔ `rospy.get_param('~port', 8088)` (`web_server.py:19`) — 일치.
  - `<param name="web_dir">` (launch:63) ↔ `rospy.get_param('~web_dir', None)` (`web_server.py:20`) — 일치.
  - `<param name="page">` (launch:64) ↔ `rospy.get_param('~page', '')` (`web_server.py:21`) — 일치.
- `web_hmi_open_browser`:
  - `<param name="port">` (launch:73) ↔ `rospy.get_param('~port', 8088)` (`open_browser.py:14`) — 일치.
  - `<param name="page">` (launch:74) ↔ `rospy.get_param('~page', 'index.html')` (`open_browser.py:15`) — 일치.

### C. 외부 의존
- `$(find rosbridge_server)/launch/rosbridge_websocket.launch` — `/opt/ros/noetic/share/rosbridge_server` 존재 (rospack find 통과).
- `$(find gps_system_localizer)/src/A2_LINK_epsg5179.shp` (launch:25) — 실재 (`/home/katech/mcar_v13/src/localization/gps_system_localizer/src/A2_LINK_epsg5179.shp`).
- `$(find gps_system_localizer)/mapfiles/K_CITY_2025` (launch:27) — 실재 (디렉토리).
- `$(find pyqt_hmi)/scripts` (launch:41 — PYTHONPATH env) — 실재. `BaseHmiStateController` 정의 파일도 실존 (`pyqt_hmi/scripts/utils/hmi_state.py`).
- `$(find web_hmi)/web` (launch:63) — 실재.
- `$(env HOME)/bag_data` (launch:17) — 현재 환경 `/home/katech/bag_data` 실재 (다른 환경은 없을 수 있으나 `<param>`에 빈 문자열 fallback 처리 — `web_hmi_bridge.py:113` `if bag_dir:` 가드).

### D. 부가 검증
- `package.xml`의 `exec_depend`(`pyqt_hmi`, `mmc_msgs`, `katech_custom_msgs`, `katech_diagnostic_msgs`, `v2x_msgs`, `perception_ros_msg`, `rosbridge_server`) 모두 워크스페이스 또는 시스템에 존재.
- `catkin_install_python`에 5개 스크립트 모두 등록 (`CMakeLists.txt:11-18`) → `devel/lib/web_hmi/`에 5개 모두 설치 확인.
- launch 의 `<arg>`별 default가 실제 사용처와 모두 연결되어 있음 (잉여 arg 없음).

# Web-based Ioniq5 HMI 구현 계획 (rosbridge + React CDN)

## Context

`claude_work_list/hmi_design/new/` 폴더에 새로운 HMI 디자인이 React/JSX 기반의 정적 HTML 시안 형태로 들어와 있다. 진입점은 `Ioniq5 HMI.html` — React 18.3.1 + ReactDOM + @babel/standalone 7.29.0 을 모두 unpkg CDN 으로 로드해 `<script type="text/babel">` 으로 `hmi/components.jsx`, `hmi/HMIScreen.jsx` 를 직접 실행한다 (빌드 없음). 디자인은 1600×900 고정, 좌패널 320px / 무한 스테이지 / 우패널 360px CSS Grid, Inter + JetBrains Mono + Pretendard 폰트.

이미 `pyqt_hmi` 패키지에 두 진입점(`main_display.py` 레거시, `main_display_a1.py` A-1) 이 운영 중이다. A-1 작업으로 추출된 `utils/hmi_state.py` (`HmiStateController(QObject)`) 가 16개 토픽을 구독하여 16개 pyqtSignal 로 노출한다 — 진단 디바운스, 신호등 매칭, AEB/ODD 팝업, rosbag 서브프로세스 등 검증된 로직 포함.

목표: 디자인 시안을 그대로 살리되 ROS 데이터에 결합된 **실시간 차량 HMI** 로 동작시키는 웹 앱을 추가한다. 기존 `pyqt_hmi` 는 손대지 않고 신규 `web_hmi` 패키지로 병행한다. 사용자가 차량 PC 에서 `roslaunch web_hmi web_hmi.launch` 한 번으로 브라우저까지 띄워 동시 검증할 수 있어야 한다.

---

## 1. 기술 선택

### 빌드 vs 노빌드

**채택: 노빌드 (시안 구조 유지)**. 시안이 이미 CDN + Babel-in-browser 로 작성되어 있고, 이를 보존하면 디자이너가 만든 JSX 를 거의 그대로 들고와 ROS 와이어업만 추가하면 된다. Vite/Webpack 도입 시 차량 PC 에 Node 22 + npm + 빌드 캐시 정책이 추가되어 운영 부담만 늘어난다.

리스크 완화: 차량 PC 가 항상 인터넷 연결되지 않을 수 있다 → CDN 자산 (React 3개 + roslib + 폰트) 을 `web/vendor/` 에 **체크인**하고 `<script src="vendor/...">` 로 변경. 약 3.5 MB. 외부 네트워크 의존 0.

### ROS ↔ Browser 브리지

**채택: rosbridge_suite + roslibjs + Python 어그리게이터 노드 (옵션 b)**.

| 옵션 | 평가 |
|---|---|
| (a) roslibjs 가 raw ROS 토픽 직접 구독 | 16개 토픽 + 커스텀 메시지 (`v_can_msg`, `to_control_team_from_local_msg` 등) JS 측 unpack/매핑 코드 중복. 진단 디바운스·팝업 우선순위 같은 결정 로직을 JS 로 다시 구현해야 함. |
| **(b) Python 브리지 노드가 도메인 토픽으로 재발행** | 기존 `HmiStateController` 의 결정 로직(진단 miss-cnt, popup priority, mode pulse, traffic light 매칭) 100% 재사용. 웹 클라이언트는 `/hmi/state`, `/hmi/diagnostics`, `/hmi/popup`, `/hmi/objects`, `/hmi/traffic` 5개 JSON 토픽만 구독. 메시지 스키마 변경에 클라이언트 영향 0. |
| (c) Custom WebSocket 서버 | rosbridge 의 검증된 인프라 버림. 비추천. |

옵션 b 의 부산물: `HmiStateController` 의 Qt 의존성을 제거한 `BaseHmiStateController` 가 만들어지면 향후 PyQt/Web 양쪽이 같은 컨트롤러를 공유하게 된다.

---

## 2. 패키지 전략 — 신규 사이블링 패키지

세 옵션 중 **(c) `src/visualization/web_hmi/` 신규 패키지**를 채택.

- (a) `pyqt_hmi` 안에 추가 — 패키지 책임이 흐려짐 (Qt + Web 혼재).
- (b) 기존 `pyqt_hmi` 를 web 으로 교체 — 검증된 PyQt 진입점 두 개를 위협. 비합리.
- **(c) 사이블링** — `pyqt_hmi` 와 코드/launch 격리. 컨트롤러는 import 경로(`from pyqt_hmi.scripts.utils.hmi_state import …`) 로 공유. PR 단위 경계가 깔끔.

`web_hmi` 의 빌드 의존: `rospy`, `std_msgs`, `pyqt_hmi` (controller import), 그리고 `pyqt_hmi` 가 의존하는 모든 커스텀 메시지 패키지 (transitive 로 자동 해결).

---

## 3. 파일 단위 계획 (절대 경로)

### Phase 1 — `pyqt_hmi` 컨트롤러 리팩토링 (Qt 의존성 분리)

| 파일 | 동작 | 비고 |
|---|---|---|
| `/home/sim/mcar/src/visualization/pyqt_hmi/scripts/utils/hmi_state.py` | **수정** | `BaseHmiStateController` 신규 클래스 — 모든 ROS 콜백/디바운스/popup/bag 로직 보유. emit 호출은 `_emit(name, *args)` 추상 메서드로 분리. `HmiStateController(BaseHmiStateController, QObject)` 는 `pyqtSignal` 16개 + `_emit` 구현(`getattr(self, name).emit(*args)`)만 담당. **시그널 시그니처/동작 변경 0** — 기존 `MainWindowA1._wire_controller` 영향 없음. |

### Phase 2 — `web_hmi` 패키지 골격

| 파일 | 동작 | 비고 |
|---|---|---|
| `/home/sim/mcar/src/visualization/web_hmi/package.xml` | 신규 | `<exec_depend>` 로 `rosbridge_server`, `pyqt_hmi`, `std_msgs`, `mmc_msgs`, `katech_custom_msgs`, `katech_diagnostic_msgs`, `v2x_msgs`, `perception_ros_msg` 명시. |
| `/home/sim/mcar/src/visualization/web_hmi/CMakeLists.txt` | 신규 | `catkin_package()`, `catkin_install_python(PROGRAMS scripts/web_hmi_bridge.py …)`, `install(DIRECTORY launch web …)`. |
| `/home/sim/mcar/src/visualization/web_hmi/launch/web_hmi.launch` | 신규 | (1) `rosbridge_websocket` (`<include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch">` port=9090) + (2) `web_hmi_bridge` 노드 + (3) Python `http.server` 노드 (포트 8088, `web/` 정적 서빙) + (4) optional `xdg-open http://localhost:8088` 브라우저 자동 오픈. `open_browser` arg 로 토글 가능. |

### Phase 3 — 브리지 노드 (Python → JSON 어그리게이터)

| 파일 | 동작 |
|---|---|
| `/home/sim/mcar/src/visualization/web_hmi/scripts/web_hmi_bridge.py` | 신규. `rospy.init_node('web_hmi_bridge')`. `BaseHmiStateController` 인스턴스화하되 emit 구현은 dict 누적 + 100 ms `rospy.Timer` 로 5개 토픽에 JSON publish. 인바운드: `/hmi/cmd/mode_request` (`std_msgs/Bool`), `/hmi/cmd/bag_toggle` (`std_msgs/Empty`) → 컨트롤러의 `request_mode`, `toggle_bag` 호출. |

발행 토픽 (모두 `std_msgs/String`, payload = JSON):

```
/hmi/state         # 100 ms — speed, gear, mode, aeb, steering, ego{e,n,yaw}, gps{rtk,std}, link, lane, odd, road_state, speed_limit
/hmi/diagnostics   # 100 ms — {gps:0|1|2, adcu:..., …, hz:{gps:9.8, adcu:50.1, …}}
/hmi/popup         # 변경 시 — {text, severity}
/hmi/objects       # 30 Hz cap — [{id,x,y,vx,vy,width,length,orientation,type}, …]
/hmi/traffic       # 변경 시 — {color:0..3, time_decisec, look_at:{intersection,signal_group}}
/hmi/bag           # 변경 시 — {recording, info}
```

이유: 단일 토픽으로 집계하면 roslibjs 측에서 Subscriber 6개로 끝. 토픽 16개 + 커스텀 메시지 unpack 보다 압도적으로 단순.

### Phase 4 — 정적 웹 자산 (시안 임포트 + ROS 와이어업)

`web/` 트리는 시안의 `claude_work_list/hmi_design/new/` 와 1:1 대응시키고, 그 위에 ROS bridge 클라이언트만 얹는다.

| 파일 | 동작 |
|---|---|
| `/home/sim/mcar/src/visualization/web_hmi/web/index.html` | 신규. 시안 `Ioniq5 HMI.html` 를 베이스로, 폰트 link 를 `vendor/fonts/...` 로, React/Babel/roslib script src 를 `vendor/` 로 변경. `<script type="text/babel" src="hmi/ros_bridge.jsx">` + `<script type="text/babel" src="hmi/components.jsx">` + `<script type="text/babel" src="hmi/HMIScreen.jsx">` + 진입 스크립트(`<RosProvider><HMIScreen /></RosProvider>`). |
| `/home/sim/mcar/src/visualization/web_hmi/web/hmi/styles.css` | 신규 (시안 `hmi/styles.css` **그대로 복사**). |
| `/home/sim/mcar/src/visualization/web_hmi/web/hmi/components.jsx` | 신규 (시안 `hmi/components.jsx` **그대로 복사** — SpeedGauge, Vehicle, VehicleTop, RangeRings, RangeRingsTop, DetectionOverlay, DetectionOverlayTop, PlanPath, TrafficObjects 모두 SVG 컴포넌트로 props만 받는 순수 함수). |
| `/home/sim/mcar/src/visualization/web_hmi/web/hmi/HMIScreen.jsx` | 신규. 시안의 `HMIScreen` 을 베이스로, hardcoded prop (speed=47, driveMode="D" …) 를 `useRosState()` 훅의 반환값으로 교체. **레이아웃 / 클래스명 / SVG 구조 전혀 변경 없음** — 시안의 비주얼은 그대로. |
| `/home/sim/mcar/src/visualization/web_hmi/web/hmi/ros_bridge.jsx` | 신규. `RosProvider` (Context) + `useRosState()`/`useDiagnostics()`/`useObjects()`/`usePopup()`/`useTraffic()`/`useBag()` 훅. roslib `Topic` 구독, JSON.parse, React state 갱신. `publishModeRequest(b)`, `publishBagToggle()` 액션. 재연결 백오프(1s → 30s 지수) 포함. |
| `/home/sim/mcar/src/visualization/web_hmi/web/vendor/react.production.min.js` | 신규 (UMD 18.3.1, 체크인). |
| `/home/sim/mcar/src/visualization/web_hmi/web/vendor/react-dom.production.min.js` | 신규 (UMD 18.3.1, 체크인). |
| `/home/sim/mcar/src/visualization/web_hmi/web/vendor/babel.min.js` | 신규 (@babel/standalone 7.29.0, 체크인). |
| `/home/sim/mcar/src/visualization/web_hmi/web/vendor/roslib.min.js` | 신규 (roslib 1.4.1 UMD, 체크인). |
| `/home/sim/mcar/src/visualization/web_hmi/web/vendor/fonts/{Inter,JetBrains-Mono,Pretendard}/*.woff2` | 신규 (오프라인 폰트, ~2 MB). `styles.css` 에서 `@font-face` 로 등록. |

### Phase 5 — 검증 보조

| 파일 | 동작 |
|---|---|
| `/home/sim/mcar/src/visualization/web_hmi/scripts/_demo_publisher.py` | 신규 (선택). roscore 만 띄우고 5개 `/hmi/*` 토픽에 페이크 JSON publish. 브라우저 단독 디버그용. |

---

## 4. 토픽 매핑 — HMIScreen prop / 섹션 → ROS 토픽

시안의 `HMIScreen` 시그니처와 하드코딩된 우패널 더미 데이터를 실제 데이터 소스에 매핑한다.

| HMIScreen 위치 | 데이터 | 출처 |
|---|---|---|
| `<SpeedGauge speed={…}>` | km/h | `BaseHmiStateController.current_speed` (`/sensors/v_can` wheel_speed 평균 ×3.6) → `/hmi/state.speed` |
| `<SpeedGauge mode="ECO">` 자리 | "AUTO" / "MANUAL" / "AEB" | `autonomous_mode` + `aeb_flag` → `/hmi/state.mode`, `.aeb` |
| `driveMode` (P/R/N/D) | gear | `/sensors/v_can.gear_status` (1/2/3/4) → `/hmi/state.gear` |
| 좌패널 "Powertrain" 4 stat | Battery/Range/Motor/Pack V | **현재 토픽 미존재 → 우선 디자인 자리 유지, 값은 "—"**. 차후 BMS/EV 토픽 합류 시 연결. (이번 PR 범위 밖) |
| 좌패널 "Heading · GNSS" Lat/Lon | WGS84 | `/sensors/cpt7_gps.lat`, `.lon` 가 직접 노출되지 않음. **EPSG:5179 host_east/north → WGS84 변환은 controller 내 `pyproj`로 추가** (또는 자리 유지하고 host_east/north 표시). |
| 스테이지 `<SpeedGauge>` 옆 표시 영역 | speed limit | `/localization/to_control_team.Speed_Limit` → `/hmi/state.speed_limit` |
| 스테이지 `<RangeRings>` / 거리감 | (시각 자산만) | 데이터 없이 정적 SVG. 그대로. |
| 스테이지 `<DetectionOverlay>` | (시각 자산만) | 정적 SVG. 그대로. |
| 스테이지 `<PlanPath>` (topView 시) | (시각 자산만) | 향후 경로 토픽 합류 시 연결. 이번엔 정적. |
| 스테이지 `<TrafficObjects>` (topView 시) | 트랙된 객체 박스 | `/track_Multi_RS` → `/hmi/objects` → `useObjects()` → 컴포넌트 props 로 [{x, y, vx, vy, type}] 전달. **컴포넌트는 props 받도록 수정 필요** (현재 시안은 하드코딩). |
| 스테이지 ego 차량 isometric/topView 토글 | view 모드 | UI state (로컬). |
| 스테이지 bag-banner | 녹화 상태 | `/hmi/bag.recording`, `.info` |
| 우패널 "Mission · Objectives" | 임무 체크리스트 | **데이터 소스 없음** — 디자인 자리 유지, 정적 텍스트. |
| 우패널 "Sensors" 6행 | 토픽명 + Hz + LED | `/hmi/diagnostics` (기존 9개 진단 키 + Hz). 디자인의 6행은 9행으로 확장 (`gps/adcu/lidar/radar/v2x/hmi/vcu/cam/ipc`). |
| 우패널 "Disk · Storage" | 디스크 사용량 | **데이터 소스 없음** — 정적 자리 유지. |
| 하단 Active Alert | 팝업 텍스트 + 한글 | `/hmi/popup.text`, `.severity` (기존 popup priority 그대로). |
| 하단 STOP REC / MARK / NEW WAYPOINT 버튼 | 액션 | STOP REC → `/hmi/cmd/bag_toggle`. 나머지는 이번 PR 에선 noop (디자인 자리 유지). |
| Topbar 시계 | KST 시각 | 클라이언트 `Date()`. |
| Topbar "ROS 1 NOETIC · ONLINE" 칩 | 연결 상태 | rosbridge `connection` 이벤트 → "ONLINE" / "OFFLINE". |
| Topbar "AUTONOMY L2+" 칩 | autonomous_mode | `/hmi/state.mode` 가 1 일 때 amber dot, 아니면 회색. |
| Topbar CPU/MEM | 시스템 모니터 | **이번 PR 범위 밖**. 정적. (선택: `psutil` 으로 브리지에서 publish 가능 — 추가 작업 항목으로 분리.) |

---

## 5. 재사용 vs 재구축

| 자산 | 결정 | 사유 |
|---|---|---|
| `pyqt_hmi.utils.hmi_state.HmiStateController` 의 ROS 콜백 / 진단 디바운스 / popup / bag 로직 | **REUSE (Base 추출)** | 의미 변경 0. PyQt + Web 양쪽이 같은 brain 공유. |
| 시안 `hmi/components.jsx` (SVG 컴포넌트 11개) | **COPY 그대로** | 순수 SVG 함수 컴포넌트. 시각 회귀 0. `TrafficObjects` 만 props 수용 가능하도록 4줄 수정. |
| 시안 `hmi/styles.css` (608 lines) | **COPY 그대로** | CSS custom property + grid layout. 빈틈 없이 그대로. |
| 시안 `HMIScreen.jsx` 의 prop 시그니처 (speed, driveMode, …) | **REUSE** | 하드코딩 default 만 `useRosState()` 로 치환. 디자이너가 시안 미리보기로 계속 쓸 수 있게 default 값은 살림. |
| `pyqt_hmi/launch/hmi_a1.launch` | **참조만** | rosparam (`map_file`, `bag_dir`) 명세 모방. |
| rosbridge_suite | **NEW DEP** | apt: `ros-noetic-rosbridge-suite`. 인스톨 1회. |
| roslibjs | **VENDOR** | UMD min 약 200 KB. CDN 사용 가능하지만 차량 PC 오프라인 가정 시 vendor 권장. |

---

## 6. 위험 / 미해결 항목

1. **rosbridge 미설치** — `sudo apt install ros-noetic-rosbridge-suite` 필요 (사용자 비밀번호: 1). launch 파일이 `<rosbridge_websocket>` include 시 패키지 부재면 즉시 실패하므로 사용자 안내 필수.
2. **CDN 차단 환경** — `vendor/` 체크인이 해법. 단 `vendor/fonts/Pretendard*.woff2` 라이선스 (OFL)는 LICENSE 파일 첨부 권장.
3. **WSLg + 브라우저** — `xdg-open` 이 Windows 측 브라우저를 띄우는지 (wslu 의 `xdg-open`) 확인 필요. 안 되면 launch arg 로 토글하고 사용자가 수동으로 `http://localhost:8088` 접속.
4. **JSON 직렬화 비용** — `/hmi/objects` 가 30 Hz × 객체 N개 × JSON encode/decode. 측정 후 100 Hz roll 이 부담되면 25 Hz 로 강등 가능. roslibjs throttle 옵션도 사용 가능.
5. **`TrafficObjects` 컴포넌트의 데이터 결합** — 시안에서는 SVG 좌표가 하드코딩됨. 실제 객체 좌표 (ego frame, m 단위) → 스테이지 픽셀 좌표 변환이 필요. 변환 함수는 `useObjects()` 내부에 구현. 거리 링 100m = 시안 SVG 의 가장 큰 원 반지름과 매칭.
6. **map(shapefile) 렌더링** — pyqt_hmi 의 차선 polyline 렌더는 PyQt QPainter. 웹에서 shapefile 을 렌더하려면 (a) pyshp + GeoJSON 변환 후 클라이언트 push (큰 파일), (b) 캔버스 PNG 프리렌더 후 정적 이미지로 사용, (c) **이번 PR 에서는 생략** 중 택일. **기본은 (c)** — 시안에 맵 표시가 없으므로 자연스러움. 차후 PR.
7. **시계 동기화** — Topbar clock 은 클라이언트 시계. 차량 PC 와 운영자 노트북이 다른 시간대일 수 있음. ROS time 으로 변경 가능하나 이번엔 클라이언트 시계 유지.
8. **여러 클라이언트 동시 접속** — rosbridge 는 N:1 multi-client 지원. mode_request 동시 발행 시 마지막 승. UX 상 문제는 안 되지만 향후 락 검토.
9. **Babel-in-browser 비용** — 첫 로드 약 200 ms 컴파일. 차량 PC 에서 한 번 로드 후 변경 없으니 무관. 단 `babel.min.js` 1.5 MB → vendor 체크인 사이즈에 영향.
10. **스크린 해상도** — 시안은 1600×900 고정. 차량 PC 모니터가 다른 해상도면 `transform: scale(...)` 으로 fit. 또는 CSS `vw/vh` 로 리스케일. 이번 PR 에서는 시안 그대로 1600×900 + 외곽 검은 배경 매트.

---

## 7. 검증 계획

빌드/실행 환경: WSL2 Ubuntu 20.04 + ROS Noetic + Chrome (Windows side via WSLg).

1. **사전 의존성 설치**
   ```
   sudo apt update && sudo apt install -y ros-noetic-rosbridge-suite
   ```
2. **임포트 체크**
   ```
   cd /home/sim/mcar/src/visualization/web_hmi/scripts
   python3 -c "from pyqt_hmi.scripts.utils.hmi_state import BaseHmiStateController; print('ok')"
   python3 -c "import web_hmi_bridge; print('ok')"
   ```
3. **빌드**
   ```
   cd /home/sim/mcar && catkin_make --pkg web_hmi
   source devel/setup.bash
   ```
4. **단독 브라우저 데모** (roscore + 페이크 publisher)
   ```
   roscore &
   rosrun web_hmi _demo_publisher.py &
   python3 -m http.server 8088 -d /home/sim/mcar/src/visualization/web_hmi/web
   # → Chrome localhost:8088 — 게이지 0→80 순환, P/R/N/D 토글, 객체 3개 공전 확인
   ```
5. **rosbridge 연결 테스트**
   ```
   roslaunch web_hmi web_hmi.launch open_browser:=false
   wscat -c ws://localhost:9090
   > {"op":"subscribe","topic":"/hmi/state","type":"std_msgs/String"}
   # → 100 ms 마다 JSON 도착 확인
   ```
6. **빈 버스 통합 실행**
   ```
   roscore &
   roslaunch web_hmi web_hmi.launch
   ```
   - 브라우저 자동 오픈, "ROS · ONLINE" 칩 그린닷, 게이지 0, 1초 후 9개 진단 LED 모두 RED, 하단 팝업 "시스템 고장 (9개 시스템 오류)" — 디바운스 동작 확인.
7. **bag 재생 통합**
   ```
   rosbag play <project_bag>
   ```
   - 게이지 ≈50 Hz tick, 객체 박스 이동, 신호등 칩 색상 변경, link/lane 갱신, 우패널 9 sensor 행 hz 표시 (`50.0 Hz`, `10.0 Hz` 등).
8. **사이드 바이 사이드**
   ```
   roslaunch pyqt_hmi hmi_a1.launch       # PyQt A-1
   roslaunch web_hmi web_hmi.launch       # Web (다른 터미널)
   ```
   - 같은 bag 으로 양쪽 동시 실행 → 속도/모드/팝업 일치 (±100ms). 컨트롤러 추출이 의미 변경 0임을 입증.
9. **명령 양방향 테스트**
   - 웹 STOP REC 버튼 클릭 → `/hmi/cmd/bag_toggle` publish → 브리지가 `BaseHmiStateController.toggle_bag()` 호출 → `bag_recording` flip → `/hmi/bag` 갱신 → UI 의 bag-banner 가 "BAG · IDLE" 로 토글. 동일 패턴으로 mode_request.
10. **오프라인 폰트/CDN 테스트** — 인터넷 끊고 재로드 → vendor/ 자산만으로 시안 그대로 렌더링.
11. **회귀 — 기존 PyQt** — `roslaunch pyqt_hmi hmi_a1.launch` 가 동일하게 동작 (HmiStateController 의 시그널 시그니처 유지).

---

## Critical Files

수정 대상 (1개):
- `/home/sim/mcar/src/visualization/pyqt_hmi/scripts/utils/hmi_state.py` — `BaseHmiStateController` 추출, `HmiStateController` 는 얇은 Qt 어댑터로 남김. 외부 시그니처 0 변경.

신규 (web_hmi 패키지 전체):
- `/home/sim/mcar/src/visualization/web_hmi/package.xml`
- `/home/sim/mcar/src/visualization/web_hmi/CMakeLists.txt`
- `/home/sim/mcar/src/visualization/web_hmi/launch/web_hmi.launch`
- `/home/sim/mcar/src/visualization/web_hmi/scripts/web_hmi_bridge.py`
- `/home/sim/mcar/src/visualization/web_hmi/scripts/_demo_publisher.py` (선택)
- `/home/sim/mcar/src/visualization/web_hmi/web/index.html`
- `/home/sim/mcar/src/visualization/web_hmi/web/hmi/HMIScreen.jsx`
- `/home/sim/mcar/src/visualization/web_hmi/web/hmi/components.jsx`
- `/home/sim/mcar/src/visualization/web_hmi/web/hmi/styles.css`
- `/home/sim/mcar/src/visualization/web_hmi/web/hmi/ros_bridge.jsx`
- `/home/sim/mcar/src/visualization/web_hmi/web/vendor/{react,react-dom,babel,roslib}.min.js`
- `/home/sim/mcar/src/visualization/web_hmi/web/vendor/fonts/**`

참조 (수정 없음):
- `/home/sim/mcar/claude_work_list/hmi_design/new/Ioniq5 HMI.html` — 진입점 구조 (CDN script 순서)
- `/home/sim/mcar/claude_work_list/hmi_design/new/hmi/{HMIScreen,components}.jsx`, `styles.css` — 시안 원본 (그대로 복사)
- `/home/sim/mcar/src/visualization/pyqt_hmi/scripts/widgets_a1/main_window_a1.py:144-176` — 컨트롤러 시그널 16개의 사용 사례 (와이어업 참고)
- `/home/sim/mcar/src/visualization/pyqt_hmi/launch/hmi_a1.launch` — rosparam 명세 모방 대상

---

## 8. 범위 밖 (별도 PR)

- BMS/EV 토픽 → Battery/Range/Motor/Pack V
- shapefile (`A2_LINK_epsg5179.shp`) 차선 렌더링
- CPU/MEM 시스템 모니터 publish
- Plan path 시각화
- Mission Objectives 데이터 소스
- 키보드 단축키 (⌘ S / M / ↵)
- 다국어 i18n 시스템 (현재는 시안의 한글 inline `<span class="ko">` 그대로)
- 모바일/태블릿 반응형 (1600×900 고정)

---

## 9. 작업 순서 (Suggested Sequencing)

1. **PR-1**: `hmi_state.py` 리팩토링만 — `BaseHmiStateController` 추출. 회귀 검증: `hmi_a1.launch` 실행 후 동작 일치.
2. **PR-2**: `web_hmi` 패키지 골격 + `web_hmi_bridge.py` + `_demo_publisher.py`. 검증: 브라우저 없이 `rostopic echo /hmi/state` 로 JSON 확인.
3. **PR-3**: `web/` 정적 자산 + roslibjs 와이어업. 검증: 7번 — bag 재생 시 시안 그대로 렌더링 + 데이터 결합.
4. **PR-4 (선택)**: `_demo_publisher.py` 정리, README, 폰트 라이선스, CPU/MEM publish 등 부가 항목.

각 PR 은 단독으로 빌드/검증 가능하며 회귀 위험 0.

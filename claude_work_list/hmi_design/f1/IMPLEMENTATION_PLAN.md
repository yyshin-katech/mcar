# F1 KATECH HMI 통합 개발 계획

> **산출물 위치**: `claude_work_list/hmi_design/f1/IMPLEMENTATION_PLAN.md`
> **작성일**: 2026-05-02
> **시안**: `Ioniq5 HMI F1 KATECH.html` + `F1HMI.jsx` (443 lines) + `F1 KATECH 웹(React) 구현 가이드.md` (533 lines)

---

## 1. Overview

`claude_work_list/hmi_design/f1/` 에 추가된 **F1 텔레메트리 스타일 시안**을 기존 `web_hmi` 패키지에 **사이블링 변형(variant)** 으로 통합한다.

**핵심 결정**
1. **노빌드 패턴 유지** — 시안 가이드는 Vite/npm/nginx 풀빌드를 권장하나, 현재 `web_hmi` 는 차량 PC 오프라인/Node 미설치 시나리오 대응을 위해 노빌드 (vendored UMD + babel-in-browser) 로 결정됨. F1 도 같은 패턴 따름.
2. **사이블링 진입점** — 기존 `web/index.html` 은 손대지 않고 `web/index_f1.html` 신규 추가. `roslaunch web_hmi web_hmi.launch variant:=f1` 로 F1 모드 실행, 디폴트는 기존 디자인. 두 시안 동시 비교 가능.
3. **인프라 100% 재사용** — 기존 `RosProvider` Context + 6 개 도메인 훅(`useRosState`, `useDiagnostics`, `useTopicHz`, `useObjects`, `usePopup`, `useTraffic`, `useBag`) + 6 개 outbound 토픽 그대로. 브리지 노드(`web_hmi_bridge.py`) 변경 0.
4. **시안 구조 보존** — `F1HMI.jsx` 의 6 섹션 + 8-cell 텔레메트리 + 톱뷰 environment 의 시각 자산은 그대로. 하드코딩 값만 props 로 받도록 리팩.

---

## 2. 현재 상태 (전제)

### `web_hmi` 패키지 PR-3 까지 완료
| 자산 | 상태 |
|---|---|
| `src/visualization/web_hmi/scripts/web_hmi_bridge.py` | 6 토픽 + topic_hz JSON 100ms 발행 |
| `web/vendor/{react,react-dom,babel,roslib}.min.js` | 체크인 완료 |
| `web/hmi/ros_bridge.jsx` | RosProvider + 6 hooks + `publishModeRequest` / `publishBagToggle` |
| `web/hmi/HMIScreen.jsx` + `components.jsx` + `styles.css` | 기존 시안 ROS 결합 완료 |
| `launch/web_hmi.launch` | rosbridge 9090 + http.server 8088 + xdg-open |

### 발행 토픽 contract
| 토픽 | 주기 | 페이로드 (JSON via `std_msgs/String`) |
|---|---|---|
| `/hmi/state` | 100 ms | `speed, gear, mode, aeb, steering, ego{east,north,yaw}, gps{rtk,lon_std,lat_std}, speed_limit, link_id, lane_label, on_odd, road_state, selected_mode` |
| `/hmi/diagnostics` | 100 ms | `status: {gps,adcu,lidar,radar,v2x,hmi,vcu,cam,ipc}` (0/1/2) |
| `/hmi/topic_hz` | 100 ms | 위 9 키별 Hz |
| `/hmi/objects` | 30 Hz cap | `[{id,x,y,vx,vy,width,length,orientation,type}, …]` |
| `/hmi/popup` | 변경 시 | `{text, severity}` |
| `/hmi/traffic` | 변경 시 | `{color:0..3, time_decisec, look_at}` |
| `/hmi/bag` | 변경 시 | `{recording, info}` |

인바운드: `/hmi/cmd/mode_request` (Bool), `/hmi/cmd/bag_toggle` (Empty).

---

## 3. 변경 파일 목록

### 신규 파일 (PR-F1)

| 경로 | 라인 | 역할 |
|---|---:|---|
| `src/visualization/web_hmi/web/index_f1.html` | ~60 | F1 진입점. vendor 4개 + `f1/F1HMI.jsx` + `f1/F1HMIScreen.jsx` 로딩. `<RosProvider><F1HMIScreen /></RosProvider>` 마운트. `FitFrame` 으로 viewport 적합 변환. |
| `src/visualization/web_hmi/web/f1/F1HMI.jsx` | ~470 | 시안 `F1HMI.jsx` 거의 그대로 복사. 변경점: (a) `Environment(objs, popupBanner)` props 받도록, (b) `HMI` → `F1HMIShell` 리네이밍 후 모든 표시값 props 화, (c) `window.HMI` 글로벌 노출 제거. |
| `src/visualization/web_hmi/web/f1/F1HMIScreen.jsx` | ~120 | 6 hooks 호출 → props 조립 → `<F1HMIShell {...props} />`. ROS state ↔ 시안 prop 변환 함수 (`mapTrafficColor`, `formatRtk`, `epsg5179HeadingDeg`, …) 모음. `<F1TopBar>`, `<F1BottomStrip>` sub-component. |

### 수정 파일 (PR-F1)

| 경로 | 변경 내용 |
|---|---|
| `src/visualization/web_hmi/web/hmi/ros_bridge.jsx` | (a) `useTopicStale(topicName, graceMs=500)` 헬퍼 추가. (b) RosContext 에 `lastMessageAgeMs`, `connectedSince` 노출. |
| `src/visualization/web_hmi/launch/web_hmi.launch` | `variant` arg 추가, page 명을 `open_browser` 노드에 전달. |
| `src/visualization/web_hmi/scripts/open_browser.py` (또는 launch inline) | `page` 파라미터 받아 URL 조립. |

### 변경 없는 파일

- `web/index.html`, `web/hmi/HMIScreen.jsx`, `web/hmi/components.jsx`, `web/hmi/styles.css` — 기존 디자인 0 회귀.
- `web_hmi_bridge.py`, `pyqt_hmi/scripts/utils/hmi_state.py` — 본 PR 범위 밖. PR-F2 에서 σ-UP/HDOP/SATS, LATERAL/JERK/LEAD/PLAN, CPU 추가.

---

## 4. F1 ↔ ROS 토픽 매핑

| F1 시안 위치 | 시안 prop | 데이터 출처 | PR |
|---|---|---|---|
| **01 VELOCITY · STEER** | | | |
| SpeedHalf value | `value` | `state.speed` | F1 |
| SpeedHalf limit | `limit` | `state.speed_limit` | F1 |
| SteerDial angle | `angle` | `state.steering` (deg) | F1 |
| SteerDial GEAR D | `gearLetter` | `{1:"P",2:"R",3:"N",4:"D"}[state.gear]` | F1 |
| THR / BRK / ACCEL | 3 stat | — (자리 "—") | **F2** |
| **02 DRIVE MODE** | | | |
| MANUAL/AUTONOMOUS toggle | `engaged` | `state.mode === 1` | F1 |
| ● ENGAGED 색 | dot color | `state.mode===1 ? green : amber` | F1 |
| v_max | `vMax` | `state.speed_limit` | F1 |
| ODD line | `oddText` | `state.on_odd === 0 ? "nominal" : "ODD 이탈"` | F1 |
| **03 V2X SIGNAL** | | | |
| TrafficLight phase | `phase` | `traffic.color` → `["GREEN","AMBER","RED","OFF"][color]` | F1 |
| TrafficLight remain | `remain` | `Math.round(traffic.time_decisec / 10)` | F1 |
| **04 LOCALIZATION** | | | |
| LANE ID | `laneId` | `state.lane_label` | F1 |
| LINK | `linkId` | `state.link_id` | F1 |
| σ-EAST / σ-NORTH | `sigE`, `sigN` | `state.gps.lon_std`, `state.gps.lat_std` (cm) | F1 |
| σ-UP / HDOP / SATS | 3 stat | — | **F2** |
| HEADING | `heading` | `state.ego.yaw` (deg) | F1 |
| right "RTK FIX" | `rtkLabel` | `state.gps.rtk` → `["NO RTK","FLOAT","FIX"][rtk]` | F1 |
| **05 SYSTEM HEALTH** | | | |
| 6 sensor row | `health[]` | `diagnostics.status[*]` + `topic_hz[*]` (gps→GPS-RTK, adcu→K-ADCU, lidar→LIDAR, radar→RADAR, cam→CAMERA, v2x→V2X) | F1 |
| right 4 dot summary | `summary` | health 합집합 (any RED → red, any AMBER → amber, else green) | F1 |
| **06 ENVIRONMENT** | | | |
| Environment objs | `objs` | `objects[]` → `metersToStage` 변환 후 `[{id,k,c,x,y,w,h,dist,spd}]`. 5.6 px/m (ringRX[4]=560px / 100m) | F1 |
| ODD warn banner | `oddBanner` | `popup.severity!=="info" ? popup.text : null` | F1 |
| REC banner path | `bag.info` | `bag.info` | F1 |
| REC banner state | `bag.recording` | `bag.recording` | F1 |
| STOP click | onClick | `publishBagToggle()` | F1 |
| **하단 8-cell** | | | |
| EGO-VEL | `egoVel` | `state.speed` (km/h) | F1 |
| Δ-LIM | `dLim` | `state.speed - state.speed_limit` | F1 |
| LATERAL / JERK / LEAD-D / LEAD-Δv / PLAN-H | 5 stat | — | **F2** |
| CPU | `cpu` | — | **F2** |
| **상단 토픽바** | | | |
| K 로고 + KATECH·IONIQ 5 | 정적 | 정적 | F1 |
| AD/HMI v3.2 · NODE | `nodeName` | `"web_hmi_bridge"` (정정: pyqt_display 아님) | F1 |
| UTC | client | `new Date().toUTCString().slice(17,25)` | F1 |
| KST | client | `formatKstClock(new Date())` | F1 |
| TICK | client | `((Date.now() - pageLoadAt)/1000).toFixed(3)+"s"` | F1 |
| NET | RTT | `lastMessageAgeMs` (RosContext) | F1 |
| ROS · /ad_can OK 칩 | `adcuOk` | `topic_hz.adcu > 0 && !stale(adcu)` | F1 |

---

## 5. PR 분할

### **PR-F1** (본 계획의 핵심)

**범위**:
- F1 진입점 + `F1HMI.jsx` + `F1HMIScreen.jsx` 신규
- `ros_bridge.jsx` 의 `useTopicStale` 추가, `lastMessageAgeMs` 노출
- `web_hmi.launch` 의 `variant` arg
- 본 IMPLEMENTATION_PLAN.md 산출물

**검증**: §6 의 9 step 모두 통과.

**DOD**:
- `roslaunch web_hmi web_hmi.launch variant:=f1` 로 F1 시안이 ROS 데이터에 결합되어 렌더링.
- `roslaunch web_hmi web_hmi.launch` (default) 회귀 0.
- `roslaunch pyqt_hmi hmi_a1.launch` 회귀 0.

### PR-F2 (선택 후속)

**범위**: `web_hmi_bridge.py` 확장
- σ-UP / HDOP / SATS publish (`/sensors/cpt7_gps` 또는 ublox NavPVT 에서 추출)
- LATERAL / JERK / LEAD-D / LEAD-Δv / PLAN-H publish (lateral_error, lead vehicle, planned path)
- CPU / MEM publish (`psutil`)

**완료 기준**: F1 의 "—" 표시가 모두 실제 값으로 바뀜.

### PR-F3 (선택 후속)

**범위**: 폰트 vendor 화 (오프라인 차량 PC 대응)
- Inter, JetBrains Mono, Pretendard `.woff2` 를 `web/vendor/fonts/` 로 체크인
- `index.html` / `index_f1.html` 의 Google Fonts link 제거, `@font-face` 로 교체
- Pretendard OFL 라이선스 첨부

### PR-F4 (선택 후속)

**범위**: nginx 운영 배포 (가이드 §10)
- `/etc/nginx/sites-available/web_hmi` reverse proxy
- systemd `web-hmi.service` (부팅 시 자동)
- `/etc/hosts` 또는 mDNS 로 hostname 할당

---

## 6. 검증 절차

### 6.1 정적 시안 회귀 (시안 폴더 readonly 확인)
```bash
cd /home/sim/mcar/claude_work_list/hmi_design/f1
python3 -m http.server 8089
# Windows 브라우저
explorer.exe "http://localhost:8089/Ioniq5%20HMI%20F1%20KATECH.html"
```
- F1 시안 그대로 렌더링.
- `claude_work_list/hmi_design/f1/F1HMI.jsx` 파일은 미변경 (디자이너 원본 보존).

### 6.2 catkin 빌드
```bash
cd /home/sim/mcar
catkin_make --pkg web_hmi
source devel/setup.bash
```
- 빌드 에러 0. (정적 자산만 추가, Python 콜 변경 없음.)

### 6.3 F1 진입점 단독 (rosbridge 없음)
```bash
cd /home/sim/mcar/src/visualization/web_hmi/web
python3 -m http.server 8088
explorer.exe "http://localhost:8088/index_f1.html"
```
- RosProvider 가 ws://localhost:9090 연결 실패 → UI default 값으로 렌더 (gauge 0, AUTONOMOUS off, traffic OFF, objs 빈 배열).
- 상단 ROS 칩이 회색/red.

### 6.4 F1 + roscore + 데모 publisher
```bash
roscore &
rosrun web_hmi _demo_publisher.py &
roslaunch web_hmi web_hmi.launch variant:=f1
```
- 브라우저 자동 오픈 → F1 시안.
- SpeedHalf 0→80 사이클, SteerDial sweep, TrafficLight 색 순환, Environment objs 공전.
- 상단 ROS 칩 green, NET ms 표시.

### 6.5 default + F1 사이드 바이 사이드
```bash
# 터미널 1
roslaunch web_hmi web_hmi.launch variant:=default open_browser:=true
# 터미널 2 (또는 같은 launch 의 두 번째 탭)
roslaunch web_hmi web_hmi.launch variant:=f1 open_browser:=true ws_port:=9091 web_port:=8089
```
- 같은 데이터 → 두 디자인이 같은 값 표시 (시각 회귀 검증).

### 6.6 bag 재생 + 진단 시나리오
```bash
roscore &
roslaunch web_hmi web_hmi.launch variant:=f1
rosbag play <project_bag>
```
- 1초 후 진단 9 키 모두 ERR (rosbag 에 진단 토픽 없으면) → SYSTEM HEALTH 6행 dot 모두 red, 우상단 4 dot summary red.
- popup 발생 시 ENVIRONMENT 의 ODD warn banner 표시.
- `/track_Multi_RS` 흐르면 `objects` 가 `Environment` 6 obj 슬롯에 박스로 표시.

### 6.7 STOP REC 양방향
- ENVIRONMENT 우상단 STOP 클릭 → `/hmi/cmd/bag_toggle` publish → `bag_recording` flip → `/hmi/bag` 갱신 → REC banner 가 IDLE 로 토글.
- 동일 패턴으로 02 DRIVE MODE 토글 → `publishModeRequest(true)`.

### 6.8 default 디자인 회귀
```bash
roslaunch web_hmi web_hmi.launch
```
- 기존 디자인 동작 100% 일치. (디폴트 variant=default)

### 6.9 PyQt 회귀
```bash
roslaunch pyqt_hmi hmi_a1.launch
```
- 변경 없음.

---

## 7. 위험 / 미해결

1. **`state.ego.yaw` 단위 확인** — 라디안인지 도(deg)인지. `web_hmi_bridge.py` 1줄 확인. F1 시안은 deg 표시 (`142.7°`). 변환 필요 시 `F1HMIScreen` 내부에서 처리.
2. **객체 좌표 변환** — F1 의 Environment SVG viewBox 880×680, ego 위치 (W/2, H*0.62). px/m = ringRX[4]=560 / 100m = 5.6 px/m. `metersToStage(x_meters_lateral, y_meters_forward)` → `(ego.x + x*5.6, ego.y - y*5.6)`. 디버그 publisher 로 검증.
3. **`pyqt_display` 노드명 텍스트** — 시안 상단의 "AD/HMI v3.2 · NODE pyqt_display" 잘못된 노드명. F1HMIScreen 에서 `web_hmi_bridge` 로 정정.
4. **scanlines overlay 클릭 차단** — 시안에 `pointerEvents: "none"` 이미 있음. 확인.
5. **`FitFrame` 도입** — 1600×900 고정 → 차량 PC 1920×1080 letterbox 발생. 가이드의 `FitFrame` (transform: scale 자동) 을 `index_f1.html` 에 한정 적용. 기존 `index.html` 은 변경 없음.
6. **폰트 link** — `index_f1.html` 도 Google Fonts link (Inter, JetBrains Mono, Pretendard) 추가 필요. 오프라인 환경은 PR-F3 에서 vendor 화.
7. **SYSTEM HEALTH 6행 vs diagnostics 9키** — 시안 라벨 6개 (GPS-RTK, K-ADCU, LIDAR, RADAR, CAMERA, V2X) ↔ 9키 중 6 매핑 (gps, adcu, lidar, radar, cam, v2x). `hmi/vcu/ipc` 는 시안에 자리 없음 — 우상단 4 dot summary 에 합산. PR-F2 에서 7행 확장 검토.
8. **traffic.color 매핑 검증** — `web_hmi_bridge.py` 의 `traffic.color`: 0=GREEN, 1=AMBER, 2=RED, 3=OFF (HmiStateController 기존 매핑 가정). 본 PR 작업 전 1회 확인.
9. **`useTopicStale` graceMs=500** — F1 가이드 권장. `web_hmi_bridge.py` 100ms 발행 → 5tick 누락 시 stale. 보수적이면서 적절.
10. **Inline styles only** — F1 디자인이 외부 CSS 미사용 (모든 스타일 inline). 다크/라이트 토글이나 토큰 외부화 어려움. 본 PR 범위 밖.

---

## 8. DOD (Definition of Done)

- [ ] **PR-F1 코드 변경**: 신규 3 파일 + 수정 3 파일.
- [ ] **빌드**: `catkin_make --pkg web_hmi` 에러 0.
- [ ] **단독 실행**: `python3 -m http.server 8088` + `index_f1.html` → 시안 default 값 렌더링.
- [ ] **rosbridge 결합**: `roslaunch web_hmi web_hmi.launch variant:=f1` → 6 hooks 데이터로 시안의 모든 위젯 갱신.
- [ ] **데모 publisher 시나리오**: speed/steer/mode/traffic/objects 모두 변화 표시.
- [ ] **STOP REC 양방향**: F1 의 STOP 버튼 → bridge → bag 토글 → UI 반영.
- [ ] **default 회귀 0**: `roslaunch web_hmi web_hmi.launch` (variant 미지정) → 기존 디자인 100% 동작.
- [ ] **pyqt_hmi 회귀 0**: `roslaunch pyqt_hmi hmi_a1.launch` → 변경 없음.
- [ ] **시안 readonly**: `claude_work_list/hmi_design/f1/F1HMI.jsx` 미변경 (디자이너 원본 보존).

---

## 9. 참조 파일

### 시안 원본 (readonly)
- `claude_work_list/hmi_design/f1/Ioniq5 HMI F1 KATECH.html` — 진입점 구조
- `claude_work_list/hmi_design/f1/F1HMI.jsx` — 컴포넌트 11개, 컬러 토큰 `T`, 1600×900 그리드
- `claude_work_list/hmi_design/f1/F1 KATECH 웹(React) 구현 가이드.md` — 인프라 의도 (Vite/nginx 는 채택 안 함, 디자인/컴포넌트 의도만 반영)

### 기존 web_hmi 인프라 (재사용)
- `src/visualization/web_hmi/web/hmi/ros_bridge.jsx` — RosProvider + 6 hooks
- `src/visualization/web_hmi/web/hmi/HMIScreen.jsx` — 시안 ↔ ROS 결합 패턴 모방
- `src/visualization/web_hmi/scripts/web_hmi_bridge.py` — 토픽 contract
- `src/visualization/web_hmi/launch/web_hmi.launch` — rosbridge + http.server 통합

### PR-F1 산출 파일
| 경로 | 신규/수정 |
|---|---|
| `src/visualization/web_hmi/web/index_f1.html` | 신규 |
| `src/visualization/web_hmi/web/f1/F1HMI.jsx` | 신규 |
| `src/visualization/web_hmi/web/f1/F1HMIScreen.jsx` | 신규 |
| `src/visualization/web_hmi/web/hmi/ros_bridge.jsx` | 수정 (`useTopicStale` + `lastMessageAgeMs`) |
| `src/visualization/web_hmi/launch/web_hmi.launch` | 수정 (`variant` arg) |
| `src/visualization/web_hmi/scripts/open_browser.py` | 수정 (`page` 파라미터) |
| `claude_work_list/hmi_design/f1/IMPLEMENTATION_PLAN.md` | 신규 (본 파일) |

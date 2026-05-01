# Ioniq5 HMI · A-1 (Solid Top-down · Amber) → ROS1 + Web 구현 가이드

> 같은 시안을 **웹 프런트엔드 (React) + ROS1 백엔드 (rosbridge / roslibjs)** 구조로 옮기는 절차서. PyQt 대비 장점 — 원격 표시·반응형·실시간 핫리로드·디자인이 곧 코드. Claude CLI 에 단계별로 붙여 넣어 실행하세요.

---

## 0. 왜 웹인가 (PyQt 와의 비교)

| 기준 | PyQt5 | Web (React + rosbridge) |
|---|---|---|
| 실행 환경 | 차량 PC 의 X 디스플레이 | 모든 브라우저 (차량 내 / 원격 태블릿) |
| 폰트·아이콘·그라데이션 | 제한적 | CSS / SVG / WebGL 자유 |
| 핫리로드 | 없음 | Vite HMR 즉시 반영 |
| 멀티 클라이언트 | 1 프로세스 = 1 디스플레이 | rosbridge 1개 → N 클라이언트 |
| 시안 ↔ 구현 거리 | SVG → QPainter 재작성 필요 | 기존 JSX 그대로 사용 |
| 단점 | — | rosbridge 한 단계 추가, 네트워크 지연 (LAN 환경에선 무시 가능) |

차량용 키오스크 모드: Chromium `--kiosk --app=http://localhost:5173` 으로 풀스크린 부팅.

---

## 1. 아키텍처

```
┌────────────────── Vehicle PC (Ubuntu 20.04) ──────────────────┐
│                                                                │
│  ROS1 Noetic ──┬── /vehicle/twist                              │
│                ├── /vehicle/gear                               │
│                ├── /perception/objects                         │
│                └── /planning/path                              │
│       │                                                        │
│       ▼                                                        │
│  rosbridge_server (websocket, port 9090)                       │
│       │                                                        │
│       ▼                                                        │
│  Vite dev server (port 5173)  ──→  Chromium --kiosk            │
│       │                                                        │
│       └─ React + roslibjs                                      │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

---

## 2. 사전 환경

```bash
# ROS1 + rosbridge
sudo apt update
sudo apt install -y \
  ros-noetic-rosbridge-server \
  ros-noetic-tf2-web-republisher \
  ros-noetic-web-video-server

# Node.js 20 (NodeSource)
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs

# 키오스크용 (선택)
sudo apt install -y chromium-browser
```

확인:
```bash
node --version   # v20.x
rospack find rosbridge_server
```

---

## 3. ROS 패키지 스캐폴딩

```bash
cd ~/catkin_ws/src
catkin_create_pkg pyqt_hmi_web rospy std_msgs
cd pyqt_hmi_web
mkdir -p launch web
```

`launch/web_hmi.launch`:
```xml
<launch>
  <!-- 1) rosbridge websocket -->
  <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch">
    <arg name="port" value="9090"/>
  </include>

  <!-- 2) (선택) tf2 → web bridge -->
  <node pkg="tf2_web_republisher" type="tf2_web_republisher"
        name="tf2_web_republisher" output="screen"/>

  <!-- 3) (선택) 카메라 토픽을 MJPEG 로 노출 -->
  <node pkg="web_video_server" type="web_video_server"
        name="web_video_server">
    <param name="port" value="8080"/>
  </node>
</launch>
```

빌드 후:
```bash
cd ~/catkin_ws && catkin build pyqt_hmi_web
source devel/setup.bash
roslaunch pyqt_hmi_web web_hmi.launch
# → ws://localhost:9090 에서 rosbridge 응답
```

---

## 4. 웹 프로젝트 부트스트랩 (Vite + React)

```bash
cd ~/catkin_ws/src/pyqt_hmi_web/web
npm create vite@latest . -- --template react
npm install
npm install roslib
```

`package.json` 스크립트:
```json
{
  "scripts": {
    "dev":   "vite --host 0.0.0.0 --port 5173",
    "build": "vite build",
    "preview": "vite preview --host 0.0.0.0 --port 5173"
  }
}
```

폰트는 `index.html` 의 `<head>` 에 추가:
```html
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&family=JetBrains+Mono:wght@400;500;600&family=Pretendard:wght@400;500;600&display=swap" rel="stylesheet">
```

오프라인 차량 환경이면 `npm install @fontsource/inter @fontsource/jetbrains-mono` 후 `main.jsx` 에서 import.

---

## 5. 디자인 시스템 (CSS 변수)

`src/styles.css`:
```css
:root {
  --bg-0: #0E0F11;
  --bg-1: #15171A;
  --bg-2: #1C1F23;
  --bg-3: #232830;
  --line-soft: #232830;
  --text-0: #F4F5F7;
  --text-1: #C8CCD2;
  --text-2: #8A9099;
  --text-3: #5A616B;
  --amber-0: #FFB547;
  --amber-1: #F59E2C;
  --amber-2: #C97A14;
  --red:   #F87171;
  --green: #34D399;
  --font-display: "Inter", "Pretendard", system-ui, sans-serif;
  --font-mono: "JetBrains Mono", ui-monospace, monospace;
}
* { box-sizing: border-box; }
html, body { margin: 0; background: var(--bg-0); color: var(--text-0);
  font-family: var(--font-display); font-variant-numeric: tabular-nums; }
```

---

## 6. ROS 클라이언트 훅 (roslibjs)

`src/ros/useRos.js`:
```javascript
import { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";

export function useRos(url = "ws://localhost:9090") {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);
  useEffect(() => {
    const r = new ROSLIB.Ros({ url });
    r.on("connection", () => setConnected(true));
    r.on("close", () => setConnected(false));
    r.on("error", () => setConnected(false));
    setRos(r);
    return () => r.close();
  }, [url]);
  return { ros, connected };
}

export function useTopic(ros, name, type, throttleRate = 100) {
  const [msg, setMsg] = useState(null);
  const subRef = useRef(null);
  useEffect(() => {
    if (!ros) return;
    const t = new ROSLIB.Topic({
      ros, name, messageType: type,
      throttle_rate: throttleRate, queue_length: 1,
    });
    subRef.current = t;
    t.subscribe(setMsg);
    return () => { t.unsubscribe(); };
  }, [ros, name, type, throttleRate]);
  return msg;
}

export function callService(ros, name, type, request) {
  return new Promise((resolve, reject) => {
    const svc = new ROSLIB.Service({ ros, name, serviceType: type });
    svc.callService(new ROSLIB.ServiceRequest(request), resolve, reject);
  });
}
```

사용 예:
```jsx
const { ros, connected } = useRos();
const twist = useTopic(ros, "/vehicle/twist", "geometry_msgs/TwistStamped");
const kmh = twist ? twist.twist.linear.x * 3.6 : 0;
return <SpeedGauge speed={kmh} />;
```

---

## 7. 시안 컴포넌트 이식

이미 만든 시안의 React 컴포넌트들을 그대로 옮깁니다 (이 프로젝트의 `hmi/components.jsx`, `hmi/HMIScreen.jsx`). 핵심은 **외부 input 만 ROS 토픽으로 교체**하는 것:

```jsx
// src/HMIApp.jsx
import { useRos, useTopic } from "./ros/useRos";
import { HMIScreen } from "./components/HMIScreen";

export default function HMIApp() {
  const { ros, connected } = useRos();
  const twist = useTopic(ros, "/vehicle/twist", "geometry_msgs/TwistStamped");
  const gear  = useTopic(ros, "/vehicle/gear", "std_msgs/String");
  const objs  = useTopic(ros, "/perception/objects", "vision_msgs/Detection3DArray");
  const path  = useTopic(ros, "/planning/path", "nav_msgs/Path");
  const power = useTopic(ros, "/vehicle/powertrain", "pyqt_hmi_web/Powertrain");
  const fix   = useTopic(ros, "/gnss/fix", "sensor_msgs/NavSatFix");

  return (
    <HMIScreen
      connected={connected}
      speed={twist ? twist.twist.linear.x * 3.6 : 0}
      gear={gear?.data ?? "P"}
      objects={objs?.detections ?? []}
      path={path?.poses ?? []}
      power={power}
      fix={fix}
    />
  );
}
```

`HMIScreen` 안에서:
- `SpeedGauge speed={speed}` → 그대로
- `VehicleStage` 안의 `<TrafficObjects>` 는 `objects` prop 을 받아 ROS 좌표를 화면 픽셀로 변환:

```jsx
function worldToScreen({ x, y }, scale = 4) {
  // ego 기준 1m → 4px, 전방 = -screenY
  return { sx: x * scale, sy: -y * scale };
}
```

---

## 8. 좌표 변환 + 차량 박스 렌더

ROS `/perception/objects` 의 각 detection 은 `bbox.center.position {x, y}` 와 `bbox.size {x, y}` 를 가집니다 (ego 프레임). 이를 시안의 박스 + 라벨 로 그대로 매핑:

```jsx
{objects.map((d, i) => {
  const { sx, sy } = worldToScreen(d.bbox.center.position);
  const w = d.bbox.size.x * 4;
  const h = d.bbox.size.y * 4;
  const cls = d.results?.[0]?.id ?? "OBJ";
  const dist = Math.hypot(d.bbox.center.position.x, d.bbox.center.position.y);
  return (
    <g key={i} transform={`translate(${sx} ${sy})`}>
      <rect x={-w/2} y={-h/2} width={w} height={h} rx="2"
            fill="rgba(124,58,237,0.08)" stroke="#7C3AED" strokeWidth="1.2"/>
      <text x={w/2 + 4} y={-h/2 + 6} fill="#7C3AED"
            fontFamily="JetBrains Mono" fontSize="7">
        #{String(i).padStart(3,"0")} · {cls}
      </text>
      <text x={w/2 + 4} y={-h/2 + 14} fill="#8A9099"
            fontFamily="JetBrains Mono" fontSize="6">
        {dist.toFixed(1)}m
      </text>
    </g>
  );
})}
```

---

## 9. 키오스크 부팅 (차량 PC 자동시작)

`/etc/systemd/system/ioniq5-hmi.service`:
```ini
[Unit]
Description=Ioniq5 HMI Web
After=network.target roscore.service

[Service]
Type=simple
User=ioniq
Environment="DISPLAY=:0"
WorkingDirectory=/home/ioniq/catkin_ws/src/pyqt_hmi_web/web
ExecStartPre=/bin/sleep 5
ExecStart=/usr/bin/chromium-browser \
  --kiosk \
  --noerrdialogs --disable-translate --no-first-run --fast \
  --disable-features=TranslateUI \
  --autoplay-policy=no-user-gesture-required \
  http://localhost:5173

Restart=on-failure

[Install]
WantedBy=graphical.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now ioniq5-hmi
```

---

## 10. 빌드 + 배포

개발:
```bash
# 터미널 1
roslaunch pyqt_hmi_web web_hmi.launch
# 터미널 2
cd ~/catkin_ws/src/pyqt_hmi_web/web && npm run dev
```

프로덕션 빌드 (정적 파일 생성):
```bash
npm run build   # → web/dist/
```

`web/dist` 를 nginx 또는 ROS 노드 안에서 정적으로 서빙:
```bash
sudo apt install -y nginx
sudo cp -r web/dist/* /var/www/html/
# Chromium 가 http://localhost 로 진입
```

---

## 11. ROS 토픽 매핑 (PyQt 가이드와 동일)

| HMI 영역 | ROS 토픽 (예시) | 메시지 |
|---|---|---|
| 속도 게이지 | `/vehicle/twist` | `geometry_msgs/TwistStamped` |
| Drive 모드 | `/vehicle/gear` | `std_msgs/String` |
| 배터리/주행거리 | `/vehicle/powertrain` | 사용자 정의 |
| GNSS | `/gnss/fix` | `sensor_msgs/NavSatFix` |
| 객체 박스 | `/perception/objects` | `vision_msgs/Detection3DArray` |
| 계획 경로 | `/planning/path` | `nav_msgs/Path` |
| 시스템 진단 | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |
| 카메라 (선택) | `/camera/front/image_raw` | `sensor_msgs/Image` → web_video_server 로 MJPEG |

카메라를 띄우려면 `<img src="http://localhost:8080/stream?topic=/camera/front/image_raw" />` 한 줄.

---

## 12. Claude CLI 워크플로

```bash
cd ~/catkin_ws/src/pyqt_hmi_web
claude
```

CLI 안에서 단계별 프롬프트:

1. *"§3 의 `web_hmi.launch` 를 만들고 `roslaunch pyqt_hmi_web web_hmi.launch` 가 9090 포트에서 응답하는지 확인해줘."*
2. *"§4 절차로 `web/` 안에 Vite + React 프로젝트를 부트스트랩하고 npm run dev 로 빈 페이지가 뜨는지 봐줘."*
3. *"§5 의 `styles.css` 를 만들고 `main.jsx` 에서 import 해줘."*
4. *"§6 의 `useRos / useTopic` 훅을 `src/ros/useRos.js` 로 추가하고 더미 컴포넌트로 connection 상태를 표시해줘."*
5. *"이 프로젝트(omelette) 의 `hmi/components.jsx` 와 `hmi/HMIScreen.jsx` 를 `src/components/` 로 이식해줘. 모든 시각 요소는 그대로, 데이터만 prop 으로 받게 분리해."*
6. *"§7 의 `HMIApp.jsx` 를 만들어 ROS 토픽 → HMIScreen prop 으로 연결해줘."*
7. *"§8 의 좌표 변환 + 박스 렌더를 `TrafficObjects` 에 적용해줘."*
8. *"§9 의 systemd 유닛으로 부팅 시 자동 풀스크린 진입하게 만들어줘."*

각 단계 후 `git commit`. 막히면 §부록 A 부터.

---

## 부록 A — 자주 마주치는 문제

| 증상 | 원인 / 해결 |
|---|---|
| `WebSocket connection failed` | rosbridge 미실행, 또는 방화벽. `sudo ufw allow 9090/tcp` |
| 토픽이 안 들어옴 | `rostopic echo /vehicle/twist` 가 동작하는지 먼저 확인. `messageType` 문자열이 정확해야 함 (`geometry_msgs/TwistStamped`) |
| 메시지 폭주로 화면이 끊김 | `useTopic(ros, name, type, 100)` 의 `throttle_rate` 를 100~200ms 로 |
| 한글이 깨짐 | `index.html` 에 `<meta charset="utf-8">` 와 `Pretendard` 또는 `Noto Sans CJK KR` 폰트 |
| Chromium 풀스크린 안됨 | `--kiosk` + `--app=` 둘 다 시도. wayland 환경이면 X11 세션으로 로그인 |
| `vision_msgs` 메시지 미정의 | `sudo apt install ros-noetic-vision-msgs` 또는 사용자 정의 메시지 (rosbridge 가 .msg 정의를 자동 인식) |
| HMR 후 ROS 연결 끊김 | Vite HMR 이 컴포넌트만 갱신해도 `useRos` 의 cleanup 이 호출됨 → 정상. 자동 재연결됨 |

## 부록 B — PyQt 와 코드 공유

같은 디자인 토큰을 두 군데 (Python `theme.py` + CSS 변수) 에 두면 드리프트 위험. JSON 한 벌을 단일 소스로 두고 양쪽이 읽도록:

```bash
resources/tokens.json   # 단일 소스
resources/qss/app.qss   # build-time 으로 생성
web/src/styles.css      # build-time 으로 생성
```

`scripts/build_tokens.py` 를 만들어 `npm run dev` 와 `catkin build` 양쪽 hook 에 연결하세요.

## 부록 C — 보안 메모

- rosbridge 는 인증이 없습니다. 차량 외부 망에 노출하지 마세요.
- 외부 접속이 필요하면 `nginx` 리버스 프록시 + Basic Auth + TLS, 또는 VPN 뒤에 배치.

---

## 끝.

웹 방식의 가장 큰 매력은 **시안 = 코드** 라는 점입니다. 이 프로젝트의 `Ioniq5 HMI.html` 안의 React 코드를 그대로 차량에서 돌릴 수 있습니다. PyQt 가이드와 함께 비교 검토 후 팀 환경에 맞춰 선택하세요.

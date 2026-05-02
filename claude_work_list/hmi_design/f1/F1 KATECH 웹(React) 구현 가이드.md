# Ioniq5 HMI · F1 KATECH — **웹(React) + ROS1 구현 가이드**

> 대상 디자인: `Ioniq5 HMI F1 KATECH.html` (이미 React JSX 구현 존재 — 이 문서는 **차량 ROS1 백엔드와 연결**해 실제 운용 가능한 웹 HMI로 만드는 절차)
> 차량 환경: **Ubuntu 20.04 · ROS Noetic**
> 표시 단말: **차량 내 임베디드 PC 크롬 / 외부 모니터링 PC 어디서나**

---

## 0. 아키텍처 개요

```
 ┌──────────────────────────────────┐
 │   Vehicle PC (Ubuntu 20.04)      │
 │                                  │
 │   ROS Noetic ──────► rosbridge_server (ws://:9090) ──┐
 │   (publishers)                                       │
 │                                                      │
 │   nginx (정적 호스팅 :80)  ──────►  / (React 빌드)   │
 │                                                      │
 └────────────────────┬─────────────────────────────────┘
                      │  WebSocket
                      ▼
       ┌──────────────────────────────────┐
       │  Browser — React HMI             │
       │   roslibjs ──► topic subscribers │
       │   1600×900 fixed canvas          │
       └──────────────────────────────────┘
```

**핵심 선택지**: rosbridge + roslibjs 를 사용. 차량 ROS1 토픽을 별도 변환 없이 브라우저로 스트리밍.

### 0.1 PyQt 대비 장점

| 측면 | PyQt5 | React 웹 |
|---|---|---|
| 배포 | 차량 PC에 직접 설치 | URL 한 줄로 어디서든 접속 |
| 다중 시청자 | 1대 | N대 동시 |
| HiDPI/4K | 수동 스케일링 | 자유 줌 |
| 개발 반복 | restart 필요 | hot reload |
| 차량과의 거리 | 항상 동일 호스트 | LAN/VPN으로 원격 가능 |

---

## 1. 사전 준비

### 1.1 차량 PC

```bash
# rosbridge
sudo apt install -y ros-noetic-rosbridge-server \
                    ros-noetic-tf2-web-republisher \
                    ros-noetic-web-video-server \
                    nginx

# 노드 (개발용 빌드/번들링은 모니터링 PC에서 해도 됨)
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs
```

### 1.2 rosbridge 실행

```xml
<!-- launch/hmi_web.launch -->
<launch>
  <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch">
    <arg name="port" value="9090"/>
    <arg name="address" value="0.0.0.0"/>
  </include>
  <node pkg="tf2_web_republisher" type="tf2_web_republisher" name="tf2_web_republisher"/>
</launch>
```

> **보안**: 차량 외부에 노출할 경우 nginx에 wss(TLS) + 베이직 인증 / 토큰 검증 미들웨어를 추가. 사내망 한정이면 ws로 충분.

---

## 2. 프로젝트 구조

기존 디자인 자산을 운용 가능한 SPA로 재패키징합니다.

```
ioniq5-hmi-web/
├── package.json
├── vite.config.js
├── index.html                    # 기존 Ioniq5 HMI F1 KATECH.html 의 운용판
├── public/
│   └── fonts/                    # JetBrainsMono / Inter / Pretendard
├── src/
│   ├── main.jsx
│   ├── App.jsx                   # 화면 조립 (시안 그대로 이식)
│   ├── theme.js                  # 색·폰트·간격 토큰
│   ├── ros/
│   │   ├── bridge.js             # roslibjs 래퍼 (싱글톤)
│   │   ├── topics.js             # 토픽 매핑 정의
│   │   └── useRos.js             # React 훅
│   ├── components/
│   │   ├── HeaderBar.jsx
│   │   ├── SpeedHalf.jsx
│   │   ├── SteerDial.jsx
│   │   ├── TrafficLight.jsx
│   │   ├── Localization.jsx
│   │   ├── SystemHealth.jsx
│   │   ├── TopDownView.jsx
│   │   ├── BottomStrip.jsx
│   │   └── RecPanel.jsx
│   └── utils/
│       └── format.js
└── nginx.conf
```

> **Vite 권장**: dev 서버에서 ws 프록시도 같이 설정 가능, prod 빌드는 정적 파일.

```bash
npm create vite@latest ioniq5-hmi-web -- --template react
cd ioniq5-hmi-web
npm i roslib
```

---

## 3. ROS 데이터 컨트랙트

PyQt 가이드와 **동일 토픽 매핑** — 단일 단원으로 관리하세요.

```js
// src/ros/topics.js
export const TOPICS = {
  velocity:    { name: "/vehicle/velocity",      type: "std_msgs/Float32" },
  steer:       { name: "/vehicle/steer_angle",   type: "std_msgs/Float32" },
  pedals:      { name: "/vehicle/pedals",        type: "geometry_msgs/Vector3" },
  accel:       { name: "/vehicle/accel",         type: "std_msgs/Float32" },
  mode:        { name: "/ad/mode",               type: "std_msgs/String"  },
  ad_state:    { name: "/ad/state",              type: "diagnostic_msgs/DiagnosticStatus" },
  spat:        { name: "/v2x/spat",              type: "custom_msgs/Spat" },
  loc:         { name: "/localization/info",     type: "custom_msgs/LocInfo" },
  objects:     { name: "/perception/objects",    type: "derived_object_msgs/ObjectArray" },
  trajectory:  { name: "/planning/trajectory",   type: "nav_msgs/Path" },
  odd_warn:    { name: "/ad/odd_warn",           type: "std_msgs/String" },
  diag:        { name: "/diagnostics_agg",       type: "diagnostic_msgs/DiagnosticArray" },
  rec_state:   { name: "/rosbag_recorder/status",type: "custom_msgs/RecState" },
};
```

> **custom 타입을 못 쓰는 환경**: 모두 `std_msgs/String`(JSON)으로 내려도 됩니다. 본 가이드 코드는 `JSON.parse` 자동 fallback.

---

## 4. roslibjs 래퍼

### 4.1 `bridge.js` — 싱글톤 + 자동 재접속

```js
// src/ros/bridge.js
import ROSLIB from "roslib";

class RosBridge {
  constructor(url) {
    this.url = url;
    this.ros = new ROSLIB.Ros({ url });
    this.subs = new Map();         // name → {topic, listeners[]}
    this.statusListeners = new Set();
    this.connected = false;
    this._wire();
  }
  _wire() {
    this.ros.on("connection", () => this._set(true));
    this.ros.on("close",      () => { this._set(false); this._reconnect(); });
    this.ros.on("error",      () => { this._set(false); });
  }
  _set(c) {
    this.connected = c;
    this.statusListeners.forEach(fn => fn(c));
  }
  _reconnect() {
    setTimeout(() => this.ros.connect(this.url), 1500);
  }
  onStatus(fn) { this.statusListeners.add(fn); return () => this.statusListeners.delete(fn); }

  subscribe(def, cb) {
    let entry = this.subs.get(def.name);
    if (!entry) {
      const topic = new ROSLIB.Topic({
        ros: this.ros, name: def.name, messageType: def.type,
        throttle_rate: 33, queue_size: 1, queue_length: 1,
      });
      entry = { topic, listeners: new Set() };
      topic.subscribe(msg => entry.listeners.forEach(l => l(msg)));
      this.subs.set(def.name, entry);
    }
    entry.listeners.add(cb);
    return () => {
      entry.listeners.delete(cb);
      if (entry.listeners.size === 0) {
        entry.topic.unsubscribe(); this.subs.delete(def.name);
      }
    };
  }

  publish(def, payload) {
    const topic = new ROSLIB.Topic({ ros: this.ros, name: def.name, messageType: def.type });
    topic.publish(new ROSLIB.Message(payload));
  }

  callService(def, request) {
    const svc = new ROSLIB.Service({ ros: this.ros, name: def.name, serviceType: def.type });
    return new Promise((res, rej) =>
      svc.callService(new ROSLIB.ServiceRequest(request), res, rej));
  }
}

export const ros = new RosBridge(
  // dev: wss://localhost:9090, prod: 페이지 호스트 그대로
  (location.protocol === "https:" ? "wss://" : "ws://") +
  (import.meta.env.VITE_ROSBRIDGE_HOST ?? `${location.hostname}:9090`)
);
```

> **throttle_rate**: 30 Hz 캡. UI는 사람 눈 기준이므로 100 Hz 제어 토픽도 33ms로 충분.

### 4.2 `useRos.js` — 훅

```js
// src/ros/useRos.js
import { useEffect, useState } from "react";
import { ros } from "./bridge";

export function useTopic(def, parser = m => m, initial = null) {
  const [v, set] = useState(initial);
  useEffect(() => ros.subscribe(def, m => {
    try { set(parser(m)); } catch (e) { console.warn(def.name, e); }
  }), [def.name, def.type]);
  return v;
}

export function useRosStatus() {
  const [c, set] = useState(ros.connected);
  useEffect(() => ros.onStatus(set), []);
  return c;
}
```

---

## 5. 컴포넌트 → 토픽 바인딩

### 5.1 속도 / 조향 (간단 케이스)

```jsx
// src/App.jsx 의 일부
import { useTopic } from "./ros/useRos";
import { TOPICS } from "./ros/topics";

const speed = useTopic(TOPICS.velocity, m => m.data, 0);     // km/h
const steer = useTopic(TOPICS.steer,    m => m.data, 0);     // deg
// ...
<SpeedHalf value={speed} max={120} limit={50} />
<SteerDial angle={steer} />
```

### 5.2 측위 (`/localization/info`)

```jsx
const loc = useTopic(TOPICS.loc, m => {
  // custom_msgs 가 없으면 JSON fallback
  if (typeof m.data === "string") return JSON.parse(m.data);
  return {
    lane_id: m.lane_id, sigma_e: m.sigma_e, sigma_n: m.sigma_n, sigma_u: m.sigma_u,
    hdop: m.hdop, sats: m.sats, max_sats: m.max_sats,
    heading: m.heading, fix: m.fix,   // "NONE"|"FLOAT"|"FIX"
  };
}, null);
```

### 5.3 Perception 객체 — ego frame 변환

```jsx
const rawObjs = useTopic(TOPICS.objects, m => m.objects ?? []);
const objects = useMemo(() => {
  if (!rawObjs) return [];
  return rawObjs.map(o => {
    const x = o.pose.position.x;     // forward, m  (ego frame 가정)
    const y = o.pose.position.y;     // left, m
    const v = Math.hypot(o.twist.linear.x, o.twist.linear.y);
    const dist = Math.hypot(x, y);
    return {
      id:   `#${String(o.id).padStart(3, "0")}`,
      kind: classifyKind(o.classification),  // "CAR" | "TRUCK" | "PED"
      x, y, v, dist,
    };
  });
}, [rawObjs]);

<TopDownView objects={objects} ... />
```

> **TF**: 토픽이 map/odom 프레임이면 `tf2_web_republisher` 또는 `ROSLIB.TFClient`로 변환. 차량 자체 노드에서 ego frame로 publish 하도록 운영 측에서 정리하는 게 가장 간단.

### 5.4 V2X / ODD / Health (한 화면 내 다 토픽)

```jsx
const spat = useTopic(TOPICS.spat,
  m => ({ phase: m.phase, remain: m.remain_s }),
  { phase: "—", remain: 0 });

const oddWarn = useTopic(TOPICS.odd_warn, m => m.data || "", "");

const diag = useTopic(TOPICS.diag, m => {
  const out = {};
  for (const s of m.status ?? []) {
    out[s.name] = { level: s.level, msg: s.message };
  }
  return out;
}, {});
```

### 5.5 Trajectory — 좌표 변환

```jsx
const path = useTopic(TOPICS.trajectory, m => {
  return (m.poses ?? []).map(p => [p.pose.position.x, p.pose.position.y]);
}, []);

<TopDownView path={path} ... />
```

`TopDownView` 내부에서 ego frame 미터를 화면 px로 변환:

```js
// scale: 1px = 0.09m → 1:8 디자인 비율과 동일
const PX_PER_M = 11;
const ex = W/2, ey = H*0.62;
const toPx = ([x, y]) => [ex + (-y)*PX_PER_M, ey + (-x)*PX_PER_M];
```

### 5.6 Bag 녹화 — 서비스 호출

```js
// src/ros/topics.js (서비스 추가)
export const SERVICES = {
  rec_start: { name: "/rosbag_recorder/start", type: "std_srvs/Empty" },
  rec_stop:  { name: "/rosbag_recorder/stop",  type: "std_srvs/Empty" },
};

// RecPanel.jsx
import { ros } from "../ros/bridge";
import { SERVICES, TOPICS } from "../ros/topics";

function RecPanel() {
  const state = useTopic(TOPICS.rec_state, m => ({
    rec: m.recording, path: m.path, dur: m.duration_s, size: m.size_bytes,
  }), { rec: false });
  const onClick = () =>
    ros.callService(state.rec ? SERVICES.rec_stop : SERVICES.rec_start, {});
  return (
    <div className="rec-panel">
      <header>● REC <span>ROSBAG</span></header>
      <div className="path">{state.path}</div>
      <button onClick={onClick}>{state.rec ? "STOP" : "START"}</button>
      <footer>{fmtDur(state.dur)}<span>{fmtSize(state.size)}</span></footer>
    </div>
  );
}
```

---

## 6. 시안 자산 이식

기존 `f1v2/F1HMI.jsx` 의 컴포넌트(`SpeedHalf`, `SteerDial`, `TrafficLight`, `Environment`, `Section` …)를 **그대로** `src/components/` 로 옮긴 뒤, 하드코딩된 데이터를 props로 노출하기만 하면 끝납니다.

| 시안 컴포넌트 | 원본 prop | 운용판 추가 prop |
|---|---|---|
| `SpeedHalf` | `value, max, limit` | (그대로) |
| `SteerDial` | `angle` | `gear` (D/R/N/P) |
| `TrafficLight` | `phase, remain` | `valid` (수신 stale 시 회색) |
| `Environment` | (하드코딩) | `objects, path, warn, scaleMperPx` |
| `Localization` | (하드코딩) | `data` (위 5.2의 객체) |
| `SystemHealth` | (하드코딩) | `diag` (위 5.4의 매핑) |

### 6.1 Stale 처리 패턴

```jsx
// 모든 토픽에 수신 시각 트래킹
function useTopicStale(def, parser, initial, staleMs = 500) {
  const [value, setValue] = useState({ v: initial, t: 0 });
  useEffect(() => ros.subscribe(def, m => setValue({ v: parser(m), t: Date.now() })), []);
  const [stale, setStale] = useState(true);
  useEffect(() => {
    const id = setInterval(() => setStale(Date.now() - value.t > staleMs), 100);
    return () => clearInterval(id);
  }, [value.t]);
  return [value.v, stale];
}
```

UI: `stale === true` 면 텍스트 색을 `T3` (회색)으로 dim.

---

## 7. 화면 스케일 / 1600×900 고정

차량 단말은 보통 16:9 1080p 또는 1280×720. 디자인을 그대로 유지하기 위해 viewport-fit 스케일링.

```jsx
// src/App.jsx
function FitFrame({ children }) {
  const [scale, setScale] = useState(1);
  useEffect(() => {
    const fit = () => setScale(Math.min(window.innerWidth/1600, window.innerHeight/900));
    fit(); window.addEventListener("resize", fit);
    return () => window.removeEventListener("resize", fit);
  }, []);
  return (
    <div style={{
      width: "100vw", height: "100vh", background: "#000",
      display: "grid", placeItems: "center", overflow: "hidden",
    }}>
      <div style={{
        width: 1600, height: 900,
        transform: `scale(${scale})`, transformOrigin: "center",
      }}>
        {children}
      </div>
    </div>
  );
}
```

---

## 8. 폰트 / 빌드

### 8.1 셀프 호스팅 (오프라인 차량 환경 대비)

```html
<!-- index.html -->
<style>
  @font-face { font-family:"Inter"; src:url("/fonts/Inter-Variable.woff2") format("woff2-variations"); font-weight:300 700; }
  @font-face { font-family:"JetBrains Mono"; src:url("/fonts/JetBrainsMono-Variable.woff2") format("woff2-variations"); font-weight:400 700; }
  @font-face { font-family:"Pretendard"; src:url("/fonts/Pretendard-Variable.woff2") format("woff2-variations"); font-weight:300 700; }
  body { margin:0; font-family: Inter, Pretendard, system-ui, sans-serif; background:#04060a; }
</style>
```

### 8.2 빌드 + 차량 PC 배포

```bash
# 모니터링 PC에서
npm run build      # → dist/

# 차량 PC로 복사
scp -r dist/* user@vehicle:/var/www/hmi/

# nginx 설정
sudo tee /etc/nginx/sites-available/hmi <<'EOF'
server {
    listen 80 default_server;
    root /var/www/hmi;
    index index.html;
    location / { try_files $uri $uri/ /index.html; }
    # rosbridge 동일 호스트로 reverse-proxy 하고 싶을 때:
    location /ws/ {
        proxy_pass http://127.0.0.1:9090/;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "Upgrade";
    }
}
EOF
sudo ln -sf /etc/nginx/sites-available/hmi /etc/nginx/sites-enabled/
sudo nginx -s reload
```

위처럼 하면 `VITE_ROSBRIDGE_HOST=` 같은 호스트, `bridge.js` 의 url을 `${location.host}/ws/` 로 변경.

---

## 9. 성능

| 항목 | 목표 | 메모 |
|---|---|---|
| 번들 크기 | < 400 KB gzip | roslibjs 60 KB / React 45 KB / 자체 코드 ~80 KB |
| WS 메시지 | < 200/s | `throttle_rate: 33` 로 노드별 30 Hz 캡 |
| Frame | 60 fps | SVG 노드 < 500개. `path`는 `<polyline points>` 로 한 번에 |
| GC | 거의 없음 | `objects` 매핑 시 새 배열만 만들고 문자열 templated 재사용 |

### 9.1 SVG vs Canvas

객체 < 30개, 경로 < 200점이면 **SVG 권장** (인라인 호버/클릭, accessibility, 디버깅 용이). 그 이상이면 `TopDownView`만 `<canvas>` 로 교체.

---

## 10. 보안 / 운용

| 항목 | 권장 |
|---|---|
| 차내 망 격리 | rosbridge 0.0.0.0 노출 시 호스트 방화벽으로 LAN 한정 |
| 외부 원격 | nginx wss + 토큰 (e.g. `?token=...`) 미들웨어 |
| 위험 서비스 | E-STOP, 모드 전환 등은 **서비스만** (토픽 publish 금지), 서버측에서 사용자 권한 검증 |
| 로그 | 클라이언트 콘솔 → `console.* → /ad/hmi_log` Pub 로 차량 로그에 통합 |

---

## 11. 테스트 시나리오

1. **rosbridge 정지 시**: 헤더 ROS 배지가 RED, 모든 라벨 회색.
2. **속도 0 → 60 → 0**: 게이지 부드럽게 추종, 숫자 정수 점프 OK.
3. **객체 64개 동시**: Frame 60 fps 유지.
4. **bag start/stop**: 좌측 REC 토글 시 rosservice 호출 확인 (`rosservice list`).
5. **창 크기 변경**: 1080p → 720p, letterbox만 변할 뿐 디자인 비율 유지.
6. **다중 시청자**: 동일 URL 4개 탭에서 모두 30 Hz 정상.

---

## 12. 점진적 도입 절차

1. 차량 PC에 `rosbridge_websocket` 띄우고 외부에서 `rostopic echo` 대신 브라우저 콘솔에서 `roslibjs`로 토픽 받아보기 (PoC).
2. 시안 HTML(`Ioniq5 HMI F1 KATECH.html`)을 그대로 차량 PC nginx에 올려 정적 페이지로 띄움.
3. `SpeedHalf` 1개부터 ROS 토픽 연결 → 디자인 그대로 살아있는지 확인.
4. 좌측 패널 카드 4개 차례로.
5. `TopDownView` (가장 큼) — perception/planning 토픽 안정성 검증.
6. RecPanel 서비스 호출 + 시스템 헬스 통합.
7. 인증·HTTPS 적용 후 사내망 공개.

---

## 13. 완료 정의 (DoD)

- [ ] `roslaunch pyqt_hmi hmi_web.launch` + nginx 한 번에 기동
- [ ] 차량 LAN의 어떤 PC에서든 `http://<vehicle-ip>/` 로 동일 화면
- [ ] 모든 토픽 stale 표시
- [ ] 1600×900 디자인이 720p~4K 단말 모두에서 비율 유지
- [ ] PyQt 가이드와 **동일 토픽/단위 컨트랙트** 준수 (한 노드만 보면 둘 다 사용 가능)

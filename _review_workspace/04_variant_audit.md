# variant-auditor 보고서

검토 범위: 4개 HTML 진입점의 스크립트 로딩 순서, 의존성, mount target,
variant 간 share/divergence 일관성. JSX 내부 로직과 launch 매핑은 별도 영역.

검토 일자: 2026-05-07
검토 대상 커밋 (HEAD): `5ea2e89` (ioniq5_hmi_dev)

---

## variant 매트릭스

| variant | HTML | 메인 컴포넌트 | mount target | 사용 JSX 모듈 | vendor 추가 |
|---------|------|-------------|-------------|--------------|------------|
| default | `index.html` | `HMIScreen` | `#root` (1600×900 frame) | `hmi/components.jsx`, `hmi/ros_bridge.jsx`, `hmi/HMIScreen.jsx` | (없음, 기본만) |
| f1      | `index_f1.html` | `F1HMIScreen` | `#root` (in `#stage` in `#fit`) | `hmi/ros_bridge.jsx`, `f1/F1HMI.jsx`, `f1/F1HMIScreen.jsx` | (없음, 기본만) |
| threejs | `index_threejs.html` | inline `ThreeJSScreen` (HTML 안에 정의) | `#root` (full viewport) | `hmi/ros_bridge.jsx`, `threejs/types.js`, `threejs/{ThreeScene,MapLayers,TrackBoxes,TrackPointClouds,EgoMesh,CameraController,ControlPanel}.jsx` | `vendor/three.min.js`, `vendor/GLTFLoader.js` |
| threejs_f1 | `index_threejs_f1.html` | `ThreejsF1Screen` | `#root` (in `#stage` in `#fit`) | `hmi/ros_bridge.jsx`, `f1/F1HMI.jsx`, `threejs/types.js`, `threejs/{ThreeScene,MapLayers,TrackBoxes,TrackPointClouds,EgoMesh,CameraController,ControlPanel}.jsx`, `threejs_f1/ThreejsF1Screen.jsx` | `vendor/three.min.js`, `vendor/GLTFLoader.js` |

공통:
- 모든 variant: vendor 4종(react, react-dom, babel, roslib) 로드 후 babel JSX 로드.
- 모든 variant: `RosProvider` 로 래핑, `wsUrl = ws://${hostname || 'localhost'}:9090` 동일.
- 모든 variant: `hmi/ros_bridge.jsx` 공유 (RosProvider + 8 hooks).
- 모든 variant: charset utf-8, html lang="ko" 일관.

---

## 발견 항목

| #  | severity | 위치 | 발견 | 권장 조치 |
|----|----------|------|------|----------|
| V1 | medium | `hmi/ros_bridge.jsx:247-252` | `Object.assign(window, {...})` 가 `useJsonTopic` 을 export 하지 않음. threejs 측 5개 컴포넌트(`MapLayers`, `TrackBoxes`, `TrackPointClouds`, `EgoMesh`, `CameraController`)가 `useJsonTopic` 을 자유 식별자로 참조 → 같은 글로벌 스크립트 환경(브라우저가 babel transform 후 `appendChild` 한 `<script>`)에 `function useJsonTopic` 이 등록되어 작동하지만, 다른 export (`RosProvider`, `useRosState` 등) 와 export 정책이 불일치하며 향후 strict mode/ESM 마이그레이션 시 깨질 수 있음. | `Object.assign(window, { ... useJsonTopic, ... })` 에 `useJsonTopic` 추가하여 export 정책 일관성 유지. (정책: 다른 모듈이 참조하는 모든 hook/컴포넌트는 `window.X` 로 명시 export.) |
| V2 | low | 4개 HTML 전반 | `<meta name="viewport">` 누락. 4 HTML 모두 누락이라 일관은 하지만, 풀스크린 1600×900 stage 가 모바일/노트북 디스플레이에서 의도대로 스케일 안 됨. f1/threejs_f1 는 자체 `applyFit()` 으로 보완하나 default/threejs 는 그렇지 않음. | default/threejs 만 `<meta name="viewport" content="width=1600, initial-scale=1">` 추가 검토. (스코프 밖이면 보고만.) |
| V3 | low | `index_threejs.html:7-8` (없음) | `index.html`/`index_f1.html`/`index_threejs_f1.html` 은 Google Fonts (Inter, JetBrains Mono, Pretendard) 를 preconnect+load 하지만, `index_threejs.html` 만 누락. inline style 의 `font-family: "Inter", system-ui, sans-serif;` 는 fallback 으로만 동작 → threejs variant 에서 system 폰트로 fallback. 의도된 차이일 수도 있으나 표면적 분기 사유 없음. | 의도면 주석 추가, 아니면 다른 3 HTML 과 동일하게 fonts CDN 링크 추가. |
| V4 | low | `index.html:11`, `index_f1.html:10`, `index_threejs_f1.html:9` | Google Fonts URL 의 `JetBrains+Mono:wght=` 파라미터가 default 는 `400;500;600` 이고 f1/threejs_f1 는 `400;500;600;700`. 700 weight 차이만 있음. 의도된 차이인지 우연한 분기인지 불명. | f1/threejs_f1 가 정말 weight 700 을 사용하면 유지, 아니면 default 와 동일 weight set 로 통일. (font weight 사용처는 frontend-auditor 영역에서 확정.) |
| V5 | info | `index.html:13` | `<link rel="stylesheet" href="hmi/styles.css" />` 가 default variant 에서만 로드됨. f1/threejs/threejs_f1 는 모두 inline `<style>` 만 사용. styles.css(13.7KB) 는 default 전용 자산으로 명확히 분리됨 → 의도된 분기로 판단. | 조치 없음. (참고: styles.css 의 미사용 클래스 정리는 frontend-auditor 영역.) |
| V6 | info | `index.html:50`, `index_f1.html:69` vs `index_threejs.html:88`, `index_threejs_f1.html:86` | 인라인 babel 스크립트 안에서 컴포넌트 참조 스타일이 두 갈래: default/f1 은 bare identifier (`<RosProvider>`, `<HMIScreen>`), threejs/threejs_f1 은 `<window.RosProvider>` prefix. 둘 다 babel 변환 후 글로벌 스크립트로 평가되므로 식별자 lookup → window 모두 가능 → 기능 동일. 그러나 스타일 일관성 부재. | 1) bare identifier 통일, 또는 2) `window.X` prefix 통일 중 하나 선택. 기능 영향 없음. |
| V7 | info | `index_threejs.html:23` | `<div id="status">init…</div>` 가 React tree 외부에 있음. mount 직후 라인 93 에서 `document.getElementById('status').textContent = 'THREE r${THREE.REVISION} · waiting for /hmi/threejs/map…'` 로 1회 갱신 후 더 이상 업데이트 안 됨. 삭제하거나 ThreeJSScreen 내부로 옮기면 정합성↑. | 조치는 frontend-auditor 영역. |
| V8 | info | `index_threejs.html:9-12` vs `index_threejs_f1.html:11-39` | threejs variant 는 `#scene { position: fixed; inset: 0 }` (full viewport), threejs_f1 는 `#scene { position: absolute; inset: 0 }` (embedded into F1 main grid). 두 HTML 모두 `<canvas id="scene">` 을 ThreeScene.jsx 가 동일한 id 로 출력하므로 같은 컴포넌트가 두 다른 CSS 컨텍스트에서 재사용됨. ThreeScene.jsx:54-69 의 resize 로직(`host.clientWidth>0` 분기)이 두 모드를 모두 핸들링. | 조치 없음. 의도된 분기 OK. |

발견 건수: medium 1, low 3, info 4 (총 8건).

---

## share/divergence 정합성 요약

### A. 의존성 순서 (forward reference) — 정상

| HTML | 검증된 순서 |
|------|-----------|
| index.html | react→react-dom→babel→roslib→components.jsx→ros_bridge.jsx→HMIScreen.jsx→inline mount. HMIScreen 이 components 의 `SpeedGauge`/`Vehicle`/...등을 참조하므로 components.jsx 가 먼저 로드되어야 함 → OK. |
| index_f1.html | react→react-dom→babel→roslib→ros_bridge.jsx→F1HMI.jsx(`window.F1Tokens=T` export)→F1HMIScreen.jsx(`const T = window.F1Tokens` import)→inline mount. F1HMIScreen 이 top-level 에서 `window.F1Tokens` 평가 → F1HMI.jsx 먼저 OK. |
| index_threejs.html | three→GLTFLoader→react→react-dom→babel→roslib→ros_bridge.jsx→types.js(classic)→ThreeScene→MapLayers→TrackBoxes→TrackPointClouds→EgoMesh→CameraController→ControlPanel→inline ThreeJSScreen. types.js 의 `window.OBJ_PALETTE`/`LAYER_STYLE`/`DEFAULT_LAYER_VIS` 가 모든 후속 컴포넌트 로드 전에 등록 → OK. |
| index_threejs_f1.html | three→GLTFLoader→react→react-dom→babel→roslib→ros_bridge.jsx→F1HMI.jsx(`window.F1Tokens=T`, `window.F1HMIShell=F1HMIShell`)→types.js→ThreeScene→...→ControlPanel→ThreejsF1Screen.jsx(top-level `const T_F1 = window.F1Tokens` 평가)→inline mount. F1HMI.jsx 가 ThreejsF1Screen 보다 먼저 → OK. |

### B. 마운트 일관성 — 정상

- 4 HTML 모두 `<div id="root">` 에 React 마운트.
- threejs variants 의 `<canvas id="scene">`, `<div id="panel">` 은 React 트리가 동적 생성 (정적 HTML 에 없음) → 컴포넌트 단위로 캡슐화 OK.
- f1/threejs_f1 의 `#fit`/`#stage` 만 1600×900 FitFrame 컨테이너로 정적 정의 → React mount 가 `#root` 에 들어가는 패턴 일관.

### C. RosProvider/wsUrl — 정상

- 4 variant 모두 `RosProvider url={wsUrl}` 으로 래핑.
- `wsUrl` 형식 동일: 템플릿 리터럴 `ws://${hostname || 'localhost'}:9090`.
- `RosProvider` 는 `hmi/ros_bridge.jsx:247` 의 `Object.assign(window, ...)` 으로 4 variant 모두 동일하게 export.

### D. 미사용/누락 자산 — 없음

- vendor 디렉토리 자산: react, react-dom, babel, roslib(공통) + three, GLTFLoader(threejs/threejs_f1 전용) + `hyundai_ioniq_5_-_lowpoly/`(EgoMesh.jsx 가 threejs/threejs_f1 에서 사용). 모두 사용됨. unused vendor 없음.
- web/ 트리 JSX 14개 모두 적어도 한 variant 에서 로드됨 (orphan 없음):
  - `hmi/components.jsx` → index.html 만
  - `hmi/HMIScreen.jsx` → index.html 만
  - `hmi/ros_bridge.jsx` → 4 모두
  - `f1/F1HMI.jsx` → index_f1.html, index_threejs_f1.html
  - `f1/F1HMIScreen.jsx` → index_f1.html 만
  - `threejs_f1/ThreejsF1Screen.jsx` → index_threejs_f1.html 만
  - `threejs/{types.js + 7 jsx}` → index_threejs.html, index_threejs_f1.html

### E. type="text/babel" 누락 — 없음

`type="text/babel"` 누락된 JSX `<script src=>` 없음.
classic `<script src=>` 는 vendor 라이브러리 5종 + `threejs/types.js` (1개) 만 — 모두 plain JS 가 맞음 (JSX 문법 미사용).

---

## 별도 영역 위임 사항

다음 항목은 본 auditor 영역 밖이며 다른 auditor 가 후속 검토:
- **JSX 내부 로직**: useEffect 의존성, 컴포넌트 prop 시그니처, render 안전성, 클리어업 → frontend-auditor.
- **bridge 노드와 토픽 매칭**: `/hmi/threejs/map`, `/hmi/threejs/tracks`, `/hmi/state` 등 publish 측 정합성 → bridge-auditor.
- **launch → variant 매핑**: `katech_test.launch` / `diagnostic_only.launch` 의 `web_hmi` 인자 → launch-auditor.

---

## 이전 산출물 처리

`_workspace/04_variant_audit.md` 는 본 보고서로 신규 작성 (선행 산출물 없음).

# frontend-auditor 보고서

대상: `src/visualization/web_hmi/web/{threejs,threejs_f1,f1,hmi}/` JSX 컴포넌트들.
HTML 진입점은 variant-auditor 담당이므로 본 보고서에서 진입점 자체는 다루지 않고,
JSX 내부 React 패턴 / 토픽 구독 / Three.js cleanup / 좌표 일관성에 집중.

정적 분석 한계: 브라우저 런타임 동작은 검증하지 못했다. 이하 발견은 grep + 코드 읽기 기반.

---

## A. 컴포넌트 인벤토리

| 파일 | 컴포넌트 (export) | 구독 토픽 (직접) | 비고 |
|------|------------------|------------------|------|
| `hmi/ros_bridge.jsx` | `RosProvider`, `useRosConnection`, `useJsonTopic`, `useTopicStale`, `useRosState`, `useDiagnostics`, `useTopicHz`, `useObjects`, `usePopup`, `useTraffic`, `useBag`, `useMap`, `buildTrafficObjs` | (정의) `/hmi/state`, `/hmi/diagnostics`, `/hmi/topic_hz`, `/hmi/objects`, `/hmi/popup`, `/hmi/traffic`, `/hmi/bag`, `/hmi/map` | 단일 정의지점 |
| `hmi/components.jsx` | `SpeedGauge`, `Vehicle`, `VehicleTop`, `DetectionOverlay`, `DetectionOverlayTop`, `RangeRings`, `RangeRingsTop`, `TrafficObjects`, `PlanPath` | – (순수 SVG) | 모두 HMIScreen이 사용 |
| `hmi/HMIScreen.jsx` | `HMIScreen` | (간접) `/hmi/state`, `/hmi/diagnostics`, `/hmi/topic_hz`, `/hmi/objects`, `/hmi/popup`, `/hmi/bag` | `useMap`/`useTraffic`은 사용 안 함 |
| `f1/F1HMI.jsx` | `F1HMIShell`, `F1Tokens` | – (순수 presentational) | 모든 데이터는 props |
| `f1/F1HMIScreen.jsx` | `F1HMIScreen` | (간접) `/hmi/state`, `/hmi/diagnostics`, `/hmi/topic_hz`, `/hmi/objects`, `/hmi/popup`, `/hmi/traffic`, `/hmi/bag`, `/hmi/map` | F1HMIShell 어댑터 |
| `threejs/ThreeScene.jsx` | `ThreeScene`, `ThreeContext`, `useThree` | – | scene/camera/renderer 컨텍스트 |
| `threejs/MapLayers.jsx` | `MapLayers` | `/hmi/threejs/map` (line 92) | – |
| `threejs/TrackBoxes.jsx` | `TrackBoxes` | `/hmi/threejs/tracks` (line 49), `/hmi/threejs/map` (line 50), `/hmi/state` (line 51 useRosState) | – |
| `threejs/TrackPointClouds.jsx` | `TrackPointClouds` | `/hmi/threejs/tracks` (line 42) | – |
| `threejs/EgoMesh.jsx` | `EgoMesh` | `/hmi/threejs/map` (line 85), `/hmi/state` (line 86 useRosState) | – |
| `threejs/CameraController.jsx` | `CameraController` | `/hmi/threejs/map` (line 17), `/hmi/state` (line 18 useRosState) | – |
| `threejs/ControlPanel.jsx` | `ControlPanel` | – (순수 DOM) | – |
| `threejs/types.js` | `OBJ_PALETTE`, `OBJ_DIMS`, `KNOWN_TYPES`, `deriveType`, `LAYER_STYLE`, `DEFAULT_LAYER_VIS` | – (상수) | – |
| `threejs_f1/ThreejsF1Screen.jsx` | `ThreejsF1Screen` | (간접) `/hmi/state`, `/hmi/diagnostics`, `/hmi/topic_hz`, `/hmi/objects`, `/hmi/popup`, `/hmi/traffic`, `/hmi/bag`, `/hmi/map` (F1HMIScreen과 동일 hook 셋) | F1 chrome + Three.js 자식 |

### useJsonTopic 직접 호출 인벤토리 (bridge-auditor 교차 비교용)

| 토픽 키 | 호출 컴포넌트 (file:line) | 디폴트 |
|---------|--------------------------|--------|
| `/hmi/state` | `hmi/ros_bridge.jsx:165` (useRosState) | `STATE_DEFAULT` |
| `/hmi/diagnostics` | `hmi/ros_bridge.jsx:174` (useDiagnostics) | `DIAG_DEFAULT` |
| `/hmi/topic_hz` | `hmi/ros_bridge.jsx:178` (useTopicHz) | `{}` |
| `/hmi/objects` | `hmi/ros_bridge.jsx:182` (useObjects) | `{count:0,data:[]}` |
| `/hmi/popup` | `hmi/ros_bridge.jsx:186` (usePopup) | `{text:'',severity:'info'}` |
| `/hmi/traffic` | `hmi/ros_bridge.jsx:190` (useTraffic) | `{color:0,time_decisec:0,look_at:null}` |
| `/hmi/bag` | `hmi/ros_bridge.jsx:194` (useBag) | `{recording:false,info:''}` |
| `/hmi/map` | `hmi/ros_bridge.jsx:200` (useMap) | `{polylines:[]}` |
| `/hmi/threejs/map` | `threejs/MapLayers.jsx:92` <br> `threejs/TrackBoxes.jsx:50` <br> `threejs/EgoMesh.jsx:85` <br> `threejs/CameraController.jsx:17` | `null` (4곳 모두) |
| `/hmi/threejs/tracks` | `threejs/TrackBoxes.jsx:49` <br> `threejs/TrackPointClouds.jsx:42` | `null` |

`/hmi/threejs/map`은 4개 컴포넌트가 각자 직접 구독한다 (래퍼 hook 부재). rosbridge 측에서는 latched 발행이라 비용은 거의 없으나, 후속 리팩토링 후보.

### 발행 측(브리지) 매트릭스

| 발행 토픽 | 발행자 | 비고 |
|-----------|-------|------|
| `/hmi/state` | `web_hmi_bridge.py:80` | 10 Hz |
| `/hmi/diagnostics` | `web_hmi_bridge.py:81` | 10 Hz |
| `/hmi/objects` | `web_hmi_bridge.py:82` | 변경 시 |
| `/hmi/popup` | `web_hmi_bridge.py:83` | latch |
| `/hmi/traffic` | `web_hmi_bridge.py:84` | latch |
| `/hmi/bag` | `web_hmi_bridge.py:85` | latch |
| `/hmi/topic_hz` | `web_hmi_bridge.py:86` | 1 Hz |
| `/hmi/map` | `web_hmi_bridge.py:87` | latch |
| `/hmi/threejs/map` | `web_hmi_threejs_bridge.py:83` | latch |
| `/hmi/threejs/tracks` | `web_hmi_threejs_bridge.py:86` | 10 Hz |

런치 (`web_hmi.launch:35,48`) 기준 두 브리지 모두 활성. variant=threejs/threejs_f1일 때만 threejs 브리지 추가 기동. 죽은 구독 / 발행 없음 토픽은 발견되지 않음.

### useRosState 디폴트 일관성

| 호출 위치 | useRosState() 호출 | 디폴트 |
|-----------|-------------------|--------|
| `f1/F1HMIScreen.jsx:151` | ✓ | `STATE_DEFAULT` (정의 한 곳) |
| `hmi/HMIScreen.jsx:159` | ✓ | 동일 |
| `threejs_f1/ThreejsF1Screen.jsx:97` | ✓ | 동일 |
| `threejs/TrackBoxes.jsx:51` | ✓ | 동일 |
| `threejs/EgoMesh.jsx:86` | ✓ | 동일 |
| `threejs/CameraController.jsx:18` | ✓ | 동일 |

`useRosState`는 wrapper hook 안에 `STATE_DEFAULT` 디폴트를 고정 — 모든 호출자가 같은 디폴트를 받음 ✓

---

## B. 발견 항목

| # | severity | 위치 | 발견 | 권장 조치 |
|---|----------|------|------|----------|
| F1 | low | `hmi/HMIScreen.jsx:2` | `globals` 주석에 `useTraffic` 명시되었지만 본문(149-352)에서 호출 없음. 죽은 import 표기. (변수 선언 없음, 런타임 영향 없음) | globals 주석에서 `useTraffic` 제거. |
| F2 | low | `hmi/ros_bridge.jsx:147-152` | `useTopicStale` 정의·export 되었으나 전체 web/ 내 호출 0건 | (a) 향후 사용 예정이면 보존, (b) 아니면 제거. 1줄 주석만이라도 의도 명시 권장. |
| F3 | low | `threejs/TrackBoxes.jsx:55-63`, `EgoMesh.jsx:139-148`, `MapLayers.jsx:122-127`, `TrackPointClouds.jsx:65-67`, `CameraController.jsx:34-63` | `React.useEffect(...)` deps array 누락 → 매 렌더 fire | `EgoMesh`, `MapLayers.jsx:122`, `TrackPointClouds.jsx:65`은 코드 주석에 의도("track ego pose every render", "Apply visibility on each render")가 있어 의도된 패턴. `CameraController.jsx:34-63`은 의도 주석이 없음. (a) 의도면 `// eslint-disable-next-line react-hooks/exhaustive-deps` + 한 줄 주석, (b) 아니면 deps `[three, map, ego, mode, zoom]` 명시. |
| F4 | info | `threejs/TrackBoxes.jsx:108` | tracks 업데이트 effect의 deps가 `[three, tracks, showBoxes, showHeading]` — `showIds`/맵 변경 누락. `showIds`는 line 120 주석 "S9 reserved"로 명시적 미사용이라 OK. tracks 안에 origin 변환 로직은 없으므로 map deps 누락도 OK. | 변경 불필요. (검증 차원 기록) |
| F5 | info | `threejs/TrackBoxes.jsx:47`, `index_threejs.html:67`, `threejs_f1/ThreejsF1Screen.jsx:175` | 부모가 `showIds={false}` prop 전달, 자식은 미사용 (S9 사전 placeholder) | 의도된 placeholder. 변경 불필요. 단, `// TODO(S9): wire showIds`가 있으면 더 명확. |
| F6 | low | `threejs/MapLayers.jsx:122-127` | visibility 적용 effect가 deps 없음 → groupsRef.current entries는 첫 렌더 후 채워지는데 빌드 effect (line 95) 와 동일 빈 deps라면 race 우려는 없으나 layerVisibility prop 변경 시에만 fire하면 충분 | deps `[layerVisibility]` 추가가 명시적 (단, 현재도 매 렌더 fire라 동작상 문제 없음). |
| F7 | low | `threejs/EgoMesh.jsx:130-135` | unmount cleanup에서 `arrow.dispose && arrow.dispose()` 호출하나, ArrowHelper에는 `dispose()` 메서드 없음 (line 134). `disposeTree(group)`이 group 자식을 traverse해 geometry/material 정리하므로 실질 누수는 없음. | dead-code 정도. 명확히 하고 싶으면 line 134 제거, 아니면 보존 무방. |
| F8 | info | `threejs/TrackBoxes.jsx:73` | size_x/size_y 변경 감지에 `slot.size_x`, `slot.size_y` 비교 — slot 객체에는 line 86에서 저장됨 ✓ | 일관성 확인됨. |
| F9 | info | `threejs/TrackBoxes.jsx:111-118` | unmount cleanup이 deps `[three]`로 묶여 three 변경 시에도 dispose. ThreeScene이 한 번만 mount되면 컴포넌트 lifetime 동안 three 참조 안정 | 의도된 패턴. |
| F10 | info | Three.js 메모리 cleanup 일관성 | `ThreeScene.jsx:90-96` (전역 traverse), `MapLayers.jsx:80-87` (`disposeGroup`), `TrackBoxes.jsx:37-44` (`disposeMesh`), `TrackPointClouds.jsx:34-38` (`disposeSlot`), `EgoMesh.jsx:20-27` (`disposeTree`) — 모든 컴포넌트가 unmount 시 geometry/material `.dispose()` 호출 | OK ✓ |
| F11 | info | 좌표계 가정 일관성 | `ThreeScene.jsx:27` `scene.scale.z = -1` (남←→북 반전). 모든 trackGroup/egoGroup 자식은 ego pose `(eEast-origin[0], 0, eNorth-origin[1])`로 변환되며 yaw는 `rotation.y = -eYaw`. CameraController는 `cam`이 scene 외부이므로 직접 `ezWorld = -(eNorth-origin[1])`로 보정. TrackPointClouds는 lidar→Three (x→x, z→y, y→z) 변환 후 trackGroup 변환을 받음 | 일관됨. |
| F12 | low | `threejs/TrackBoxes.jsx:90`, `TrackPointClouds.jsx:117` 주석 vs 실제 | TrackBoxes track의 `trk.x/y` 그대로 X/Z에 매핑 (높이 0). TrackPointClouds는 Y/Z swap. 즉 같은 ego frame에서 들어오는데 한 쪽은 swap, 한 쪽은 안 함 — track centroid는 평면 위치만 다루고 cloud는 3D라 height가 필요해서 다름. 문서 부재 | TrackBoxes 주석에 "tracks are planar; centroid 사용" 한 줄 추가하면 의도 명확. |
| F13 | medium | `threejs/MapLayers.jsx:122-127` | `layerVisibility` prop이 변경되어도 deps 누락이라 매 렌더 fire ⇒ 동작은 되지만 시각화 토글이 부모 setState 발생 시점이 아닌 매 렌더 동기화. 부모 (`ThreeJSScreen`/`ThreejsF1Screen`)의 다른 state 변경 시에도 불필요 호출. 비용은 작음 (`Object.entries(groupsRef.current)` 순회). | deps `[layerVisibility]` 명시. |
| F14 | low | `threejs/CameraController.jsx:34-63` | wheel 이벤트로 `zoom` state 갱신 + 해당 effect는 매 렌더 fire이므로 zoom 변경 시 일관 동작. 그러나 effect 내부에서 `Math.cos/sin(eYaw)`을 매 렌더 호출. 60fps 기준 무시 가능. | 변경 불필요. (정밀화 시 `useMemo`로 yaw 회전 행렬 캐시) |
| F15 | low | `f1/F1HMI.jsx:303-340` | `F1HMIShell` props 디폴트값 다수. `mainContent` props는 옵션 (line 339, line 555-565에서 fallback to `Environment`). 부모는 항상 `mainContent` 또는 `objs/oddBanner/...` 둘 중 하나를 경로로 채움. | 정합. |
| F16 | info | `f1/F1HMIScreen.jsx:206-215` vs `threejs_f1/ThreejsF1Screen.jsx:147-156` | 두 어댑터 모두 비슷한 bottom strip 빌드 로직. ThreejsF1Screen이 `TRACKS` 카운트를 별도로 추가, F1HMIScreen은 `PLAN-H`를 추가. 의도된 변형 | 변경 불필요. |
| F17 | info | `hmi/HMIScreen.jsx:38-46` | `epsg5179ToWgs` 단일점 선형화. 주석에 "intentionally approximate" 명시 | OK. |
| F18 | low | `hmi/ros_bridge.jsx:75-81` | `lastMessageAgeMs` 인터벌 500ms로 ref→state 동기화. 주석은 "once per second"라 명시되어 있는데 실제 500ms | 주석 또는 값 수정. 동작 영향 없음. |
| F19 | low | 캐시 버스터 0건 | 모든 4개 HTML(`index.html`, `index_f1.html`, `index_threejs.html`, `index_threejs_f1.html`)에 `<script src="...?v=YYYYMMDDx">` 사용 사례 0건 | 일관된 부재이므로 stale 위험은 변경 시 모든 파일에 동시 부여하면 됨. variant-auditor 영역. |
| F20 | info | `threejs_f1/ThreejsF1Screen.jsx:159-198` | 동일한 ThreeJSScreen 상태 셋업이 `index_threejs.html:48-83` (인라인) 와 `threejs_f1/ThreejsF1Screen.jsx`에 중복. 7개 useState + onLayerVis useCallback + 동일 props chain | 공통 hook (예: `useThreeJSPanelState()`)으로 추출 후보. variant-auditor 협의 필요. |

---

## 요약

- 발행/구독 키 미스매치 0건 (모든 useJsonTopic 키가 두 브리지 중 하나에서 발행됨).
- 죽은 구독 0건. 미사용 export 1건 (`useTopicStale`).
- 메모리 cleanup은 5개 Three.js 컴포넌트 모두 일관되게 dispose 적용.
- 좌표계 가정(`scene.scale.z=-1`, `rotation.y=-yaw`)은 모든 컴포넌트가 일관 적용.
- React deps 누락은 의도된 "매 렌더 ego pose 동기화" 패턴이 다수이나 명시 주석/deps array 보강 필요 케이스 1건 (CameraController).
- ThreeJSScreen 상태(`index_threejs.html` 인라인) ↔ `ThreejsF1Screen` 사이 동일 패턴 중복 → 추후 추출 후보.

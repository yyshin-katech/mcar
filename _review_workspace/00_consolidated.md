# web_hmi 검토 결과 통합 보고서

**검토 일자:** 2026-05-07
**검토 대상 커밋:** `5ea2e89` (ioniq5_hmi_dev)
**검토 영역:** launch / bridge / frontend / variant (4 auditor 병렬)

---

## 총계

| severity | 건수 |
|----------|------|
| critical | 0    |
| medium   | 6    |
| low      | 18   |
| info     | 24   |
| **합계** | **48** |

영역별:
- launch-auditor: 0건 (모두 정합)
- bridge-auditor: 20건 (medium 4 / low 6 / info 10)
- frontend-auditor: 20건 (medium 1 / low 9 / info 10)
- variant-auditor: 8건 (medium 1 / low 3 / info 4)

---

## Critical / Major (즉시 조치)

| # | 영역 | 위치 | 발견 | 권장 조치 |
|---|------|------|------|----------|
| C1 | bridge ↔ frontend | `web_hmi_threejs_bridge.py:179-185` ↔ `threejs/TrackPointClouds.jsx:112-118` | **교차 검증 결과**: 브리지가 `points` 페이로드에 NaN 좌표를 그대로 직렬화(`json.dumps`는 `NaN` 토큰을 출력 → 표준 JSON 파서 거부; rosbridge/브라우저 JSON.parse 동작 비표준). TrackPointClouds.jsx는 `arr[j], arr[j+1], arr[j+2]`를 그대로 BufferGeometry position에 넣음 → NaN vertex가 깨진 frustum culling/draw call 유발 가능. 라이브에서 NaN 다수 확인 (cloud_indices가 invalid 영역 가리킴). | **브리지 측 수정**: `_slice_points`에서 `np.isfinite(pts).all(axis=1)` 마스크로 NaN 제거 후 mm round. (frontend 측에서 처리하면 매 프레임 비용↑) |
| C2 | bridge | `web_hmi_threejs_bridge.py:148-186` | `/percept_topic` (~7.3 Hz) ↔ `/fusion_lidar_points` (~1.5 Hz 주석 명시) 시간 동기 미검증. 동일 클라우드를 4-5번 슬라이스하거나 percept의 `cloud_indices`가 stale 클라우드 인덱스 공간을 가리킬 가능성. C1의 NaN 빈발과 일관됨. | `lf.timestamp.data` ↔ `cloud msg.header.stamp` 차이를 1회 logwarn_throttle. 차이가 크면 `points` 미포함 또는 가장 가까운 클라우드를 짝짓는 round-robin queue. |
| C3 | bridge | `web_hmi_threejs_bridge.py:140-146, 156, 176` | `try/except AttributeError: return`이 silent drop. 비정상 percept 메시지 진단 불가. | `rospy.logwarn_throttle(5.0, ...)` 추가하여 디버깅 가능. |
| C4 | variant ↔ frontend | `hmi/ros_bridge.jsx:247-252` | `Object.assign(window, {...})`에 `useJsonTopic` 누락. threejs 5개 컴포넌트(MapLayers/TrackBoxes/TrackPointClouds/EgoMesh/CameraController)가 자유 식별자로 참조 → babel 변환 후 글로벌 스코프 공유로 우연히 동작. ESM/strict 마이그레이션 시 깨짐. | `Object.assign(window, { ..., useJsonTopic, ... })` 추가하여 export 정책 일관성 유지. |
| C5 | frontend | `threejs/MapLayers.jsx:122-127` | `layerVisibility` prop 변경 적용 effect의 deps 누락 → 매 렌더 fire. 동작은 OK이나 부모의 무관한 state 변경 시에도 불필요 호출. | deps `[layerVisibility]` 명시. |
| C6 | frontend | `threejs/CameraController.jsx:34-63` | wheel→zoom effect deps 누락. 의도 주석 부재. | (a) 의도면 `// eslint-disable react-hooks/exhaustive-deps` + 한 줄 주석, (b) 아니면 deps `[three, map, ego, mode, zoom]` 명시. |

---

## Minor / Info (선택 개선)

bridge:
- B3 (low) `_slice_points` cloud_xyz race - GIL atomic으로 안전, 코멘트만 권장
- B4 (low) `web_hmi_bridge.py:117-120` Timer shutdown 미명시 - rospy.spin이 정리하나 명시성↓
- B5/B6 (low) emit/dedup 패턴 - 동작상 문제 없음
- B10 (medium→low로 재분류) `/hmi/state` queue=2 - latest-state 의미상 OK
- B12 (info) `gps.lon_std/lat_std` 단위 .msg에 미명시 - 별도 영역
- B13 (low) cloud snapshot 비원자 - GIL 안전, `n_cloud = cloud.shape[0]`로 제거 가능
- B7-B9, B14-B16, B18-B20 (info): import fallback / 좌표계 일관성 / 토픽 매트릭스 등 정합 확인

frontend:
- F1 (low) `hmi/HMIScreen.jsx:2` globals 주석에 useTraffic 명시되었으나 미호출 → 주석 제거
- F2 (low) `useTopicStale` 정의/export 되었으나 호출 0건 → 의도 불명, 제거 또는 주석 권장
- F3 (low) useEffect deps 누락 5건 (TrackBoxes/EgoMesh/MapLayers/TrackPointClouds/CameraController) - 4건은 의도된 "매 렌더 ego pose 동기화" 패턴, deps 또는 주석 보강 권장
- F7 (low) `EgoMesh.jsx:134` ArrowHelper.dispose() 호출하나 메서드 부재 - dead-code, disposeTree로 실제 정리됨
- F12 (low) TrackBoxes(평면) ↔ TrackPointClouds(3D) 좌표 swap 비대칭 - 의도, 주석 추가 권장
- F14 (low) wheel 이벤트 yaw cos/sin 매 렌더 - 60fps 무시 가능, useMemo 후보
- F15 (low) F1HMIShell mainContent fallback 정합
- F18 (low) ros_bridge.jsx:75-81 인터벌 500ms인데 주석 "once per second" - 주석 또는 값 수정
- F19 (low) 4 HTML 모두 cache buster 부재 - 일관된 부재 (variant 영역)
- F4-F6, F8-F11, F16, F17, F20 (info): tracks deps / 좌표계 일관성 / mainContent props 등 정합 확인

variant:
- V2 (low) viewport meta 4 HTML 모두 누락 - 일관, default/threejs는 fit 보완 부재
- V3 (low) `index_threejs.html`만 Google Fonts 누락 - 의도 불명
- V4 (low) JetBrains Mono weight set 미세 차이 (default 600 vs f1/threejs_f1 700) - 의도 불명
- V5-V8 (info): styles.css 분리 / window prefix 스타일 일관성 / ThreeJSScreen 중복 등 정합 확인

---

## 교차 검증 PASS

1. **토픽 발행/구독 매칭** (10/10)
   - 발행: `/hmi/state`, `/hmi/diagnostics`, `/hmi/topic_hz`, `/hmi/objects`, `/hmi/popup`, `/hmi/traffic`, `/hmi/bag`, `/hmi/map`, `/hmi/threejs/map`, `/hmi/threejs/tracks` (10건)
   - 구독: 동일 10건 + 죽은 구독 0건 + 발행 없는 토픽 구독 0건
   - JSON shape 키 미스매치 0건

2. **launch ↔ scripts 정합**
   - 4 노드(`web_hmi_bridge`, `web_hmi_threejs_bridge`, `web_hmi_server`, `web_hmi_open_browser`) 모두 `scripts/`에 실재 + `catkin_install_python` 등록 + 실행권한 +x
   - launch arg ↔ rospy.get_param 키 1:1 매칭
   - 외부 의존(`rosbridge_server`, `gps_system_localizer/...`) 모두 실재

3. **variant HTML ↔ Component 매핑**
   - 4 HTML 진입점 ↔ 메인 컴포넌트(HMIScreen / F1HMIScreen / inline ThreeJSScreen / ThreejsF1Screen) 일관
   - 14개 JSX 모두 최소 한 variant에서 로드됨 (orphan 0건)
   - vendor 라이브러리 사용 일관 (three/GLTFLoader는 threejs 변형만)

4. **메시지 타입/필드 정합**
   - 모든 import + Subscriber 타입 ↔ `.msg` 정의 1:1 매칭
   - typo 필드명(`trakcer_id`, `hassupplmentinfo`)도 .msg 그대로 사용 (정합)
   - 좌표계 단위 일관: EPSG:5179 [m], yaw [rad], scene flip(`scale.z=-1`)+`rotation.y=-yaw` 일관

5. **메모리 cleanup 일관성**
   - 5개 Three.js 컴포넌트(ThreeScene/MapLayers/TrackBoxes/TrackPointClouds/EgoMesh) 모두 unmount 시 geometry/material `.dispose()` 호출

---

## 다음 단계 제안

**우선순위 1 — 즉시 수정 권장 (Critical/Major):**
1. **C1 NaN 필터** (`web_hmi_threejs_bridge.py` `_slice_points`): `np.isfinite` 마스크 추가. 라이브에서 다수 NaN 확인되어 BufferGeometry 깨짐 가능성 있음.
2. **C4 useJsonTopic export** (`hmi/ros_bridge.jsx:247-252`): `Object.assign(window, ...)`에 추가. 1줄 변경.

**우선순위 2 — 진단/안정성:**
3. **C2 percept ↔ cloud 시간 동기 측정**: `lf.timestamp.data ↔ cloud msg.header.stamp` 차이 logwarn_throttle 1회 추가 → stale window 정량화 후 결정.
4. **C3 silent except 로깅**: `_on_percept` 의 `try/except`에 `logwarn_throttle(5.0, ...)` 추가.

**우선순위 3 — 코드 품질:**
5. **C5/C6 useEffect deps**: MapLayers/CameraController 의도 명확화.
6. **frontend F1/F2/F18**: 죽은 주석/export/주석-실제값 불일치 정리.
7. **variant V3/V4**: Google Fonts 누락/weight 불일치 → 의도 확인 후 통일.

**검토 외 작업 후보:**
- `gps.lon_std/lat_std` 단위 `.msg` 주석 추가 (B12)
- ThreeJSScreen 상태 셋업 중복 추출 (`useThreeJSPanelState` hook 후보, F20)

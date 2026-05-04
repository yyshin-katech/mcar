# Three.js HMI variant — 구현 계획

> 결재 후 본 문서는 `~/mcar/claude_work_list/threejs_hmi_plan.md`에도 사본 저장 후 S1부터 실행.

## Context

`web_hmi` 패키지에 **새로운 `threejs` variant**를 추가하여 K-City HD맵 + 주변 객체 + 포인트클라우드를 Three.js로 시각화한다. 동기:

- 기존 F1 variant는 SVG 기반이라 포인트클라우드 등 대용량 3D 데이터 표현이 어려움
- K_CITY_2025 폴더의 13개 레이어 HD맵 전체를 활용하고 싶음 (현재는 `A2_LINK` 1개만 사용)
- `/track_Multi_RS` 객체에 연관된 점군을 트랙 단위로 시각화하여 인지 결과 디버깅 강화
- **F1 variant는 안정 작동 중이므로 절대 손대지 않음** (외과적 변경 원칙)

사용자 결정 사항(이미 확정):
1. F1과 별도의 새 variant (`threejs`) 추가
2. 객체 타입 분류: `valid_level` 우선 파싱, 미일치 시 `status==1→pedestrian` 폴백
3. 포인트클라우드 다운샘플 없이 트랙당 모든 점 송신 (대역폭 부하 감수)

## High-level architecture

```
K_CITY_2025/*.shp (UTM52N)
        │ pyproj 32652→5179, geopandas
        ▼
web_hmi_threejs_bridge.py (신규 노드)
   ├ A1~C6 13개 레이어 → /hmi/threejs/map (latched, 1회)
   └ /track_Multi_RS + /percept_topic ─ ApproxTimeSync ─→ /hmi/threejs/tracks (10Hz)
        │ rosbridge ws://9090
        ▼
index_threejs.html (신규)
   ├ ThreeScene.jsx (camera/light/animate)
   ├ MapLayers.jsx (LineSegments / ShapeGeometry / InstancedMesh per kind)
   ├ TrackBoxes.jsx (타입별 Box/Cylinder + 색상)
   ├ TrackPointClouds.jsx (THREE.Points, 사전할당 BufferAttribute)
   └ ControlPanel.jsx (체크박스: bbox / cloud / 레이어별 on-off)
```

`/hmi/state`(ego pose)는 기존 `web_hmi_bridge.py`가 발행하는 것을 재사용 → threejs variant 실행 시 **두 브리지 노드 동시 기동** (launch에서 자동).

## 신규/수정 파일 (모두 `/home/sim/mcar/src/visualization/web_hmi/` 하위)

### 신규
| 경로 | 역할 |
|---|---|
| `scripts/web_hmi_threejs_bridge.py` | shp 13개 로드 + EPSG 변환 + percept/track 매칭 + 발행 |
| `web/index_threejs.html` | 진입점 (`index_f1.html` 미러링, three.js 스크립트 추가) |
| `web/threejs/ThreeScene.jsx` | scene/camera/renderer/animate loop |
| `web/threejs/MapLayers.jsx` | 레이어별 Object3D 빌드 |
| `web/threejs/TrackBoxes.jsx` | 타입별 박스/실린더 |
| `web/threejs/TrackPointClouds.jsx` | THREE.Points + 사전할당 버퍼 |
| `web/threejs/ControlPanel.jsx` | 체크박스 패널 |
| `web/threejs/ThreeJSScreen.jsx` | 최상위 React 컴포넌트 |
| `web/threejs/types.js` | 타입 분류 함수 + 팔레트 |
| `web/vendor/three.min.js` | Three.js r160 UMD 빌드 (전역 `THREE`) |

### 수정
| 경로 | 변경 |
|---|---|
| `launch/web_hmi.launch` | `variant` 인자에 `threejs` 추가, `~mapdir` 파라미터 신설, threejs 브리지 노드 조건부 추가 |
| `CMakeLists.txt` | `catkin_install_python`에 신규 스크립트 추가 |

### 절대 손대지 않음
- `web/f1/*`, `web/hmi/*`, `web/index_f1.html`, `web/index.html`, `scripts/web_hmi_bridge.py`

## 재사용할 기존 함수/패턴

- **EPSG 변환**: `src/localization/gps_system_localizer/src/gps2tf.py`의 `pyproj.Transformer.from_crs()` 패턴
- **GeoPandas 로딩**: `src/localization/gps_system_localizer/src/local_test_node.py:66`의 `gpd.read_file().to_crs()`
- **Latched JSON 발행**: `web_hmi_bridge.py:_publish_map_once()` (lines 237-264) 그대로 모방
- **Approx time sync**: `message_filters.ApproximateTimeSynchronizer` (rospy 표준)
- **rviz 객체 분류 prior art**: `src/visualization/rviz_filter/src/main.cpp` (CUBE marker 매핑)
- **타입 폴백 규칙**: `src/visualization/pyqt_hmi/scripts/utils/hmi_state.py:277` (`status==1 → pedestrian`)

## JSON 메시지 계약

### `/hmi/threejs/map` (std_msgs/String, latched, 1회)

```json
{
  "epsg": 5179,
  "origin": [east0, north0],
  "layers": {
    "A2_LINK":   {"kind": "polyline", "data": [[[de,dn],...], ...]},
    "B2_SURFACELINEMARK": {"kind": "polyline", "data": [...]},
    "A3_DRIVEWAYSECTION": {"kind": "polygon",  "data": [...]},
    "C1_TRAFFICLIGHT":    {"kind": "point",    "data": [[de,dn], ...]},
    ...
  }
}
```

좌표는 **origin 기준 cm 단위 델타 + Float32 안전 범위**. 13개 레이어 통합 페이로드 5MB 초과 시 per-layer 분리 발행으로 폴백 (S3에서 측정 후 결정).

### `/hmi/threejs/tracks` (std_msgs/String, 10Hz)

```json
{
  "stamp": 1714867200.123,
  "tracks": [
    {"id": 42, "type": "car", "x": 12.4, "y": -3.1,
     "vx": 8.2, "vy": 0.1, "size_x": 4.4, "size_y": 1.85,
     "orientation": 0.05, "confidence": 0.91,
     "points": [x0,y0,z0, x1,y1,z1, ...]}
  ]
}
```

`points`는 **flat Float32 배열**. `/percept_topic`에서 매칭되지 않은 트랙은 `points` 키 생략.

## 객체 타입 분류 (`web/threejs/types.js`)

```js
const KNOWN = new Set(['Car','Truck','Motorcycle','Pedestrian','Other','Unknown','Invalid']);
function deriveType(o) {
  const head = (o.valid_level || '').split(',')[0].trim();
  if (KNOWN.has(head)) return head === 'Invalid' ? 'unknown' : head.toLowerCase();
  return o.status === 1 ? 'pedestrian' : 'car';  // 폴백
}
```

| 타입 | 모양 | 색상 |
|---|---|---|
| car | Box(size_x, 1.5, size_y) wireframe | `#ff5ea8` magenta |
| truck | Box(size_x, 2.5, size_y) | `#7c3aed` violet |
| motorcycle | Box(size_x, 1.0, size_y) | `#22e09a` green |
| pedestrian | Cylinder(0.3, 0.3, 1.7) | `#ffb547` amber |
| other/unknown | Box(size_x, 1.0, size_y) | `#7a8492` gray |

## Three.js scene tree

```
Scene
├ HemisphereLight + DirectionalLight
├ GridHelper (40m, 1m cells, ego 기준)
├ MapLayersGroup (모든 레이어, 절대 EPSG:5179−origin 좌표)
├ EgoMesh (BoxGeometry 4.635×1.89×1.5, cyan wireframe + 헤딩 화살표)
├ TracksGroup
│   ├ TrackBox#id (per-type 모양)
│   └ TrackPoints#id (THREE.Points, 사전할당 4096pt × 3float, 부족 시만 grow)
└ PerspectiveCamera (ego 추종, 60° tilt, 60m 후방)
```

**좌표 전략**: 맵은 `EPSG:5179 − origin`으로 한 번만 빌드, ego 위치만 매 프레임 갱신. Three.js GPU 변환이 매 프레임 호출되므로 SVG처럼 폴리라인 재계산 불필요.

**축 매핑**: world +X = east, +Z = north, +Y = up (height). yaw는 +Y축 회전.

## UI Control Panel (오른쪽 260px)

```
DISPLAY               MAP LAYERS              CAMERA
[×] Bounding boxes    [×] A1 Nodes            [Top]
[×] Track IDs         [×] A2 Lane lines       [Iso]
[×] Heading arrows    [×] A3 Driveway         [Free]
[×] Point clouds      [ ] A4 Subsidiary       
Point size [─●─]      [ ] A5 Parking          
                      [×] B1 Safety signs     
                      [×] B2 Surface lines    
                      [ ] B3 Surface marks    
                      [×] C1 Traffic light    
                      [ ] C3 Protection       
                      [ ] C4 Speed bumps      
                      [ ] C5 Height barrier   
                      [ ] C6 Post points      
```

체크박스 → 해당 Three.js 그룹의 `visible` 토글. 씬 재구성 없음.

## 단계별 실행 (목표 기반 검증)

| # | 작업 | 검증 |
|---|---|---|
| **S1** | `three.min.js`(r160 UMD) vendor 추가, `index_threejs.html` 골격 + 빈 cyan 큐브 렌더 | 브라우저에서 큐브 보임 |
| **S2** | `web_hmi_threejs_bridge.py` 골격: A2_LINK만 UTM52N→5179 변환 후 `/hmi/threejs/map` 발행 | `rostopic echo -n1 /hmi/threejs/map` 헤더 확인 |
| **S3** | 13개 레이어 전체 로드 (geopandas), 폴리곤은 outer ring만, layers 사전 빌드 | echo로 13개 키 모두 존재, 페이로드 KB 로그 출력 |
| **S4** | `MapLayers.jsx` 구현: kind별 LineSegments / ShapeGeometry / InstancedMesh | 브라우저에서 K-City 맵 보임, 레이어 토글 동작 |
| **S5** | `/track_Multi_RS` 구독, bbox-only 발행 (10Hz, 타입 분류 적용) | `~/bag_data/20260423` bag play, `rostopic hz` ≈10Hz |
| **S6** | `TrackBoxes.jsx`: 타입별 박스/실린더 렌더, ego-frame 배치 | 브라우저 박스 수가 `rviz_filter` 마커 수와 일치 |
| **S7** | `/percept_topic` 구독 + ApproxTimeSync 추가, **`Object.coreinfo.trakcer_id`로 매칭** (⚠ 오타 "trakcer" 그대로), `cloud_indices`로 점 슬라이스 | echo 결과에 `points` 배열 존재, 길이 100~수천 |
| **S8** | `TrackPointClouds.jsx`: 트랙 ID별 사전할당 BufferAttribute, 부족 시만 재할당 | 브라우저 점군 표시, DevTools FPS ≥ 30 |
| **S9** | `ControlPanel.jsx`: 모든 체크박스 + visibility 와이어링 + ego mesh + grid | 모든 체크박스가 그룹 가시성 토글, 새로고침 시 기본값 복원 |
| **S10** | `launch/web_hmi.launch` `variant=threejs` 분기 추가 (`page=index_threejs.html`, 조건부 노드, `~mapdir`) | `roslaunch web_hmi web_hmi.launch variant:=threejs`로 풀 HMI 실행, `variant:=f1`도 정상 |
| **S11** | `CMakeLists.txt` 갱신, `catkin_make --pkg web_hmi` 빌드 확인 | 빌드 성공 + `rosrun` 정상 |

## 알려진 위험과 대응

1. **`Object.msg`에 최상위 `id` 없음** — 매칭은 `Object.coreinfo.trakcer_id` (오타 "trakcer" 그대로). S7 첫 메시지를 `rostopic echo`로 검증 후 락인.
2. **점군 좌표계 불일치 가능성** — `scan_pointcloud`는 lidar frame, 트랙 위치는 ego frame. 시각적 오프셋 발견 시 `/base_link ← lidar_link` TF 룩업 1회 후 회전/이동 적용 (S8에서 검증).
3. **레이어 통합 JSON 5MB 초과 시** rosbridge 멈춤 가능 — S3에서 측정, 초과 시 per-layer 분리 발행으로 한 줄 리팩터.
4. **`/percept_topic` Hz > `/track_Multi_RS` Hz**일 때 ApproxTimeSync 드롭 — `slop=0.1`, `queue_size=10` 권장. 매칭률 로깅으로 모니터.
5. **Three.js ESM vs UMD** — F1은 in-browser Babel + 전역 스크립트. **UMD 빌드 사용**으로 통일 (`window.THREE`).
6. **두 브리지 노드 동시 실행** — threejs variant도 ego pose는 `web_hmi_bridge`의 `/hmi/state` 재사용. launch에서 항상 함께 기동.

## 검증 종합 시나리오

1. `roslaunch web_hmi web_hmi.launch variant:=threejs`
2. 별도 터미널: `rosbag play --loop ~/bag_data/20260423/2026-04-23-10-52-44_*.bag`
3. 브라우저 자동 오픈 (`http://localhost:8088/index_threejs.html`)
4. 확인 항목:
   - K-City HD맵 13레이어 모두 표출
   - 체크박스로 레이어/박스/점군 개별 on-off
   - 트랙 박스가 타입별 다른 색/모양 (Car=magenta, Pedestrian=amber cylinder 등)
   - 점군 토글 시 트랙별 점 표시/숨김
   - Ego 차량(IONIQ 5 박스) 중앙에 정지, 맵이 ego 따라 이동
   - DevTools FPS ≥ 30, rosbridge ws 트래픽 5MB/s 이하
5. `variant:=f1`로 다시 기동하여 **F1 SVG HMI 무손상 작동** 회귀 확인

## Critical files (구현 시 반드시 참조/수정)

- `/home/sim/mcar/src/visualization/web_hmi/scripts/web_hmi_threejs_bridge.py` (신규)
- `/home/sim/mcar/src/visualization/web_hmi/web/index_threejs.html` (신규)
- `/home/sim/mcar/src/visualization/web_hmi/web/threejs/ThreeScene.jsx` (신규)
- `/home/sim/mcar/src/visualization/web_hmi/web/threejs/TrackPointClouds.jsx` (신규, 성능 핵심)
- `/home/sim/mcar/src/visualization/web_hmi/launch/web_hmi.launch` (수정)
- `/home/sim/mcar/src/visualization/web_hmi/scripts/web_hmi_bridge.py:237-264` (모방 대상, 수정 X)
- `/home/sim/mcar/src/msgs/perception_ros_msg/msg/Object.msg` + `CoreInfo.msg` + `SupplementInfo.msg` (필드 참조)
- `/home/sim/mcar/src/localization/gps_system_localizer/mapfiles/K_CITY_2025/*.shp` (입력 데이터)

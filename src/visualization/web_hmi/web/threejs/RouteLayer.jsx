/* global React, THREE, window, useThree, useJsonTopic, useRosState, useEgoPose */

// 주행 예정 경로 리본 (global_nav_hmi).
//
// /hmi/threejs/route (latched, extract_route.py 산출) 의 중앙 대표선을 지면
// 삼각 스트립(폭 있는 리본)으로 그린다. /hmi/threejs/map 의 origin(EPSG:5179)
// 기준 delta 로 렌더 — MapLayers/BlockZones/CrosswalkZones/EgoMesh 와 동일 프레임
// (+X=east_delta, +Z=north_delta, scene scale.z=-1).
//
// 전환: /hmi/state 의 on_block_link===1 && do_not_go_forward===1 이 처음
// 성립하면 old→new 로 스위치하고 latch(이후 트리거 해제돼도 유지). 신규
// 토픽/콜백 없이 기존 /hmi/state(useRosState) 만 구독.
//
// 진행 페이드: /hmi/ego_pose(useEgoPose, ~50 Hz) 최근접 정점 index 까지 잘라내
// 자차 앞 구간만 표시(지나간 구간 미표시).

const ROUTE_Y = 0.16;          // BlockZones(0.1)/CrosswalkZones(0.12) 위 — z-fighting 회피
const ROUTE_HALF_W = 1.6;      // 리본 반폭(m) → 총 폭 ≈3.2 m (한 차선 폭)
const COLOR_OLD = 0x00e5ff;    // 시안 (기존 경로)
const COLOR_NEW = 0x39ff88;    // 그린 (신규 경로, 전환 강조)
const ROUTE_OPACITY = 0.55;
const PROGRESS_STEP = 1;       // nearest index 가 이만큼 전진해야 리본 재생성(히스테리시스)

// 지면 삼각 스트립 리본 빌더 — BlockZones.buildQuad 재질 레시피 계승.
function buildRibbon(points, origin, color) {
  // points: [[e,n],...] EPSG:5179 절대. 최소 2점.
  const n = points.length;
  if (n < 2) return null;
  // delta 좌표(+X=east_delta, +Z=north_delta)로 변환
  const cx = new Float32Array(n);
  const cz = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    cx[i] = points[i][0] - origin[0];
    cz[i] = points[i][1] - origin[1];
  }
  const pos = new Float32Array(n * 2 * 3);      // 정점 2개/센터점 (left,right)
  for (let i = 0; i < n; i++) {
    // 탄젠트(이웃 차분) → XZ 평면 좌법선(-tz, tx)
    const iP = Math.max(0, i - 1);
    const iN = Math.min(n - 1, i + 1);
    let tx = cx[iN] - cx[iP];
    let tz = cz[iN] - cz[iP];
    const L = Math.hypot(tx, tz) || 1;
    tx /= L; tz /= L;
    const nx = -tz;                             // 좌법선(±방향 무관, 리본 대칭)
    const nz = tx;
    const o = i * 6;
    pos[o + 0] = cx[i] + nx * ROUTE_HALF_W; pos[o + 1] = ROUTE_Y; pos[o + 2] = cz[i] + nz * ROUTE_HALF_W; // left
    pos[o + 3] = cx[i] - nx * ROUTE_HALF_W; pos[o + 4] = ROUTE_Y; pos[o + 5] = cz[i] - nz * ROUTE_HALF_W; // right
  }
  const idx = [];
  for (let i = 0; i < n - 1; i++) {
    const a = i * 2;
    const b = i * 2 + 1;
    const c = (i + 1) * 2;
    const d = (i + 1) * 2 + 1;
    idx.push(a, b, c, b, d, c);                 // 두 삼각형/세그먼트
  }
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geom.setIndex(idx);
  geom.computeVertexNormals();
  const mat = new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity: ROUTE_OPACITY,
    side: THREE.DoubleSide,
    depthWrite: false,
  });
  return new THREE.Mesh(geom, mat);
}

// 진행 페이드용 최근접 정점 index. from 이후만 탐색 → 단조 전진(이미 지난 점 재선택 방지).
function nearestIndex(points, e, n, from) {
  let bi = from;
  let bd = Infinity;
  for (let i = from; i < points.length; i++) {
    const dx = points[i][0] - e;
    const dz = points[i][1] - n;
    const d = dx * dx + dz * dz;
    if (d < bd) { bd = d; bi = i; }
  }
  return bi;
}

function disposeChildren(group) {
  while (group.children.length) {
    const c = group.children[0];
    group.remove(c);
    if (c.geometry) c.geometry.dispose();
    if (c.material) {
      const ms = Array.isArray(c.material) ? c.material : [c.material];
      ms.forEach((m) => m && m.dispose && m.dispose());
    }
  }
}

function RouteLayer() {
  const three = useThree();
  const map = useJsonTopic('/hmi/threejs/map', null);   // origin
  const route = useJsonTopic('/hmi/threejs/route', null);
  const state = useRosState();
  const ego = useEgoPose();

  const groupRef = React.useRef(null);
  const idxRef = React.useRef(0);        // 현재 nearest index (단조 증가)
  const builtFromRef = React.useRef(-1); // 마지막으로 리본 만든 시작 index
  const [latched, setLatched] = React.useState(false);

  // latch: on_block_link && do_not_go_forward 한 번 성립하면 영구 new (기존 트리거 재사용).
  React.useEffect(() => {
    if (!latched && state && state.on_block_link === 1 && state.do_not_go_forward === 1) {
      setLatched(true);
    }
  });
  const activeKey = latched ? 'new' : 'old';

  // 활성 경로/맵/전환 변경 시 상태 리셋 + 그룹 준비.
  React.useEffect(() => {
    if (!three || !map || !map.origin || !route || !route.routes) return undefined;
    const group = new THREE.Group();
    group.name = 'route_layer';
    three.mapGroup.add(group);
    groupRef.current = group;
    idxRef.current = 0;
    builtFromRef.current = -1;   // 전환 시 진행도 리셋(새 경로 처음부터)
    return () => {
      three.mapGroup.remove(group);
      disposeChildren(group);
      groupRef.current = null;
    };
  }, [three, map, route, activeKey]);

  // ego 이동마다: nearest index 갱신 → 전진 시 [idx..end] 리본 재생성(지나간 구간 미표시).
  React.useEffect(() => {
    const group = groupRef.current;
    if (!group || !map || !map.origin || !route || !route.routes) return;
    const active = route.routes[activeKey];
    const pts = active && active.points;
    if (!pts || pts.length < 2) return;
    const e = (ego && ego.east) || 0;
    const n = (ego && ego.north) || 0;
    const ni = nearestIndex(pts, e, n, idxRef.current);
    idxRef.current = ni;
    if (builtFromRef.current >= 0 && ni - builtFromRef.current < PROGRESS_STEP) return; // 히스테리시스
    disposeChildren(group);
    const ahead = pts.slice(Math.max(0, ni));   // 지나간 구간 잘라내기
    const mesh = buildRibbon(ahead, map.origin, activeKey === 'new' ? COLOR_NEW : COLOR_OLD);
    if (mesh) group.add(mesh);
    builtFromRef.current = ni;
  });   // deps 없음: ego 재렌더마다 실행(내부 히스테리시스로 재생성 억제)

  return null;
}

window.RouteLayer = RouteLayer;

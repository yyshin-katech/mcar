/* global React, THREE, window, useThree, useJsonTopic, useRosState */

// "전방 보행자 주의" — 횡단보도 보행자 퓨전(자체 라이다 + OBU V2X) 지면 표시.
//
// #1/#2 횡단보도 다각형을 지면(y≈0.12) 위에 상시 표시하고, 퓨전 노드가
// active(=LINK_ID 기반)로 지목하고 pedestrian_present 인 횡단보도만 붉은색으로
// 점멸시킨다. 좌표는 EPSG:5179 절대좌표를 map `origin` 으로 시프트 (MapLayers /
// BlockZones 와 동일 규약: +X=east_delta, +Z=north_delta, scene scale.z=-1).
// state 소스: /hmi/state 의 crosswalk_ped_active / crosswalk_ped_present /
// crosswalk_ped_source (web_hmi_bridge 가 additive 로 발행).
//
// ThreeScene 은 per-frame 훅(useFrame)을 노출하지 않으므로 점멸은
// setInterval + material.opacity 토글로 구현한다.

// SINGLE SOURCE: 좌표는 src/sensing/can/src/katech_ped_detector.py:106-107 의
// crosswalk_data 와 반드시 동일하게 유지할 것 (검출 판정과 표시가 어긋나면 안 됨).
const CROSSWALK_POLYS = {
  1: [
    [930819.312725, 1929593.158143], [930807.530293, 1929580.608639], [930804.537889, 1929582.883314],
    [930803.754045, 1929582.693963], [930800.736230, 1929585.001303], [930798.147151, 1929585.252169],
    [930814.166390, 1929602.326491], [930814.430028, 1929599.606956], [930813.356668, 1929598.462133],
    [930816.299566, 1929596.210116], [930816.286256, 1929595.495250],
  ],
  2: [
    [930819.933061, 1929617.498679], [930817.484334, 1929614.491229], [930817.695865, 1929613.842349],
    [930815.283504, 1929610.895521], [930813.983369, 1929609.298809], [930788.698959, 1929628.204441],
    [930789.932348, 1929629.623349], [930790.729092, 1929629.069490], [930793.150823, 1929631.994565],
    [930792.937083, 1929632.723870], [930795.460264, 1929635.651967],
  ],
};

const CROSSWALK_Y = 0.12;          // BlockZones(0.1) 위에 살짝 띄워 z-fighting 회피
const BASE_COLOR = 0x5cf2ff;       // 상시 표시(중립) — 옅은 시안
const BASE_OPACITY = 0.15;
const ALERT_COLOR = 0xff3030;      // 점멸 붉은색 (요구: 영역은 붉은 점멸)
const BLINK_HI = 0.55, BLINK_LO = 0.12, BLINK_MS = 400;

// Build a filled polygon (centroid triangle-fan) from N absolute [E, N] corners,
// shifted by origin (+X=east_delta, +Z=north_delta). DoubleSide +
// depthWrite:false → flat, semi-transparent fill (same material recipe as
// BlockZones.buildQuad). Correct for convex / star-convex outlines.
function buildPolygon(corners, origin) {
  if (!corners || corners.length < 3) return null;
  const n = corners.length;
  let cx = 0, cy = 0;
  for (let i = 0; i < n; i++) { cx += corners[i][0]; cy += corners[i][1]; }
  cx /= n; cy /= n;

  const positions = new Float32Array((n + 1) * 3);
  // vertex 0 = centroid
  positions[0] = cx - origin[0];   // +X = east_delta
  positions[1] = CROSSWALK_Y;      // ground
  positions[2] = cy - origin[1];   // +Z = north_delta
  // vertices 1..n = polygon corners
  for (let i = 0; i < n; i++) {
    positions[(i + 1) * 3 + 0] = corners[i][0] - origin[0];
    positions[(i + 1) * 3 + 1] = CROSSWALK_Y;
    positions[(i + 1) * 3 + 2] = corners[i][1] - origin[1];
  }
  // triangle-fan around the centroid: (0, i+1, i+2 wrap)
  const indices = [];
  for (let i = 0; i < n; i++) {
    indices.push(0, i + 1, ((i + 1) % n) + 1);
  }
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geom.setIndex(indices);
  geom.computeVertexNormals();
  const mat = new THREE.MeshBasicMaterial({
    color: BASE_COLOR,
    transparent: true,
    opacity: BASE_OPACITY,
    side: THREE.DoubleSide,
    depthWrite: false,
  });
  return new THREE.Mesh(geom, mat);
}

function disposeGroup(group) {
  group.traverse((obj) => {
    if (obj.geometry) obj.geometry.dispose();
    if (obj.material) {
      const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
      mats.forEach((m) => m && m.dispose && m.dispose());
    }
  });
}

function CrosswalkZones() {
  const three = useThree();
  const map = useJsonTopic('/hmi/threejs/map', null);
  const state = useRosState();
  const groupRef = React.useRef(null);
  const meshRef = React.useRef({});
  const [blinkOn, setBlinkOn] = React.useState(false);

  // (Re)build both crosswalk polygons whenever the map origin changes.
  React.useEffect(() => {
    if (!three || !map || !map.origin) return undefined;
    const origin = map.origin;

    if (groupRef.current) {
      three.mapGroup.remove(groupRef.current);
      disposeGroup(groupRef.current);
      groupRef.current = null;
      meshRef.current = {};
    }

    const group = new THREE.Group();
    group.name = 'crosswalk_zones';
    const meshes = {};
    [1, 2].forEach((id) => {
      const mesh = buildPolygon(CROSSWALK_POLYS[id], origin);
      if (mesh) { group.add(mesh); meshes[id] = mesh; }
    });
    three.mapGroup.add(group);
    groupRef.current = group;
    meshRef.current = meshes;

    return () => {
      if (groupRef.current) {
        three.mapGroup.remove(groupRef.current);
        disposeGroup(groupRef.current);
        groupRef.current = null;
        meshRef.current = {};
      }
    };
  }, [three, map]);

  // Blink toggle — ThreeScene exposes no per-frame hook (useFrame), so drive the
  // alert opacity from a setInterval instead.
  React.useEffect(() => {
    const id = setInterval(() => setBlinkOn((v) => !v), BLINK_MS);
    return () => clearInterval(id);
  }, []);

  // Every render: both polygons stay visible; only the active+present one blinks
  // red, the rest sit at the neutral base color/opacity.
  React.useEffect(() => {
    const meshes = meshRef.current;
    [1, 2].forEach((id) => {
      const mesh = meshes[id];
      if (!mesh) return;
      const alert = state && state.crosswalk_ped_active === id && state.crosswalk_ped_present === true;
      if (alert) {
        mesh.material.color.setHex(ALERT_COLOR);
        mesh.material.opacity = blinkOn ? BLINK_HI : BLINK_LO;
      } else {
        mesh.material.color.setHex(BASE_COLOR);
        mesh.material.opacity = BASE_OPACITY;
      }
      mesh.visible = true;
    });
  });

  return null;
}
window.CrosswalkZones = CrosswalkZones;

// Map-canvas overlay: "전방 보행자 주의" banner. Shown while
// crosswalk_ped_present; text/border color encodes the fusion source
// (1=own/lidar=주황 #ff9800, 2=obu/v2x=빨강 #ff3030, 3=both=자홍 #ff30ff). Placed
// below BlockBanner (top:64) so the two never overlap. Subscribes /hmi/state
// independently to stay in sync with CrosswalkZones without threading props
// through the shell.
function CrosswalkPedBanner() {
  const state = useRosState();
  const present = state && state.crosswalk_ped_present === true;
  if (!present) return null;
  const src = (state && state.crosswalk_ped_source) || 0;
  const color = ({ 1: '#ff9800', 2: '#ff3030', 3: '#ff30ff' })[src] || '#ff3030';
  return (
    <div style={{
      position: "absolute", top: 112, left: "50%", transform: "translateX(-50%)",
      zIndex: 30, display: "flex", alignItems: "center",
      padding: "10px 24px", borderRadius: 8,
      background: "rgba(10,12,18,0.72)",
      border: `1px solid ${color}`,
      boxShadow: "0 6px 24px rgba(0,0,0,0.5)",
      fontWeight: 700, letterSpacing: "0.04em",
      fontFamily: "Inter, Pretendard, system-ui, sans-serif",
      pointerEvents: "none", userSelect: "none",
    }}>
      <span style={{ fontSize: 22, color, textShadow: "0 1px 4px rgba(0,0,0,0.6)" }}>
        ⚠ 전방 보행자 주의
      </span>
    </div>
  );
}
window.CrosswalkPedBanner = CrosswalkPedBanner;

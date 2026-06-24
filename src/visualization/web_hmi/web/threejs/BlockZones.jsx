/* global React, THREE, window, useThree, useJsonTopic, useRosState */

// "전방 직진 주행 금지" ground boxes.
//
// The md anchors are points lying ALONG the link centerline (nearly collinear),
// not box corners — so we treat each cluster as a road-axis polyline and extrude
// it perpendicular by a lane half-width to build a filled, semi-transparent red
// rectangle that overlaps the displayed link on the map. Drawn flat on the ground
// (y≈0.1) in EPSG:5179 absolute coords, shifted by the map `origin` exactly like
// MapLayers (world axes: +X=east_delta, +Z=north_delta, scene scale.z=-1, so a
// north-delta at z renders toward real north). Visible only while
// state.on_block_link === 1.

// Box A (link 417 시작부 일대) — EPSG:5179 absolute [east, north] along the link.
const BLOCK_A = [
  [931706.402, 1928787.276],
  [931709.078, 1928785.213],
  [931703.929, 1928789.172],
];
// Box B (417 corridor 일대).
const BLOCK_B = [
  [931741.676, 1928832.478],
  [931739.247, 1928834.373],
  [931744.369, 1928830.448],
];

const BLOCK_Y = 0.1;        // sit just above the ground plane
const BLOCK_COLOR = 0xff3030;
const BLOCK_OPACITY = 0.3;
const BLOCK_HALF_W = 2.0;   // half lane width [m] → 4 m wide box over the link
const BLOCK_EXT = 1.0;      // extend each end along the link [m] for full coverage

// Build a filled rectangle covering the link: take the two farthest anchor points
// as the road axis, extend the ends, then offset perpendicular by the half-width.
// Returns a 4-corner quad (2 triangles) in absolute [E, N], shifted by origin.
function buildBlockBox(points, origin) {
  if (!points || points.length < 2) return null;
  // farthest pair = road-axis endpoints
  let a = 0;
  let b = 1;
  let best = -1;
  for (let i = 0; i < points.length; i++) {
    for (let j = i + 1; j < points.length; j++) {
      const d = Math.hypot(points[i][0] - points[j][0], points[i][1] - points[j][1]);
      if (d > best) { best = d; a = i; b = j; }
    }
  }
  const p0 = points[a];
  const p1 = points[b];
  const dx = p1[0] - p0[0];
  const dy = p1[1] - p0[1];
  const len = Math.hypot(dx, dy) || 1;
  const ux = dx / len;
  const uy = dy / len;             // unit along link
  const px = -uy * BLOCK_HALF_W;
  const py = ux * BLOCK_HALF_W;    // perpendicular offset
  // endpoints extended along the link
  const s0x = p0[0] - ux * BLOCK_EXT;
  const s0y = p0[1] - uy * BLOCK_EXT;
  const s1x = p1[0] + ux * BLOCK_EXT;
  const s1y = p1[1] + uy * BLOCK_EXT;
  // 4 corners (E, N), shifted by origin → (+X=east_delta, +Z=north_delta)
  const corners = [
    [s0x + px, s0y + py],
    [s1x + px, s1y + py],
    [s1x - px, s1y - py],
    [s0x - px, s0y - py],
  ];
  const positions = new Float32Array(4 * 3);
  for (let i = 0; i < 4; i++) {
    positions[i * 3 + 0] = corners[i][0] - origin[0];
    positions[i * 3 + 1] = BLOCK_Y;
    positions[i * 3 + 2] = corners[i][1] - origin[1];
  }
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geom.setIndex([0, 1, 2, 0, 2, 3]);
  geom.computeVertexNormals();
  const mat = new THREE.MeshBasicMaterial({
    color: BLOCK_COLOR,
    transparent: true,
    opacity: BLOCK_OPACITY,
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

function BlockZones() {
  const three = useThree();
  const map = useJsonTopic('/hmi/threejs/map', null);
  const state = useRosState();
  const groupRef = React.useRef(null);

  // (Re)build the two boxes whenever the map origin changes.
  React.useEffect(() => {
    if (!three || !map || !map.origin) return undefined;
    const origin = map.origin;

    if (groupRef.current) {
      three.mapGroup.remove(groupRef.current);
      disposeGroup(groupRef.current);
      groupRef.current = null;
    }

    const group = new THREE.Group();
    group.name = 'block_zones';
    [BLOCK_A, BLOCK_B].forEach((pts) => {
      const mesh = buildBlockBox(pts, origin);
      if (mesh) group.add(mesh);
    });
    group.visible = false;  // hidden until on_block_link === 1
    three.mapGroup.add(group);
    groupRef.current = group;

    return () => {
      if (groupRef.current) {
        three.mapGroup.remove(groupRef.current);
        disposeGroup(groupRef.current);
        groupRef.current = null;
      }
    };
  }, [three, map]);

  // Toggle visibility from /hmi/state on every render.
  React.useEffect(() => {
    if (groupRef.current) {
      groupRef.current.visible = (state && state.on_block_link === 1);
    }
  });

  return null;
}

window.BlockZones = BlockZones;
window.__buildBlockBox = buildBlockBox;

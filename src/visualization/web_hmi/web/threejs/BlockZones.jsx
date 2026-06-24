/* global React, THREE, window, useThree, useJsonTopic, useRosState */

// "전방 직진 주행 금지" ground ribbon.
//
// The red blocking strip is a single oriented rectangle built from the
// hmi_block.md region anchors (6 WGS84 points, region7=region6 dup dropped).
// The anchors are projected onto their principal road axis and wrapped by an
// oriented bounding box (OBB) — a clean, non-self-intersecting quad spanning the
// whole blocking corridor (Box A 시작부 ~ Box B 끝부, ≈59 m × 8.5 m). Drawn flat
// on the ground (y≈0.1) in EPSG:5179 absolute coords, shifted by the map `origin`
// exactly like MapLayers (world axes: +X=east_delta, +Z=north_delta,
// scene scale.z=-1, so a north-delta at z renders toward real north).
// Visible only while state.on_block_link === 1.

// md anchor 6점 → 도로축 OBB 4코너 (EPSG:5179 [east, north]).
// Regenerate via _hmi_block_workspace (PCA principal axis, ±1 m len/width pad).
const BLOCK_ANCHOR_QUAD = [
  [931709.221, 1928783.771],
  [931745.789, 1928830.608],
  [931739.093, 1928835.836],
  [931702.525, 1928788.999],
];

const BLOCK_Y = 0.1;        // sit just above the ground plane
const BLOCK_COLOR = 0xff3030;
const BLOCK_OPACITY = 0.3;

// Build one filled quad (2 triangles) from 4 absolute [E, N] corners, shifted by
// origin (+X=east_delta, +Z=north_delta). DoubleSide + depthWrite:false → flat,
// semi-transparent rectangle over the blocking corridor.
function buildQuad(corners, origin) {
  if (!corners || corners.length !== 4) return null;
  const positions = new Float32Array(4 * 3);
  for (let i = 0; i < 4; i++) {
    positions[i * 3 + 0] = corners[i][0] - origin[0];   // +X = east_delta
    positions[i * 3 + 1] = BLOCK_Y;                      // 0.1, ground
    positions[i * 3 + 2] = corners[i][1] - origin[1];    // +Z = north_delta
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

  // (Re)build the blocking quad whenever the map origin changes.
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
    const mesh = buildQuad(BLOCK_ANCHOR_QUAD, origin);
    if (mesh) group.add(mesh);
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
      groupRef.current.visible = (state && state.on_block_link === 1 && state.do_not_go_forward === 1);
    }
  });

  return null;
}

window.BlockZones = BlockZones;
window.__buildQuad = buildQuad;

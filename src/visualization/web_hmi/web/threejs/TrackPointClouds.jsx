/* global React, THREE, window, useThree, useJsonTopic, OBJ_PALETTE */

// Per-track THREE.Points slot. Buffers are pre-allocated and reused;
// capacity grows on overflow (next power of 2). Each slot lives inside
// a 'pointclouds' subgroup of trackGroup so a single visibility flag
// on the parent toggles all clouds.

const POINTS_INITIAL_CAP = 4096;
const FALLBACK_COLOR = '#cfd8e3';

function nextPow2(n) {
  let v = 1;
  while (v < n) v *= 2;
  return v;
}

function makePointsSlot(numPoints, color, pointSize) {
  const cap = Math.max(POINTS_INITIAL_CAP, nextPow2(numPoints));
  const positions = new Float32Array(cap * 3);
  const geom = new THREE.BufferGeometry();
  const attr = new THREE.BufferAttribute(positions, 3);
  if (THREE.DynamicDrawUsage !== undefined) attr.setUsage(THREE.DynamicDrawUsage);
  geom.setAttribute('position', attr);
  geom.setDrawRange(0, 0);
  const mat = new THREE.PointsMaterial({
    color, size: pointSize || 0.08, sizeAttenuation: true,
  });
  const points = new THREE.Points(geom, mat);
  // Per-track sphere with dynamic drawRange would mis-cull; cheap clouds anyway.
  points.frustumCulled = false;
  return { points, capacity: cap };
}

function disposeSlot(parent, slot) {
  parent.remove(slot.points);
  if (slot.points.geometry) slot.points.geometry.dispose();
  if (slot.points.material) slot.points.material.dispose();
}

function TrackPointClouds({ showClouds, pointSize }) {
  const three = useThree();
  const tracks = useJsonTopic('/hmi/threejs/tracks', null);
  const slotsRef = React.useRef(new Map());   // track id → slot
  const parentRef = React.useRef(null);

  // Mount the cloud parent group inside trackGroup; cleanup on unmount.
  React.useEffect(() => {
    if (!three) return undefined;
    const parent = new THREE.Group();
    parent.name = 'pointclouds';
    three.trackGroup.add(parent);
    parentRef.current = parent;
    return () => {
      three.trackGroup.remove(parent);
      for (const slot of slotsRef.current.values()) {
        if (slot.points.geometry) slot.points.geometry.dispose();
        if (slot.points.material) slot.points.material.dispose();
      }
      slotsRef.current.clear();
      parentRef.current = null;
    };
  }, [three]);

  // Apply visibility flag every render (cheap toggle).
  React.useEffect(() => {
    if (parentRef.current) parentRef.current.visible = !!showClouds;
  });

  // Update per-track point buffers.
  React.useEffect(() => {
    const parent = parentRef.current;
    if (!parent || !tracks || !tracks.tracks) return undefined;
    const palette = window.OBJ_PALETTE || {};
    const seen = new Set();

    tracks.tracks.forEach((trk) => {
      seen.add(trk.id);
      const arr = trk.points;
      const slotMap = slotsRef.current;
      let slot = slotMap.get(trk.id);

      // Track has no points this frame: drop existing slot, if any.
      if (!arr || !arr.length) {
        if (slot) {
          disposeSlot(parent, slot);
          slotMap.delete(trk.id);
        }
        return;
      }

      const numPoints = (arr.length / 3) | 0;
      const color = palette[trk.type] || FALLBACK_COLOR;

      // Build new slot or grow if capacity insufficient.
      if (!slot || slot.capacity < numPoints) {
        if (slot) disposeSlot(parent, slot);
        slot = makePointsSlot(numPoints, color, pointSize);
        slot.points.name = 'cloud#' + trk.id;
        parent.add(slot.points);
        slotMap.set(trk.id, slot);
      } else {
        // Sync color in case track type changed (CAN id reused for different obj).
        slot.points.material.color.set(color);
        if (pointSize !== undefined && slot.points.material.size !== pointSize) {
          slot.points.material.size = pointSize;
        }
      }

      // Lidar /base_link is REP-103: x=fwd, y=left, z=up. Three.js Y is up,
      // so swap the trailing two: lidar.z → Three.js Y (height),
      // lidar.y → Three.js Z (lateral, becomes -Z=real north after scene flip).
      const positions = slot.points.geometry.attributes.position.array;
      for (let i = 0; i < numPoints; i++) {
        const j = i * 3;
        positions[j    ] = arr[j    ]; // lidar x  → +X (forward)
        positions[j + 1] = arr[j + 2]; // lidar z  → +Y (up)
        positions[j + 2] = arr[j + 1]; // lidar y  → +Z (left, flipped to north)
      }
      slot.points.geometry.setDrawRange(0, numPoints);
      slot.points.geometry.attributes.position.needsUpdate = true;
    });

    // Drop slots whose track id disappeared.
    for (const [id, slot] of slotsRef.current) {
      if (!seen.has(id)) {
        disposeSlot(parent, slot);
        slotsRef.current.delete(id);
      }
    }
    return undefined;
  }, [tracks, pointSize]);

  return null;
}

window.TrackPointClouds = TrackPointClouds;
window.__makePointsSlot = makePointsSlot;
window.__nextPow2 = nextPow2;

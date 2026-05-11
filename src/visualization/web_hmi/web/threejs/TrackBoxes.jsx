/* global React, THREE, window, useThree, useJsonTopic,
          OBJ_PALETTE, OBJ_DIMS */

// Build a per-type bbox mesh (wireframe). Returns a THREE.Object3D.
// type: lowercase ('car','truck','motorcycle','pedestrian','other','unknown')
// size_x / size_y: meters (ego-frame length / width)
function buildBox(type, size_x, size_y) {
  const dims = OBJ_DIMS[type] || OBJ_DIMS.other;
  const color = OBJ_PALETTE[type] || OBJ_PALETTE.other;
  const mat = new THREE.MeshBasicMaterial({
    color, wireframe: true, transparent: true, opacity: 0.95,
  });
  let geom;
  if (type === 'pedestrian') {
    const r = (dims.radius || 0.3);
    geom = new THREE.CylinderGeometry(r, r, dims.h, 8);
  } else {
    const lx = Math.max(0.3, size_x || 1.0);
    const ly = Math.max(0.3, size_y || 1.0);
    geom = new THREE.BoxGeometry(lx, dims.h, ly);
  }
  const mesh = new THREE.Mesh(geom, mat);
  // Position so bbox sits on the ground plane (mesh origin is centroid).
  mesh.position.y = dims.h / 2;
  return mesh;
}

// Heading arrow (forward direction along +X in ego-frame).
function buildHeadingArrow(type, size_x) {
  const color = OBJ_PALETTE[type] || OBJ_PALETTE.other;
  const len = Math.max(0.5, (size_x || 1.5) * 0.6);
  const dir = new THREE.Vector3(1, 0, 0);
  const origin = new THREE.Vector3(0, 0, 0);
  return new THREE.ArrowHelper(dir, origin, len, color, 0.4, 0.25);
}

function disposeMesh(obj) {
  obj.traverse((c) => {
    if (c.geometry) c.geometry.dispose();
    if (c.material) {
      const ms = Array.isArray(c.material) ? c.material : [c.material];
      ms.forEach((m) => m && m.dispose && m.dispose());
    }
  });
}

function TrackBoxes({ showBoxes, showHeading, showIds }) {
  const three = useThree();
  const tracks = useJsonTopic('/hmi/threejs/tracks', null);
  const map = useJsonTopic('/hmi/threejs/map', null);
  const slotsRef = React.useRef(new Map()); // id → { group, type }

  // Tracks are emitted in ego-frame at perception rate (~10 Hz). The bridge
  // pairs each emission with the ego pose used to compute those local
  // coords (`ego_at_emit`). Transforming the trackGroup with that snapshot
  // — instead of the live 50 Hz /hmi/ego_pose — keeps tracks fixed in the
  // world between perception ticks. (Using live ego makes them slide ~1 m
  // every tick as ego drifts in the gap.)
  React.useEffect(() => {
    if (!three) return;
    const origin = (map && map.origin) || [0, 0];
    const e = (tracks && tracks.ego_at_emit) || null;
    const eEast  = (e && e.east)  || 0;
    const eNorth = (e && e.north) || 0;
    const eYaw   = (e && e.yaw)   || 0;
    three.trackGroup.position.set(eEast - origin[0], 0, eNorth - origin[1]);
    three.trackGroup.rotation.y = -eYaw;
  }, [three, tracks, map]);

  // Add/update/remove tracks keyed by id.
  React.useEffect(() => {
    if (!three || !tracks || !tracks.tracks) return undefined;
    const seen = new Set();
    // Two anti-flicker filters tuned against the live perception stream:
    //  - confirm threshold: a new ID becomes visible only after its 2nd
    //    sighting. Single-frame ghost detections (frequent, ~30% of new
    //    IDs in the bag) are silently dropped.
    //  - miss grace: keep the last pose for up to ~1 s of absence. The
    //    tracker drops 43% of IDs for 1–6 frames at random; grace covers
    //    those and lets a true disappearance clear within a second.
    const TRACK_CONFIRM = 2;
    // Lowered from 10 to 4: the bridge now caps tracks at the N nearest, so
    // anything trimmed (further away) used to linger ~5 s on screen at the
    // throttled rate. Shorter grace prunes those promptly.
    const TRACK_MISS_GRACE = 4;
    tracks.tracks.forEach((trk) => {
      seen.add(trk.id);
      let slot = slotsRef.current.get(trk.id);
      if (!slot || slot.type !== trk.type) {
        // Rebuild only on type change. Perception emits size_x/size_y at
        // full float precision, fluctuating ~0.1 m every tick (~10 Hz);
        // rebuilding the geometry on each fluctuation makes the box
        // visibly flicker. The first-emission size is good enough — small
        // ongoing variation is not worth the dispose/recreate cost.
        if (slot) {
          three.trackGroup.remove(slot.group);
          disposeMesh(slot.group);
        }
        const group = new THREE.Group();
        group.name = 'track#' + trk.id;
        group.add(buildBox(trk.type, trk.size_x, trk.size_y));
        const arrow = buildHeadingArrow(trk.type, trk.size_x);
        arrow.name = 'arrow';
        group.add(arrow);
        // Hidden until confirmed by a second sighting (set below).
        group.visible = false;
        three.trackGroup.add(group);
        slot = { group, type: trk.type, hits: 0 };
        slotsRef.current.set(trk.id, slot);
      }
      slot.hits = (slot.hits || 0) + 1;
      slot.missed = 0;
      slot.group.position.set(trk.x, 0, trk.y);
      slot.group.rotation.y = -trk.orientation;
      const confirmed = slot.hits >= TRACK_CONFIRM;
      slot.group.visible = confirmed && (!!showBoxes || !!showHeading);
      const arrow = slot.group.getObjectByName('arrow');
      if (arrow) arrow.visible = !!showHeading;
      // boxes themselves toggled via children visibility
      slot.group.children.forEach((c) => {
        if (c.name === 'arrow') return;
        c.visible = !!showBoxes;
      });
    });
    for (const [id, slot] of slotsRef.current) {
      if (seen.has(id)) continue;
      slot.missed = (slot.missed || 0) + 1;
      if (slot.missed > TRACK_MISS_GRACE) {
        three.trackGroup.remove(slot.group);
        disposeMesh(slot.group);
        slotsRef.current.delete(id);
      }
    }
  }, [three, tracks, showBoxes, showHeading]);

  // Cleanup on unmount.
  React.useEffect(() => () => {
    if (!three) return;
    for (const slot of slotsRef.current.values()) {
      three.trackGroup.remove(slot.group);
      disposeMesh(slot.group);
    }
    slotsRef.current.clear();
  }, [three]);

  // showIds is reserved for S9 (label sprites); skip in S6.
  return null;
}

window.TrackBoxes = TrackBoxes;
window.__buildBox = buildBox;

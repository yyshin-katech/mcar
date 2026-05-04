/* global React, THREE, window, useThree, useJsonTopic, useRosState,
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
  const ego = useRosState();
  const slotsRef = React.useRef(new Map()); // id → { group, type }

  // Reposition the trackGroup whenever ego pose or origin changes.
  React.useEffect(() => {
    if (!three) return;
    const origin = (map && map.origin) || [0, 0];
    const eEast  = (ego && ego.ego && ego.ego.east)  || 0;
    const eNorth = (ego && ego.ego && ego.ego.north) || 0;
    const eYaw   = (ego && ego.ego && ego.ego.yaw)   || 0;
    three.trackGroup.position.set(eEast - origin[0], 0, eNorth - origin[1]);
    three.trackGroup.rotation.y = -eYaw;
  });

  // Add/update/remove tracks keyed by id.
  React.useEffect(() => {
    if (!three || !tracks || !tracks.tracks) return undefined;
    const seen = new Set();
    tracks.tracks.forEach((trk) => {
      seen.add(trk.id);
      let slot = slotsRef.current.get(trk.id);
      if (!slot || slot.type !== trk.type
          || slot.size_x !== trk.size_x || slot.size_y !== trk.size_y) {
        // Geometry depends on type/size — rebuild if any changed.
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
        three.trackGroup.add(group);
        slot = { group, type: trk.type, size_x: trk.size_x, size_y: trk.size_y };
        slotsRef.current.set(trk.id, slot);
      }
      slot.group.position.set(trk.x, 0, trk.y);
      slot.group.rotation.y = -trk.orientation;
      slot.group.visible = !!showBoxes || !!showHeading;
      const arrow = slot.group.getObjectByName('arrow');
      if (arrow) arrow.visible = !!showHeading;
      // boxes themselves toggled via children visibility
      slot.group.children.forEach((c) => {
        if (c.name === 'arrow') return;
        c.visible = !!showBoxes;
      });
    });
    // Drop tracks no longer present.
    for (const [id, slot] of slotsRef.current) {
      if (!seen.has(id)) {
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

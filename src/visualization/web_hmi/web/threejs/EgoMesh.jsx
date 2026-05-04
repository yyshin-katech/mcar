/* global React, THREE, window, useThree, useJsonTopic, useRosState */

// IONIQ 5 ego mesh — sized to spec (4635×1890×1605 mm). Sits on Y=0,
// follows ego world position from /hmi/state, rotated by yaw.

const EGO_LENGTH = 4.635;
const EGO_WIDTH  = 1.890;
const EGO_HEIGHT = 1.605;
const EGO_COLOR  = '#00e5ff';

function EgoMesh() {
  const three = useThree();
  const map = useJsonTopic('/hmi/threejs/map', null);
  const ego = useRosState();
  const groupRef = React.useRef(null);

  React.useEffect(() => {
    if (!three) return undefined;
    const geom = new THREE.BoxGeometry(EGO_LENGTH, EGO_HEIGHT, EGO_WIDTH);
    const mat = new THREE.MeshBasicMaterial({
      color: EGO_COLOR, wireframe: true, transparent: true, opacity: 0.9,
    });
    const mesh = new THREE.Mesh(geom, mat);
    mesh.position.y = EGO_HEIGHT / 2;

    const arrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(EGO_LENGTH / 2, EGO_HEIGHT / 2, 0),
      2.5, EGO_COLOR, 1.0, 0.5,
    );

    const group = new THREE.Group();
    group.name = 'ego_mesh';
    group.add(mesh);
    group.add(arrow);
    three.egoGroup.add(group);
    groupRef.current = group;

    return () => {
      three.egoGroup.remove(group);
      geom.dispose();
      mat.dispose();
      // ArrowHelper.dispose() is r155+; guard.
      if (arrow.dispose) arrow.dispose();
    };
  }, [three]);

  // Track ego pose every render (cheap; React batches state updates).
  React.useEffect(() => {
    const g = groupRef.current;
    if (!g) return;
    const origin = (map && map.origin) || [0, 0];
    const eEast  = (ego && ego.ego && ego.ego.east)  || 0;
    const eNorth = (ego && ego.ego && ego.ego.north) || 0;
    const eYaw   = (ego && ego.ego && ego.ego.yaw)   || 0;
    g.position.set(eEast - origin[0], 0, eNorth - origin[1]);
    g.rotation.y = -eYaw;
  });

  return null;
}

window.EgoMesh = EgoMesh;

/* global React, THREE, window, useThree, useJsonTopic, useRosState */

// Switches the scene camera between two presets:
//   'iso' — chase-cam ~60m behind ego, 60m up, looking at ego
//   'top' — bird's-eye 100m above ego, north-up
// (plan also lists 'free'; OrbitControls deferred — use bag replay + iso.)

const ISO_BACK = 60;
const ISO_HEIGHT = 60;
const TOP_HEIGHT = 100;

function CameraController({ mode }) {
  const three = useThree();
  const map = useJsonTopic('/hmi/threejs/map', null);
  const ego = useRosState();

  React.useEffect(() => {
    if (!three) return;
    const cam = three.camera;
    const origin = (map && map.origin) || [0, 0];
    const eEast  = (ego && ego.ego && ego.ego.east)  || 0;
    const eNorth = (ego && ego.ego && ego.ego.north) || 0;
    const eYaw   = (ego && ego.ego && ego.ego.yaw)   || 0;
    // Scene has scale.z = -1, so a child placed at +north_delta has its world
    // z = -(north_delta). The camera lives in world space (not added to scene),
    // so we must compute ego's world z directly.
    const ex = eEast - origin[0];
    const ezWorld = -(eNorth - origin[1]);

    if (mode === 'top') {
      // Real north = world -Z (after flip). Put -Z toward screen-up.
      cam.up.set(0, 0, -1);
      cam.position.set(ex, TOP_HEIGHT, ezWorld);
    } else {
      cam.up.set(0, 1, 0);
      // Heading_world = (cos yaw, 0, -sin yaw) post-flip, so "behind" is
      // ego_world - ISO_BACK * heading_world.
      const cx = ex      - ISO_BACK * Math.cos(eYaw);
      const cz = ezWorld + ISO_BACK * Math.sin(eYaw);
      cam.position.set(cx, ISO_HEIGHT, cz);
    }
    cam.lookAt(ex, 0, ezWorld);
    cam.updateProjectionMatrix();
  });

  return null;
}

window.CameraController = CameraController;

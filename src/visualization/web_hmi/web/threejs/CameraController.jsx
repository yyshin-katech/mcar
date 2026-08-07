/* global React, THREE, window, useThree, useJsonTopic, useEgoPose */

// Switches the scene camera between two presets:
//   'iso' — chase-cam ~60m behind ego, 60m up, looking at ego
//   'top' — bird's-eye 100m above ego, north-up
// (plan also lists 'free'; OrbitControls deferred — use bag replay + iso.)
// Ego pose comes from /hmi/ego_pose (~50 Hz raw) instead of /hmi/state's
// 10 Hz throttled snapshot, matching rviz follow smoothness.

const ISO_BACK = 60;
const ISO_HEIGHT = 60;
const TOP_HEIGHT = 100;
const ZOOM_MIN = 0.2;
const ZOOM_MAX = 5.0;
const WHEEL_SENS = 0.001;

function CameraController({ mode }) {
  const three = useThree();
  const map = useJsonTopic('/hmi/threejs/map', null);
  const ego = useEgoPose();
  const [zoom, setZoom] = React.useState(1);

  // Mouse-wheel zoom on the scene canvas. zoom>1 = closer, zoom<1 = farther.
  React.useEffect(() => {
    if (!three) return undefined;
    const canvas = three.renderer.domElement;
    const onWheel = (e) => {
      e.preventDefault();
      const factor = Math.exp(-e.deltaY * WHEEL_SENS);
      setZoom((z) => Math.max(ZOOM_MIN, Math.min(ZOOM_MAX, z * factor)));
    };
    canvas.addEventListener('wheel', onWheel, { passive: false });
    return () => canvas.removeEventListener('wheel', onWheel);
  }, [three]);

  React.useEffect(() => {
    if (!three) return;
    const cam = three.camera;
    const origin = (map && map.origin) || [0, 0];
    const eEast  = (ego && ego.east)  || 0;
    const eNorth = (ego && ego.north) || 0;
    const eYaw   = (ego && ego.yaw)   || 0;
    // Scene has scale.z = -1, so a child placed at +north_delta has its world
    // z = -(north_delta). The camera lives in world space (not added to scene),
    // so we must compute ego's world z directly.
    const ex = eEast - origin[0];
    const ezWorld = -(eNorth - origin[1]);

    if (mode === 'top') {
      // Real north = world -Z (after flip). Put -Z toward screen-up.
      cam.up.set(0, 0, -1);
      cam.position.set(ex, TOP_HEIGHT / zoom, ezWorld);
    } else {
      cam.up.set(0, 1, 0);
      // Heading_world = (cos yaw, 0, -sin yaw) post-flip, so "behind" is
      // ego_world - ISO_BACK * heading_world.
      const back   = ISO_BACK   / zoom;
      const height = ISO_HEIGHT / zoom;
      const cx = ex      - back * Math.cos(eYaw);
      const cz = ezWorld + back * Math.sin(eYaw);
      cam.position.set(cx, height, cz);
    }
    cam.lookAt(ex, 0, ezWorld);
    cam.updateProjectionMatrix();
  });

  return null;
}

window.CameraController = CameraController;

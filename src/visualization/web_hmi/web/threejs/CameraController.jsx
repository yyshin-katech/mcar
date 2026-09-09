/* global React, THREE, window, useThree, useJsonTopic, useEgoPose */

// Camera modes:
//   'iso'   — chase-cam ~60 m behind ego, 60 m up, looking at ego
//   'top'   — bird's-eye 100 m above ego, north-up
//   'orbit' — rviz-style free look. Mouse/touch orbits, pans and dollies
//             around a focal point that keeps following ego, exactly like
//             rviz's Orbit view controller with a target frame set.
//
// Any drag on the canvas enters 'orbit' seeded from the current preset, so the
// view never jumps. Picking a preset radio — or double-clicking the canvas —
// snaps back.
//
// Bindings and sensitivity mirror rviz OrbitViewController:
//   left drag              → rotate   (yaw += dx*0.005, elevation += dy*0.005)
//   middle drag / shift+L  → pan the focal point in the ground plane
//   right drag / ctrl+L    → dolly    (drag down = zoom out)
//   wheel                  → dolly
//   one finger → rotate, two fingers → pinch dolly + pan
//
// Ego pose comes from /hmi/ego_pose (~50 Hz raw) instead of /hmi/state's
// 10 Hz throttled snapshot, matching rviz follow smoothness.

const ISO_BACK = 60;
const ISO_HEIGHT = 60;
const TOP_HEIGHT = 100;
const ZOOM_MIN = 0.2;
const ZOOM_MAX = 5.0;
const WHEEL_SENS = 0.001;

// Orbit tuning. ROT_SENS is rviz's 0.005 rad/px so the feel carries over.
const ROT_SENS = 0.005;
const DOLLY_DRAG_SENS = 0.005;
const PITCH_MIN = -1.35;   // rad; below the ground plane is allowed, like rviz
const PITCH_MAX = 1.45;    // stay off the pole so the ground basis stays valid
const DIST_MIN = 3;
const DIST_MAX = 3000;

// Scratch vectors — pan runs at pointer rate, so avoid per-event allocation.
const _right = new THREE.Vector3();
const _fwd = new THREE.Vector3();

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

function CameraController({ mode, onMode }) {
  const three = useThree();
  const map = useJsonTopic('/hmi/threejs/map', null);
  const ego = useEgoPose();
  const [zoom, setZoom] = React.useState(1);

  // Orbit state lives in a ref: pointer handlers must mutate it at input rate
  // without forcing a React render (the camera is positioned imperatively).
  const orbit = React.useRef({
    active: false, yaw: 0, pitch: 0.7, dist: 85, panE: 0, panN: 0,
  });
  // Preset to fall back to when the orbit is reset.
  const lastPreset = React.useRef('iso');
  // Newest props/state for the imperative handlers, which are installed once.
  const latest = React.useRef({});
  latest.current = { three, mode, map, ego, zoom, onMode };

  // Ego position as a map-frame delta from the map origin.
  const egoDelta = React.useCallback(() => {
    const { map: m, ego: e } = latest.current;
    const origin = (m && m.origin) || [0, 0];
    return {
      ex: ((e && e.east) || 0) - origin[0],
      en: ((e && e.north) || 0) - origin[1],
    };
  }, []);

  // Position the camera from the current mode + orbit state.
  const apply = React.useCallback(() => {
    const { three: t, mode: md, ego: e, zoom: z } = latest.current;
    if (!t) return;
    const cam = t.camera;
    const { ex, en } = egoDelta();
    const eYaw = (e && e.yaw) || 0;
    // Scene has scale.z = -1, so a child placed at +north_delta has its world
    // z = -(north_delta). The camera lives in world space (not added to scene),
    // so we must compute ego's world z directly.
    const ezWorld = -en;
    const o = orbit.current;

    if (o.active) {
      const fx = ex + o.panE;
      const fz = -(en + o.panN);
      const r = o.dist * Math.cos(o.pitch);
      cam.up.set(0, 1, 0);
      cam.position.set(fx + r * Math.cos(o.yaw),
                       o.dist * Math.sin(o.pitch),
                       fz + r * Math.sin(o.yaw));
      cam.lookAt(fx, 0, fz);
    } else if (md === 'top') {
      // Real north = world -Z (after flip). Put -Z toward screen-up.
      cam.up.set(0, 0, -1);
      cam.position.set(ex, TOP_HEIGHT / z, ezWorld);
      cam.lookAt(ex, 0, ezWorld);
    } else {
      cam.up.set(0, 1, 0);
      // Heading_world = (cos yaw, 0, -sin yaw) post-flip, so "behind" is
      // ego_world - ISO_BACK * heading_world.
      const back = ISO_BACK / z;
      const height = ISO_HEIGHT / z;
      cam.position.set(ex - back * Math.cos(eYaw), height,
                       ezWorld + back * Math.sin(eYaw));
      cam.lookAt(ex, 0, ezWorld);
    }
    cam.updateProjectionMatrix();
  }, [egoDelta]);

  // Seed orbit params from the preset the user is currently looking at, so
  // grabbing the canvas continues from the exact same viewpoint.
  const seed = React.useCallback(() => {
    const { ego: e, zoom: z } = latest.current;
    const o = orbit.current;
    if (lastPreset.current === 'top') {
      // Keep the eye height; PITCH_MAX is a hair off vertical.
      o.dist = (TOP_HEIGHT / z) / Math.sin(PITCH_MAX);
      o.pitch = PITCH_MAX;
    } else {
      const back = ISO_BACK / z;
      const height = ISO_HEIGHT / z;
      o.dist = Math.hypot(back, height);
      o.pitch = Math.atan2(height, back);
    }
    // Preset offset is (-back·cos eYaw, h, +back·sin eYaw) → yaw = π − eYaw.
    o.yaw = Math.PI - ((e && e.yaw) || 0);
    o.panE = 0;
    o.panN = 0;
  }, []);

  // Move the focal point in the ground plane, screen-relative (OrbitControls'
  // pan scaling: world metres per pixel at the focal distance).
  const panBy = React.useCallback((canvas, dx, dy) => {
    const { three: t } = latest.current;
    if (!t) return;
    const cam = t.camera;
    const o = orbit.current;
    const k = 2 * Math.tan((cam.fov * Math.PI / 180) / 2) * o.dist
              / (canvas.clientHeight || 1);
    // lookAt() only writes the quaternion, so refresh the matrices before
    // reading the basis — otherwise the axes lag a frame (or are identity
    // before the first render) and the pan drifts off-axis.
    cam.updateMatrixWorld();
    _right.setFromMatrixColumn(cam.matrixWorld, 0);
    cam.getWorldDirection(_fwd);
    _right.y = 0;
    _fwd.y = 0;
    if (_right.lengthSq() < 1e-9 || _fwd.lengthSq() < 1e-9) return;
    _right.normalize();
    _fwd.normalize();
    // Drag right → focal moves screen-left → world appears to follow the cursor.
    const wx = (-_right.x * dx + _fwd.x * dy) * k;
    const wz = (-_right.z * dx + _fwd.z * dy) * k;
    o.panE += wx;
    o.panN -= wz;   // world z = -(north delta)
  }, []);

  // Re-seat the camera on every render (ego updates at ~50 Hz).
  React.useEffect(apply);

  // Preset switch: reset the orbit unless the user asked for orbit itself.
  React.useEffect(() => {
    if (mode === 'iso' || mode === 'top') lastPreset.current = mode;
    if (mode === 'orbit') {
      if (!orbit.current.active) { seed(); orbit.current.active = true; }
    } else {
      orbit.current.active = false;
    }
    apply();
  }, [mode, seed, apply]);

  // Pointer / wheel input. Installed once per renderer.
  React.useEffect(() => {
    if (!three) return undefined;
    const canvas = three.renderer.domElement;
    const prevTouch = canvas.style.touchAction;
    const prevCursor = canvas.style.cursor;
    canvas.style.touchAction = 'none';   // stop browser scroll/zoom gestures
    canvas.style.cursor = 'grab';

    const pts = new Map();     // pointerId → last client position
    let gesture = null;        // 'rotate' | 'pan' | 'dolly' | 'pinch'
    let pinchDist = 0;

    const twoDist = () => {
      const it = pts.values();
      const a = it.next().value;
      const b = it.next().value;
      return (a && b) ? Math.hypot(a.x - b.x, a.y - b.y) : 0;
    };

    const beginOrbit = () => {
      const o = orbit.current;
      if (o.active) return;
      seed();
      o.active = true;
      const { mode: md, onMode: cb } = latest.current;
      if (cb && md !== 'orbit') cb('orbit');
    };

    const onDown = (e) => {
      e.preventDefault();
      try { canvas.setPointerCapture(e.pointerId); } catch (err) { /* ignore */ }
      pts.set(e.pointerId, { x: e.clientX, y: e.clientY });
      if (pts.size >= 2) {
        gesture = 'pinch';
        pinchDist = twoDist();
      } else if (e.pointerType !== 'mouse') {
        gesture = 'rotate';
      } else if (e.button === 1 || e.shiftKey) {
        gesture = 'pan';
      } else if (e.button === 2 || e.ctrlKey) {
        gesture = 'dolly';
      } else {
        gesture = 'rotate';
      }
      beginOrbit();
      canvas.style.cursor = gesture === 'pan' ? 'move' : 'grabbing';
    };

    const onMove = (e) => {
      const p = pts.get(e.pointerId);
      if (!p) return;
      const dx = e.clientX - p.x;
      const dy = e.clientY - p.y;
      p.x = e.clientX;
      p.y = e.clientY;
      const o = orbit.current;

      if (gesture === 'pinch' && pts.size >= 2) {
        const d = twoDist();
        if (pinchDist > 0 && d > 0) {
          o.dist = clamp(o.dist * (pinchDist / d), DIST_MIN, DIST_MAX);
        }
        pinchDist = d;
        panBy(canvas, dx / 2, dy / 2);   // both fingers contribute half
      } else if (gesture === 'pan') {
        panBy(canvas, dx, dy);
      } else if (gesture === 'dolly') {
        o.dist = clamp(o.dist * Math.exp(dy * DOLLY_DRAG_SENS),
                       DIST_MIN, DIST_MAX);
      } else {
        // Drag right → camera orbits counter-clockwise; drag down → camera
        // rises toward a top-down view (rviz / OrbitControls convention).
        o.yaw += dx * ROT_SENS;
        o.pitch = clamp(o.pitch + dy * ROT_SENS, PITCH_MIN, PITCH_MAX);
      }
      apply();
    };

    const onUp = (e) => {
      pts.delete(e.pointerId);
      try { canvas.releasePointerCapture(e.pointerId); } catch (err) { /* ignore */ }
      if (pts.size === 0) {
        gesture = null;
        canvas.style.cursor = 'grab';
      } else if (pts.size === 1) {
        gesture = 'rotate';
      }
    };

    const onWheel = (e) => {
      e.preventDefault();
      const o = orbit.current;
      if (o.active) {
        o.dist = clamp(o.dist * Math.exp(e.deltaY * WHEEL_SENS),
                       DIST_MIN, DIST_MAX);
        apply();
      } else {
        // Preset modes keep the original zoom-factor behaviour.
        const factor = Math.exp(-e.deltaY * WHEEL_SENS);
        setZoom((z) => clamp(z * factor, ZOOM_MIN, ZOOM_MAX));
      }
    };

    // Double-click restores the last preset — the quick way out of a bad angle.
    const onDblClick = () => {
      const o = orbit.current;
      o.active = false;
      o.panE = 0;
      o.panN = 0;
      const { mode: md, onMode: cb } = latest.current;
      if (cb && md === 'orbit') cb(lastPreset.current);
      apply();
    };

    const onCtx = (e) => e.preventDefault();   // right-drag must not open a menu

    canvas.addEventListener('pointerdown', onDown);
    canvas.addEventListener('pointermove', onMove);
    canvas.addEventListener('pointerup', onUp);
    canvas.addEventListener('pointercancel', onUp);
    canvas.addEventListener('wheel', onWheel, { passive: false });
    canvas.addEventListener('dblclick', onDblClick);
    canvas.addEventListener('contextmenu', onCtx);
    return () => {
      canvas.removeEventListener('pointerdown', onDown);
      canvas.removeEventListener('pointermove', onMove);
      canvas.removeEventListener('pointerup', onUp);
      canvas.removeEventListener('pointercancel', onUp);
      canvas.removeEventListener('wheel', onWheel);
      canvas.removeEventListener('dblclick', onDblClick);
      canvas.removeEventListener('contextmenu', onCtx);
      canvas.style.touchAction = prevTouch;
      canvas.style.cursor = prevCursor;
    };
  }, [three, seed, apply, panBy]);

  return null;
}

window.CameraController = CameraController;

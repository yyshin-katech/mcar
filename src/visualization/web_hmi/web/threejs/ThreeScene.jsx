/* global React, THREE, window */

// ThreeContext exposes scene/camera/renderer to children components.
// World axes:  +X = east, +Z = north, +Y = up (height). yaw rotates around +Y.

const ThreeContext = React.createContext(null);

function useThree() {
  return React.useContext(ThreeContext);
}

function ThreeScene({ children }) {
  const canvasRef = React.useRef(null);
  const [api, setApi] = React.useState(null);

  React.useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || typeof THREE === 'undefined') return undefined;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color('#04060a');
    // Three.js right-handed +X/+Y/+Z makes "looking east" put north on screen
    // right — opposite of map intuition (north on left when facing east). Flip
    // world Z so geometry placed at +north_delta visually appears in -Z (real
    // north). Camera math (CameraController) compensates by negating ego.z and
    // using cam.up=(0,0,-1) in top mode.
    scene.scale.z = -1;

    const camera = new THREE.PerspectiveCamera(50, 1, 0.5, 5000);
    camera.position.set(0, 60, 60);
    camera.lookAt(0, 0, 0);

    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio || 1);

    scene.add(new THREE.HemisphereLight('#9bd1ff', '#1a2530', 0.7));
    const dir = new THREE.DirectionalLight('#ffffff', 0.6);
    dir.position.set(10, 50, 10);
    scene.add(dir);

    const grid = new THREE.GridHelper(80, 80, '#0e2b3d', '#0a1c28');
    grid.name = 'grid';
    scene.add(grid);

    const mapGroup  = new THREE.Group(); mapGroup.name  = 'map';
    const trackGroup = new THREE.Group(); trackGroup.name = 'tracks';
    const egoGroup  = new THREE.Group(); egoGroup.name  = 'ego';
    scene.add(mapGroup);
    scene.add(trackGroup);
    scene.add(egoGroup);

    // Embedded mode: canvas parent has measurable size (e.g. F1 main grid area).
    // Fullscreen mode: parent has 0 size, fall back to viewport - panel width.
    const resize = () => {
      const host = canvas.parentElement;
      let w, h;
      if (host && host.clientWidth > 0 && host.clientHeight > 0) {
        w = host.clientWidth;
        h = host.clientHeight;
      } else {
        w = window.innerWidth - 260;
        h = window.innerHeight;
      }
      renderer.setSize(w, h, false);
      canvas.style.width = w + 'px';
      canvas.style.height = h + 'px';
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    };
    window.addEventListener('resize', resize);
    const ro = (typeof ResizeObserver !== 'undefined' && canvas.parentElement)
      ? new ResizeObserver(resize) : null;
    if (ro) ro.observe(canvas.parentElement);
    resize();

    let raf = 0;
    const loop = () => {
      raf = requestAnimationFrame(loop);
      renderer.render(scene, camera);
    };
    loop();

    setApi({ scene, camera, renderer, mapGroup, trackGroup, egoGroup });

    return () => {
      cancelAnimationFrame(raf);
      window.removeEventListener('resize', resize);
      if (ro) ro.disconnect();
      renderer.dispose();
      // dispose child geometries/materials best-effort
      scene.traverse((obj) => {
        if (obj.geometry) obj.geometry.dispose();
        if (obj.material) {
          const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
          mats.forEach((m) => m && m.dispose && m.dispose());
        }
      });
    };
  }, []);

  return (
    <>
      <canvas ref={canvasRef} id="scene" />
      {api ? (
        <ThreeContext.Provider value={api}>{children}</ThreeContext.Provider>
      ) : null}
    </>
  );
}

window.ThreeContext = ThreeContext;
window.useThree = useThree;
window.ThreeScene = ThreeScene;

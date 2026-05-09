/* global React, THREE, window, useThree, useJsonTopic, useEgoPose */

// IONIQ 5 ego mesh — loads a glTF model and normalizes it to spec
// (4635×1890×1605 mm). Sits on Y=0, follows ego world position from
// /hmi/ego_pose (~50 Hz raw, rviz-grade smoothness), rotated by yaw.
// While the glTF is loading (or if it fails), a wireframe box of the
// same dimensions is shown as fallback.
//
// Model: "Hyundai Ioniq 5 - Lowpoly" by andikapratamaw (CC-BY-4.0).
// See vendor/hyundai_ioniq_5_-_lowpoly/license.txt for attribution.

const EGO_LENGTH = 4.635;
const EGO_WIDTH  = 1.890;
const EGO_HEIGHT = 1.605;
const EGO_COLOR  = '#00e5ff';
// IONIQ 5 Magnetic Silver 톤. M_Ioniq material(차체)의 baseColorTexture에
// 곱해지는 tint이므로 너무 어두우면 디테일이 사라진다.
const EGO_BODY_COLOR = '#BFC2C7';
const EGO_GLTF_URL = 'vendor/hyundai_ioniq_5_-_lowpoly/scene.gltf';

function disposeTree(obj) {
  obj.traverse((c) => {
    if (c.geometry) c.geometry.dispose();
    if (c.material) {
      const ms = Array.isArray(c.material) ? c.material : [c.material];
      ms.forEach((m) => m && m.dispose && m.dispose());
    }
  });
}

function buildFallback() {
  const geom = new THREE.BoxGeometry(EGO_LENGTH, EGO_HEIGHT, EGO_WIDTH);
  const mat = new THREE.MeshBasicMaterial({
    color: EGO_COLOR, wireframe: true, transparent: true, opacity: 0.5,
  });
  const mesh = new THREE.Mesh(geom, mat);
  mesh.position.y = EGO_HEIGHT / 2;
  mesh.name = 'fallback';
  return mesh;
}

// Wrap the loaded glTF scene so the longest horizontal axis points along
// +X (forward), the model is centered horizontally, and the wheels sit at
// Y=0. Returns the wrapper group.
function wrapModel(gltfScene) {
  const pivot = new THREE.Group();
  pivot.add(gltfScene);

  // Inspect native size to align the long axis with +X. The +180° term is
  // because the Sketchfab IONIQ 5 model's native long-axis points to the
  // trunk, not the bonnet — without it the car drives backwards.
  const nativeBox = new THREE.Box3().setFromObject(gltfScene);
  const nativeSize = nativeBox.getSize(new THREE.Vector3());
  const xLonger = nativeSize.x >= nativeSize.z;
  pivot.rotation.y = (xLonger ? 0 : -Math.PI / 2) + Math.PI;

  // Uniform scale: longest horizontal extent → EGO_LENGTH.
  const nativeLen = xLonger ? nativeSize.x : nativeSize.z;
  if (nativeLen > 0) pivot.scale.setScalar(EGO_LENGTH / nativeLen);

  // Recompute bbox after rotation+scale, then translate so wheels touch
  // y=0 and the centroid sits on the X/Z origin.
  pivot.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(pivot);
  const center = box.getCenter(new THREE.Vector3());
  pivot.position.set(-center.x, -box.min.y, -center.z);

  // The parent scene has scale.z=-1 which inverts triangle winding; force
  // double-sided rendering so the body doesn't appear inside-out.
  // Also tint the body material (M_Ioniq) to silver-gray.
  gltfScene.traverse((c) => {
    if (c.isMesh && c.material) {
      const ms = Array.isArray(c.material) ? c.material : [c.material];
      ms.forEach((m) => {
        m.side = THREE.DoubleSide;
        if (m && m.name === 'M_Ioniq' && m.color) m.color.set(EGO_BODY_COLOR);
      });
    }
  });

  return pivot;
}

function EgoMesh() {
  const three = useThree();
  const map = useJsonTopic('/hmi/threejs/map', null);
  const ego = useEgoPose();
  const groupRef = React.useRef(null);

  React.useEffect(() => {
    if (!three) return undefined;
    const group = new THREE.Group();
    group.name = 'ego_mesh';

    const fallback = buildFallback();
    group.add(fallback);

    const arrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(EGO_LENGTH / 2, EGO_HEIGHT / 2, 0),
      2.5, EGO_COLOR, 1.0, 0.5,
    );
    arrow.name = 'heading_arrow';
    group.add(arrow);

    three.egoGroup.add(group);
    groupRef.current = group;

    let canceled = false;
    if (typeof THREE.GLTFLoader === 'function') {
      const loader = new THREE.GLTFLoader();
      loader.load(EGO_GLTF_URL, (gltf) => {
        if (canceled) {
          disposeTree(gltf.scene);
          return;
        }
        const wrapped = wrapModel(gltf.scene);
        wrapped.name = 'ioniq5_model';
        group.remove(fallback);
        disposeTree(fallback);
        group.add(wrapped);
      }, undefined, (err) => {
        // eslint-disable-next-line no-console
        console.warn('[EgoMesh] glTF load failed, keeping fallback:', err);
      });
    } else {
      // eslint-disable-next-line no-console
      console.warn('[EgoMesh] THREE.GLTFLoader missing — vendor script not loaded?');
    }

    return () => {
      canceled = true;
      three.egoGroup.remove(group);
      disposeTree(group);
      if (arrow.dispose) arrow.dispose();
    };
  }, [three]);

  // Track ego pose every render (cheap; React batches state updates).
  React.useEffect(() => {
    const g = groupRef.current;
    if (!g) return;
    const origin = (map && map.origin) || [0, 0];
    const eEast  = (ego && ego.east)  || 0;
    const eNorth = (ego && ego.north) || 0;
    const eYaw   = (ego && ego.yaw)   || 0;
    g.position.set(eEast - origin[0], 0, eNorth - origin[1]);
    g.rotation.y = -eYaw;
  });

  return null;
}

window.EgoMesh = EgoMesh;

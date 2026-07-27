/* global React, THREE, window, useThree, useJsonTopic, LAYER_STYLE */

// Convert JSON map payload (delta-coords from origin, EPSG:5179) to Three.js Object3Ds.
// World axes here: +X = east_delta, +Z = north_delta, +Y = up. Map lies on Y=0 plane.

function buildPolyline(line, color, width, alpha) {
  // line: [[de, dn], ...] → LineSegments with paired vertices.
  const n = line.length;
  if (n < 2) return null;
  const positions = new Float32Array((n - 1) * 2 * 3);
  for (let i = 0; i < n - 1; i++) {
    const [e0, n0] = line[i];
    const [e1, n1] = line[i + 1];
    const o = i * 6;
    positions[o + 0] = e0; positions[o + 1] = 0; positions[o + 2] = n0;
    positions[o + 3] = e1; positions[o + 4] = 0; positions[o + 5] = n1;
  }
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  const mat = new THREE.LineBasicMaterial({ color, linewidth: width || 1.0 });
  if (alpha != null && alpha < 1.0) {
    mat.transparent = true;
    mat.opacity = alpha;
  }
  return new THREE.LineSegments(geom, mat);
}

function buildPolygonOutline(ring, color, alpha) {
  // ring: [[de, dn], ...] outer ring → LineLoop on Y=0.
  const n = ring.length;
  if (n < 3) return null;
  const positions = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    positions[i * 3 + 0] = ring[i][0];
    positions[i * 3 + 1] = 0;
    positions[i * 3 + 2] = ring[i][1];
  }
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  const mat = new THREE.LineBasicMaterial({
    color, transparent: true, opacity: alpha || 0.4,
  });
  return new THREE.LineLoop(geom, mat);
}

function buildPointCloud(points, color, size) {
  if (!points || !points.length) return null;
  const positions = new Float32Array(points.length * 3);
  for (let i = 0; i < points.length; i++) {
    positions[i * 3 + 0] = points[i][0];
    positions[i * 3 + 1] = 0;
    positions[i * 3 + 2] = points[i][1];
  }
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  const mat = new THREE.PointsMaterial({
    color, size: size || 0.5, sizeAttenuation: true,
  });
  return new THREE.Points(geom, mat);
}

function buildLayer(name, layer) {
  const style = LAYER_STYLE[name];
  if (!style) return null;
  const group = new THREE.Group();
  group.name = name;
  if (layer.kind === 'polyline') {
    layer.data.forEach((line) => {
      const obj = buildPolyline(line, style.color, style.width, style.alpha);
      if (obj) group.add(obj);
    });
  } else if (layer.kind === 'polygon') {
    layer.data.forEach((ring) => {
      const obj = buildPolygonOutline(ring, style.color, style.alpha);
      if (obj) group.add(obj);
    });
  } else if (layer.kind === 'point') {
    const obj = buildPointCloud(layer.data, style.color, style.size);
    if (obj) group.add(obj);
  }
  return group;
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

function MapLayers({ layerVisibility }) {
  const three = useThree();
  const map = useJsonTopic('/hmi/threejs/map', null);
  const groupsRef = React.useRef({});

  React.useEffect(() => {
    if (!three || !map || !map.layers) return undefined;

    // Dispose previous build (handles map re-publish during dev).
    Object.values(groupsRef.current).forEach((g) => {
      three.mapGroup.remove(g);
      disposeGroup(g);
    });
    groupsRef.current = {};

    Object.entries(map.layers).forEach(([name, layer]) => {
      const g = buildLayer(name, layer);
      if (!g) return;
      three.mapGroup.add(g);
      groupsRef.current[name] = g;
    });

    return () => {
      Object.values(groupsRef.current).forEach((g) => {
        three.mapGroup.remove(g);
        disposeGroup(g);
      });
      groupsRef.current = {};
    };
  }, [three, map]);

  // Apply visibility on each render.
  React.useEffect(() => {
    if (!layerVisibility) return;
    Object.entries(groupsRef.current).forEach(([name, g]) => {
      g.visible = layerVisibility[name] !== false;
    });
  });

  return null;
}

window.MapLayers = MapLayers;
window.__buildPolyline = buildPolyline;
window.__buildPolygonOutline = buildPolygonOutline;
window.__buildPointCloud = buildPointCloud;

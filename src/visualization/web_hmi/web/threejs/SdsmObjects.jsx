/* global React, THREE, window, useThree, useJsonTopic */

// V2X SDSM (SAE J3224) — RSU 가 검지한 오브젝트 표시.
//
// 브리지(web_hmi_threejs_bridge.py `_on_sdsm`)가 /obu/sdsm 를 받아
// 절대 EPSG:5179 좌표로 /hmi/threejs/sdsm 에 발행한다. 여기서는 MapLayers /
// BlockZones / CrosswalkZones 와 동일하게 map `origin` 으로 시프트해서
// mapGroup 에 올린다 (+X=east_delta, +Z=north_delta, scene scale.z=-1).
//
// 좌표 변환(로컬 offset → ENU)은 전적으로 브리지 쪽에서 끝난다. 원천은
// ~/노바코스GPS변환.txt (RSU 송신측 코드): offsetX/offsetY 는 dm 단위이고
// offsetX 는 부호가 반전돼 실려온다. rviz_filter 의 SDSM 마커 경로는
// 이번 작업에서 손대지 않았으므로 두 표시는 독립이다.
//
// 자차 인지(/hmi/threejs/tracks, 분홍/보라 계열)와 구분되도록 V2X 오브젝트는
// 청록~노랑 계열 + 상단 비콘 막대로 그린다.

// objType (J3224 ObjectType): 0=unknown, 1=vehicle, 2=vru, 3=animal.
// 치수는 rviz_filter.cpp:413-435 와 동일하게 맞춘다.
const SDSM_DIMS = {
  0: { x: 2.0, y: 2.0, h: 1.5 },
  1: { x: 4.5, y: 2.0, h: 1.5 },
  2: { x: 0.8, y: 0.8, h: 1.8 },
  3: { x: 2.0, y: 2.0, h: 1.5 },
};
const SDSM_COLORS = {
  0: 0x00ffa3,   // unknown — 실데이터는 전량 이 값
  1: 0x00c2ff,   // vehicle
  2: 0xffe14d,   // vru
  3: 0xb388ff,   // animal
};
const SDSM_REF_COLOR = 0x4d9fff;   // RSU 기준점(refPos)
const SDSM_STALE_S = 3.0;          // 이 시간 이상 무수신이면 전부 숨김
const SDSM_MISS_GRACE = 3;         // 연속 미검출 프레임 허용치

function sdsmDims(type) { return SDSM_DIMS[type] || SDSM_DIMS[0]; }
function sdsmColor(type) {
  return (SDSM_COLORS[type] === undefined) ? SDSM_COLORS[0] : SDSM_COLORS[type];
}

function buildSdsmObject(type) {
  const dims = sdsmDims(type);
  const color = sdsmColor(type);
  const group = new THREE.Group();

  const box = new THREE.Mesh(
    new THREE.BoxGeometry(dims.x, dims.h, dims.y),
    new THREE.MeshBasicMaterial({
      color, wireframe: true, transparent: true, opacity: 0.95,
    }),
  );
  box.position.y = dims.h / 2;
  box.name = 'box';
  group.add(box);

  // 진행방향 화살표 (+X = 물체 전방; group.rotation.y = -yaw 로 회전).
  const arrow = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, dims.h / 2, 0),
    Math.max(1.0, dims.x * 0.8), color, 0.5, 0.3,
  );
  arrow.name = 'arrow';
  group.add(arrow);

  // V2X 임을 한눈에 구분하는 비콘 막대 (자차 인지 박스에는 없음).
  const beacon = new THREE.Mesh(
    new THREE.CylinderGeometry(0.06, 0.06, 2.2, 6),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.6 }),
  );
  beacon.position.y = dims.h + 1.1;
  beacon.name = 'beacon';
  group.add(beacon);

  return group;
}

function buildRefMarker() {
  const group = new THREE.Group();
  const cone = new THREE.Mesh(
    new THREE.ConeGeometry(0.9, 2.4, 8),
    new THREE.MeshBasicMaterial({
      color: SDSM_REF_COLOR, wireframe: true, transparent: true, opacity: 0.9,
    }),
  );
  cone.position.y = 1.2;
  group.add(cone);
  return group;
}

function disposeSdsm(obj) {
  obj.traverse((c) => {
    if (c.geometry) c.geometry.dispose();
    if (c.material) {
      const ms = Array.isArray(c.material) ? c.material : [c.material];
      ms.forEach((m) => m && m.dispose && m.dispose());
    }
  });
}

function SdsmObjects({ showSdsm }) {
  const three = useThree();
  const map = useJsonTopic('/hmi/threejs/map', null);
  const sdsm = useJsonTopic('/hmi/threejs/sdsm', null);
  const rootRef = React.useRef(null);
  const refMarkRef = React.useRef(null);
  const slotsRef = React.useRef(new Map());   // objectID → { group, type, missed }

  // Root group lives under mapGroup; rebuilt only when the map origin changes.
  React.useEffect(() => {
    if (!three || !map || !map.origin) return undefined;
    const root = new THREE.Group();
    root.name = 'sdsm';
    three.mapGroup.add(root);
    rootRef.current = root;

    const refMark = buildRefMarker();
    refMark.visible = false;
    root.add(refMark);
    refMarkRef.current = refMark;

    return () => {
      three.mapGroup.remove(root);
      disposeSdsm(root);
      rootRef.current = null;
      refMarkRef.current = null;
      slotsRef.current.clear();
    };
  }, [three, map]);

  // Add / update / retire objects on every SDSM emission.
  React.useEffect(() => {
    const root = rootRef.current;
    const origin = (map && map.origin) || null;
    if (!root || !origin) return;

    const on = !!showSdsm;
    const objs = (sdsm && sdsm.objects) || [];
    // stamp 은 브리지의 rospy.get_time() (epoch 초) — 브라우저 시계와 다를 수
    // 있으므로 절대 비교 대신 "새 메시지 도착" 여부로만 신선도를 본다.
    const seen = new Set();

    if (refMarkRef.current) {
      const ref = sdsm && sdsm.ref;
      if (on && ref) {
        refMarkRef.current.position.set(ref.east - origin[0], 0, ref.north - origin[1]);
        refMarkRef.current.visible = true;
      } else {
        refMarkRef.current.visible = false;
      }
    }

    objs.forEach((o) => {
      seen.add(o.id);
      let slot = slotsRef.current.get(o.id);
      if (!slot || slot.type !== o.type) {
        if (slot) { root.remove(slot.group); disposeSdsm(slot.group); }
        const group = buildSdsmObject(o.type);
        group.name = 'sdsm#' + o.id;
        root.add(group);
        slot = { group, type: o.type };
        slotsRef.current.set(o.id, slot);
      }
      slot.missed = 0;
      slot.group.position.set(o.east - origin[0], 0, o.north - origin[1]);
      slot.group.rotation.y = -(o.yaw || 0);
      slot.group.visible = on;
    });

    for (const [id, slot] of slotsRef.current) {
      if (seen.has(id)) continue;
      slot.missed = (slot.missed || 0) + 1;
      if (slot.missed > SDSM_MISS_GRACE) {
        root.remove(slot.group);
        disposeSdsm(slot.group);
        slotsRef.current.delete(id);
      } else {
        slot.group.visible = on;
      }
    }
  }, [three, map, sdsm, showSdsm]);

  return null;
}
window.SdsmObjects = SdsmObjects;

// 좌하단 상태 칩 — SDSM 수신 여부/개수/RSU ID. 토글이 켜져 있을 때만 뜬다.
// 무수신이 SDSM_STALE_S 를 넘으면 회색 "V2X SDSM 무수신" 으로 바뀐다.
function SdsmStatus({ showSdsm }) {
  const sdsm = useJsonTopic('/hmi/threejs/sdsm', null);
  const [, tick] = React.useReducer((v) => v + 1, 0);
  const seenAtRef = React.useRef(0);

  React.useEffect(() => {
    if (sdsm) seenAtRef.current = Date.now() / 1000.0;
  }, [sdsm]);
  React.useEffect(() => {
    const h = setInterval(tick, 1000);
    return () => clearInterval(h);
  }, []);

  if (!showSdsm) return null;
  const fresh = sdsm && (Date.now() / 1000.0 - seenAtRef.current) < SDSM_STALE_S;
  const n = (sdsm && sdsm.objects && sdsm.objects.length) || 0;
  const color = fresh ? '#00ffa3' : '#7a8492';
  const text = fresh
    ? `V2X SDSM · ${n} obj · RSU ${(sdsm && sdsm.src) || '—'}`
    : 'V2X SDSM 무수신';
  return (
    <div style={{
      position: "absolute", left: 12, bottom: 12, zIndex: 20,
      padding: "5px 12px", borderRadius: 6,
      background: "rgba(8,12,18,0.82)",
      border: `1px solid ${color}`,
      fontFamily: "Inter, Pretendard, system-ui, sans-serif",
      fontSize: 11, letterSpacing: "0.04em", color,
      pointerEvents: "none", userSelect: "none",
    }}>
      {text}
    </div>
  );
}
window.SdsmStatus = SdsmStatus;

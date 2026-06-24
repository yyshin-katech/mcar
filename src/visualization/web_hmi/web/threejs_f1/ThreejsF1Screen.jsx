/* global React, window,
          F1HMIShell, F1Tokens,
          ThreeScene, MapLayers, TrackBoxes, TrackPointClouds, EgoMesh,
          CameraController, ControlPanel,
          DEFAULT_LAYER_VIS,
          useRosConnection, useRosState, useDiagnostics, useTopicHz,
          useObjects, usePopup, useTraffic, useBag, useMap */

// Combined HMI: F1 left dashboard + Three.js main scene.
// Reuses F1HMIShell as the chrome (top bar / left panel / bottom strip),
// injects a Three.js scene into the `mainContent` slot, and floats the
// ControlPanel at the top-right of the main area.

const T_F1 = window.F1Tokens;

const GEAR_LABEL_F1 = { 1: "P", 2: "R", 3: "N", 4: "D" };

function radToDegF1(r) { return (typeof r === "number" ? (r * 180 / Math.PI) : 0); }

function mapTrafficPhaseF1(color) {
  return ["OFF", "GREEN", "AMBER", "RED"][color] || "OFF";
}

function formatRtkF1(rtk) {
  if (rtk === 2) return { label: "RTK FIX",   color: T_F1.green };
  if (rtk === 1) return { label: "RTK FLOAT", color: T_F1.amber };
  return { label: "NO RTK", color: T_F1.text3 };
}

function fmtSigmaCmF1(m) {
  if (typeof m !== "number" || m <= 0) return "—";
  return (m * 100).toFixed(2).padStart(5, "0") + " cm";
}

function fmtKmhF1(v) {
  if (typeof v !== "number") return "—";
  return v.toFixed(1) + " km/h";
}

function statusFromCodeF1(code) {
  if (code === 2) return ["ERR",  T_F1.red];
  if (code === 1) return ["WARN", T_F1.amber];
  return ["OK", T_F1.green];
}

const HEALTH_ROWS_F1 = [
  { label: "GPS-RTK", key: "gps",   detail: (s, hz) => s.rtkText || `${hz} Hz` },
  { label: "VCU",     key: "vcu",   detail: (_,  hz) => `${hz} Hz` },
  { label: "LIDAR",   key: "lidar", detail: (_,  hz) => `${hz} Hz` },
  { label: "RADAR",   key: "radar", detail: (_,  hz) => `${hz} Hz` },
  { label: "CAMERA",  key: "cam",   detail: (_,  hz) => `${hz} Hz` },
  { label: "V2X",     key: "v2x",   detail: (_,  hz) => `${hz} Hz` },
];

function buildHealthF1(diag, hz, gpsRtkLabel) {
  return HEALTH_ROWS_F1.map((r) => {
    const code = (diag.status && diag.status[r.key]) || 0;
    const [st, c] = statusFromCodeF1(code);
    const hzVal = (hz[r.key] || 0).toFixed(1);
    const ctx = r.key === "gps" ? { rtkText: `${gpsRtkLabel} · ${hzVal} Hz` } : {};
    return [r.label, r.detail(ctx, hzVal), st, c];
  });
}

function buildSummaryF1(diag) {
  return HEALTH_ROWS_F1.slice(0, 4).map((r) => {
    const code = (diag.status && diag.status[r.key]) || 0;
    if (code === 2) return T_F1.red;
    if (code === 1) return T_F1.amber;
    return T_F1.green;
  });
}

function pad2F1(n) { return String(n).padStart(2, "0"); }

function formatUtcF1(d) {
  return `${pad2F1(d.getUTCHours())}:${pad2F1(d.getUTCMinutes())}:${pad2F1(d.getUTCSeconds())}`;
}

function formatKstF1(d) {
  const kst = new Date(d.getTime() + 9 * 3600 * 1000);
  return `${pad2F1(kst.getUTCHours())}:${pad2F1(kst.getUTCMinutes())}:${pad2F1(kst.getUTCSeconds())}`;
}

function useClockF1(intervalMs = 250) {
  const [now, setNow] = React.useState(() => Date.now());
  React.useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), intervalMs);
    return () => clearInterval(id);
  }, [intervalMs]);
  return now;
}

// Map-canvas overlay: "전방 직진 주행 금지" banner + 좌/우 진행 가능 화살표.
// Banner shows on (on_block_link || do_not_go_forward); the ←/→ arrows (좌/우
// 진행 가능) show only on on_block_link. Subscribes /hmi/state independently so
// it stays in sync with BlockZones without threading props through the shell.
function BlockBanner() {
  const state = useRosState();
  const onLink = state && state.on_block_link === 1;
  const doNotGo = state && state.do_not_go_forward === 1;
  if (!onLink && !doNotGo) return null;
  return (
    <div style={{
      position: "absolute", top: 64, left: "50%", transform: "translateX(-50%)",
      zIndex: 30, display: "flex", alignItems: "center", gap: 18,
      padding: "12px 26px", borderRadius: 8,
      background: "rgba(120,12,12,0.62)",
      border: "1px solid rgba(255,80,80,0.85)",
      boxShadow: "0 6px 24px rgba(0,0,0,0.5)",
      color: "#ffdede", fontWeight: 700, letterSpacing: "0.04em",
      fontFamily: "Inter, Pretendard, system-ui, sans-serif",
      pointerEvents: "none", userSelect: "none",
    }}>
      {onLink ? <span style={{ fontSize: 30, color: "#ff7a7a" }}>←</span> : null}
      <span style={{ fontSize: 22, color: "#ff5050", textShadow: "0 1px 4px rgba(0,0,0,0.6)" }}>
        ⚠ 전방 직진 주행 금지
      </span>
      {onLink ? <span style={{ fontSize: 30, color: "#ff7a7a" }}>→</span> : null}
    </div>
  );
}
window.BlockBanner = BlockBanner;

function ThreejsF1Screen() {
  // ─── F1 dashboard data ─────────────────────────────────────
  const conn  = useRosConnection();
  const state = useRosState();
  const diag  = useDiagnostics();
  const hz    = useTopicHz();
  const objects = useObjects();
  const popup = usePopup();
  const traffic = useTraffic();
  const bag = useBag();
  const map = useMap();

  const now = useClockF1(250);
  const pageLoadAtRef = React.useRef(Date.now());

  const rtk = formatRtkF1(state.gps && state.gps.rtk);
  const speedNum = (typeof state.speed === "number") ? state.speed : 0;
  const speedLimitNum = (typeof state.speed_limit === "number") ? state.speed_limit : 0;
  const trafficPhase = mapTrafficPhaseF1(traffic.color);
  const trafficRemain = Math.round((traffic.time_decisec || 0) / 10);
  const yawDeg = radToDegF1(state.ego && state.ego.yaw);
  const engaged = state.mode === 1;
  const oddNominal = state.on_odd === 0;
  const oddBanner = (popup && popup.severity && popup.severity !== "info" && popup.text) ? popup.text : null;
  const health = buildHealthF1(diag, hz, rtk.label);
  const summary = buildSummaryF1(diag);

  const dNow = new Date(now);
  const utcText = formatUtcF1(dNow);
  const kstText = formatKstF1(dNow);
  const tickText = ((now - pageLoadAtRef.current) / 1000).toFixed(3) + "s";
  const anyTopicOk = Object.values(hz).some(v => v > 0.5);
  const rosOk = conn.connected && anyTopicOk;
  const rosLabel = !conn.connected ? "ROS · OFFLINE"
    : anyTopicOk ? "ROS · ONLINE"
    : "ROS · WAITING";
  const netText = (conn.lastMessageAgeMs === Infinity || !conn.connected)
    ? "—" : `${Math.round(conn.lastMessageAgeMs)}ms`;

  const sigE = fmtSigmaCmF1(state.gps && state.gps.lon_std);
  const sigN = fmtSigmaCmF1(state.gps && state.gps.lat_std);
  const heading = (typeof yawDeg === "number")
    ? (((yawDeg % 360) + 360) % 360).toFixed(1) + "°" : "—";
  const laneId = state.lane_label || "—";
  const linkId = state.link_id ? String(state.link_id) : "—";

  const dLim = (speedLimitNum > 0 ? (speedNum - speedLimitNum) : 0);
  const dLimStr = (speedLimitNum > 0
    ? (dLim >= 0 ? "+" : "−") + Math.abs(dLim).toFixed(1) + " km/h"
    : "—");
  const dLimColor = speedLimitNum > 0
    ? (dLim > 0 ? T_F1.amber : T_F1.green)
    : T_F1.text3;
  const bottom = [
    ["EGO-VEL",  fmtKmhF1(speedNum), T_F1.cyan],
    ["Δ-LIM",    dLimStr, dLimColor],
    ["TRACKS",   String((objects && objects.count) || 0), T_F1.text3],
    ["LATERAL",  "—", T_F1.text3],
    ["JERK",     "—", T_F1.text3],
    ["LEAD-D",   "—", T_F1.text3],
    ["LEAD-Δv",  "—", T_F1.text3],
    ["CPU",      "—", T_F1.text3],
  ];

  // ─── Three.js panel state ──────────────────────────────────
  const [layerVis,    setLayerVis]    = React.useState(window.DEFAULT_LAYER_VIS);
  const [showBoxes,   setShowBoxes]   = React.useState(true);
  const [showHeading, setShowHeading] = React.useState(true);
  const [showClouds,  setShowClouds]  = React.useState(true);
  const [pointSize,   setPointSize]   = React.useState(0.08);
  const [cameraMode,  setCameraMode]  = React.useState('iso');
  const [nearestOnly, setNearestOnly] = React.useState(false);
  const NEAREST_N = 5;

  // ROSBAG: include LiDAR/perception topics in the next recording (default on).
  // Read by the bridge at start_bag time, so we publish on every toggle.
  const [bagIncludeLidar, setBagIncludeLidar] = React.useState(true);
  const onBagLidarToggle = React.useCallback(() => {
    setBagIncludeLidar((prev) => {
      const next = !prev;
      conn.publishBagLidar(next);
      return next;
    });
  }, [conn]);
  // Re-assert the LiDAR flag right before starting, so a bridge restart can't
  // leave the next recording out of sync with the on-screen toggle.
  const onBagToggle = React.useCallback(() => {
    if (!(bag && bag.recording)) conn.publishBagLidar(bagIncludeLidar);
    conn.publishBagToggle();
  }, [conn, bag, bagIncludeLidar]);

  const onLayerVis = React.useCallback((name, val) => {
    setLayerVis((prev) => ({ ...prev, [name]: val }));
  }, []);

  // mainContent: Three.js scene + floating control panel.
  const mainContent = (
    <div style={{ position: "absolute", inset: 0, overflow: "hidden", background: "#04060a" }}>
      <window.ThreeScene>
        <window.MapLayers layerVisibility={layerVis} />
        <window.BlockZones />
        <window.TrackBoxes showBoxes={showBoxes} showHeading={showHeading} showIds={false}
                           nearestOnly={nearestOnly} nearestN={NEAREST_N} />
        <window.TrackPointClouds showClouds={showClouds} pointSize={pointSize}
                           nearestOnly={nearestOnly} nearestN={NEAREST_N} />
        <window.EgoMesh />
        <window.CameraController mode={cameraMode} />
      </window.ThreeScene>
      <window.BlockBanner />
      <div style={{
        position: "absolute", top: 140, right: 8, width: 240, maxHeight: "calc(92% - 140px)",
        zIndex: 10, overflowY: "auto",
        background: "rgba(8,12,18,0.85)",
        border: "1px solid rgba(0,229,255,0.18)",
        padding: 10, fontSize: 11,
        fontFamily: "Inter, Pretendard, system-ui, sans-serif",
        color: "#e6edf3",
      }}>
        <window.ControlPanel
          layerVis={layerVis} onLayerVis={onLayerVis}
          showBoxes={showBoxes} onShowBoxes={setShowBoxes}
          showHeading={showHeading} onShowHeading={setShowHeading}
          showClouds={showClouds} onShowClouds={setShowClouds}
          pointSize={pointSize} onPointSize={setPointSize}
          nearestOnly={nearestOnly} onNearestOnly={setNearestOnly} nearestN={NEAREST_N}
          cameraMode={cameraMode} onCameraMode={setCameraMode} />
      </div>
    </div>
  );

  return (
    <F1HMIShell
      nodeName="web_hmi_threejs"
      utcText={utcText} kstText={kstText} tickText={tickText} netText={netText}
      rosOk={rosOk} rosLabel={rosLabel}
      speed={speedNum} speedLimit={speedLimitNum}
      steeringAngle={typeof state.steering === "number" ? state.steering : 0}
      gearLetter={GEAR_LABEL_F1[state.gear] || "—"}
      thr="—" brk="—" accel="—"
      engaged={engaged}
      vMaxText={speedLimitNum > 0 ? String(speedLimitNum) : "—"}
      oddText={oddNominal ? "nominal" : "ODD 이탈"}
      trafficPhase={trafficPhase} trafficRemain={trafficRemain}
      laneId={laneId} linkId={linkId}
      sigE={sigE} sigN={sigN} sigU="—"
      hdop="—" sats="—" heading={heading}
      rtkLabel={rtk.label} rtkColor={rtk.color}
      health={health} summary={summary}
      objs={[]} oddBanner={oddBanner}
      mapPolylines={(map && map.polylines) || []}
      egoEast={(state.ego && state.ego.east) || 0}
      egoNorth={(state.ego && state.ego.north) || 0}
      egoYaw={(state.ego && state.ego.yaw) || 0}
      bagRecording={!!(bag && bag.recording)}
      bagInfo={(bag && bag.info) || ""}
      onBagToggle={onBagToggle}
      bagIncludeLidar={bagIncludeLidar}
      onBagLidarToggle={onBagLidarToggle}
      bottom={bottom}
      mainContent={mainContent}
    />
  );
}

window.ThreejsF1Screen = ThreejsF1Screen;

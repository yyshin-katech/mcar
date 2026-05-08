/* global React,
          F1HMIShell, F1Tokens,
          useRosConnection, useRosState, useDiagnostics, useTopicHz,
          useObjects, usePopup, useTraffic, useBag, useMap */

// F1 KATECH HMI screen — bridges 7 ROS hooks into F1HMIShell props.

const T = window.F1Tokens; // shared color palette

const GEAR_LABEL = { 1: "P", 2: "R", 3: "N", 4: "D" };

// ROS rad → deg
function radToDeg(r) {
  return (typeof r === "number" ? (r * 180 / Math.PI) : 0);
}

// /hmi/traffic.color: 0=none, 1=green, 2=amber, 3=red (from web_hmi_bridge.py:143)
function mapTrafficPhase(color) {
  return ["OFF", "GREEN", "AMBER", "RED"][color] || "OFF";
}

// /hmi/state.gps.rtk: 0=No, 1=Float, 2=Fix
function formatRtk(rtk) {
  if (rtk === 2) return { label: "RTK FIX",   color: T.green };
  if (rtk === 1) return { label: "RTK FLOAT", color: T.amber };
  return { label: "NO RTK", color: T.text3 };
}

// std meters → cm with 2-digit pad ("01.84 cm")
function fmtSigmaCm(m) {
  if (typeof m !== "number" || m <= 0) return "—";
  const cm = m * 100;
  return cm.toFixed(2).padStart(5, "0") + " cm";
}

function fmtKmh(v) {
  if (typeof v !== "number") return "—";
  return v.toFixed(1) + " km/h";
}

function fmtDeg(d) {
  if (typeof d !== "number") return "—";
  return d.toFixed(1) + "°";
}

// Severity → 0..2 status code
function statusFromCode(code) {
  if (code === 2) return ["ERR", T.red];
  if (code === 1) return ["WARN", T.amber];
  return ["OK", T.green];
}

// Map /hmi/diagnostics + /hmi/topic_hz → 6 health rows.
// Visual order: GPS-RTK, K-ADCU, LIDAR, RADAR, CAMERA, V2X.
// Bridge keys:    gps,    adcu,   lidar, radar, cam,    v2x
const HEALTH_ROWS = [
  { label: "GPS-RTK", key: "gps",   detail: (s, hz) => s.rtkText || `${hz} Hz` },
  { label: "K-ADCU",  key: "adcu",  detail: (_,  hz) => `${hz} Hz` },
  { label: "LIDAR",   key: "lidar", detail: (_,  hz) => `${hz} Hz` },
  { label: "RADAR",   key: "radar", detail: (_,  hz) => `${hz} Hz` },
  { label: "CAMERA",  key: "cam",   detail: (_,  hz) => `${hz} Hz` },
  { label: "V2X",     key: "v2x",   detail: (_,  hz) => `${hz} Hz` },
];

function buildHealth(diag, hz, gpsRtkLabel) {
  return HEALTH_ROWS.map((r) => {
    const code = (diag.status && diag.status[r.key]) || 0;
    const [st, c] = statusFromCode(code);
    const hzVal = (hz[r.key] || 0).toFixed(1);
    const ctx = r.key === "gps" ? { rtkText: `${gpsRtkLabel} · ${hzVal} Hz` } : {};
    const info = r.detail(ctx, hzVal);
    return [r.label, info, st, c];
  });
}

function buildSummary(diag) {
  const codes = HEALTH_ROWS.map((r) => (diag.status && diag.status[r.key]) || 0);
  // 4 dot summary: take first 4 health rows for indicator strip.
  return codes.slice(0, 4).map((code) => {
    if (code === 2) return T.red;
    if (code === 1) return T.amber;
    return T.green;
  });
}

// Convert ros object list → F1 Environment props.
// /hmi/objects payload: { count, data: [{id, x, y, vx, vy, type, width, length, ...}] }
function objectKindLabel(type) {
  if (type === "pedestrian") return { kind: "PED",   color: T.amber };
  if (type === "truck")      return { kind: "TRUCK", color: T.magenta };
  if (type === "bus")        return { kind: "BUS",   color: T.magenta };
  return { kind: "CAR", color: T.magenta };
}

function buildF1Objects(objects) {
  const list = (objects && objects.data) ? objects.data : [];
  // Visual cap: 12 closest objects to keep stage readable.
  return list.slice(0, 12).map((o) => {
    const fwd = o.x || 0;     // ego-frame forward (meters)
    const lat = o.y || 0;     // ego-frame lateral (meters, +left)
    const w = Math.max(12, (o.width  || 1.8) * 5.6);
    const h = Math.max(14, (o.length || 4.4) * 5.6);
    const dist = Math.hypot(fwd, lat).toFixed(1) + "m";
    const vmag = Math.hypot(o.vx || 0, o.vy || 0);
    const sign = (o.vx || 0) >= 0 ? "+" : "−";
    const spd = `${sign}${vmag.toFixed(1)}m/s`;
    const k = objectKindLabel(o.type);
    return {
      id: "#" + String(o.id ?? "?").padStart(3, "0"),
      kind: k.kind,
      color: k.color,
      forward_m: fwd,
      lateral_m: -lat,  // SVG x = +right, but ROS y = +left → invert.
      orientation: typeof o.orientation === "number" ? o.orientation : 0,  // rad, sensor frame
      w, h, dist, spd,
    };
  });
}

// ─── Time formatters ───────────────────────────────────────
function pad2(n) { return String(n).padStart(2, "0"); }
function pad3(n) { return String(n).padStart(3, "0"); }

function formatUtc(d) {
  return `${pad2(d.getUTCHours())}:${pad2(d.getUTCMinutes())}:${pad2(d.getUTCSeconds())}`;
}

function formatKst(d) {
  // KST = UTC+9
  const kst = new Date(d.getTime() + 9 * 3600 * 1000);
  return `${pad2(kst.getUTCHours())}:${pad2(kst.getUTCMinutes())}:${pad2(kst.getUTCSeconds())}`;
}

function formatTick(uptimeMs) {
  const sec = uptimeMs / 1000;
  return sec.toFixed(3) + "s";
}

function useClock(intervalMs = 250) {
  const [now, setNow] = React.useState(() => Date.now());
  React.useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), intervalMs);
    return () => clearInterval(id);
  }, [intervalMs]);
  return now;
}

// ─── Main screen ───────────────────────────────────────────
function F1HMIScreen() {
  const conn = useRosConnection();
  const state = useRosState();
  const diag = useDiagnostics();
  const hz = useTopicHz();
  const objects = useObjects();
  const popup = usePopup();
  const traffic = useTraffic();
  const bag = useBag();
  const map = useMap();

  const now = useClock(250);
  const pageLoadAtRef = React.useRef(Date.now());

  const rtk = formatRtk(state.gps && state.gps.rtk);
  const speedNum = (typeof state.speed === "number") ? state.speed : 0;
  const speedLimitNum = (typeof state.speed_limit === "number") ? state.speed_limit : 0;
  const trafficPhase = mapTrafficPhase(traffic.color);
  const trafficRemain = Math.round((traffic.time_decisec || 0) / 10);
  const yawDeg = radToDeg(state.ego && state.ego.yaw);
  const engaged = state.mode === 1;
  const oddNominal = state.on_odd === 0;
  const oddBanner = (popup && popup.severity && popup.severity !== "info" && popup.text) ? popup.text : null;
  const objsForStage = buildF1Objects(objects);
  const health = buildHealth(diag, hz, rtk.label);
  const summary = buildSummary(diag);

  // Top-bar fields
  const dNow = new Date(now);
  const utcText = formatUtc(dNow);
  const kstText = formatKst(dNow);
  const tickText = formatTick(now - pageLoadAtRef.current);
  const anyTopicOk = Object.values(hz).some(v => v > 0.5);
  const rosOk = conn.connected && anyTopicOk;
  const rosLabel = !conn.connected ? "ROS · OFFLINE"
    : anyTopicOk ? "ROS · ONLINE"
    : "ROS · WAITING";
  const netText = (conn.lastMessageAgeMs === Infinity || !conn.connected)
    ? "—"
    : `${Math.round(conn.lastMessageAgeMs)}ms`;

  // 04 LOCALIZATION
  const sigE = fmtSigmaCm(state.gps && state.gps.lon_std);
  const sigN = fmtSigmaCm(state.gps && state.gps.lat_std);
  const heading = (typeof yawDeg === "number")
    ? (((yawDeg % 360) + 360) % 360).toFixed(1) + "°" : "—";
  const laneId = state.lane_label || "—";
  const linkId = state.link_id ? String(state.link_id) : "—";

  // Bottom strip (PR-F1: 3 live + 5 placeholder)
  const dLim = (speedLimitNum > 0 ? (speedNum - speedLimitNum) : 0);
  const dLimStr = (speedLimitNum > 0
    ? (dLim >= 0 ? "+" : "−") + Math.abs(dLim).toFixed(1) + " km/h"
    : "—");
  const dLimColor = speedLimitNum > 0
    ? (dLim > 0 ? T.amber : T.green)
    : T.text3;
  const bottom = [
    ["EGO-VEL",  fmtKmh(speedNum), T.cyan],
    ["Δ-LIM",    dLimStr, dLimColor],
    ["LATERAL",  "—", T.text3],
    ["JERK",     "—", T.text3],
    ["LEAD-D",   "—", T.text3],
    ["LEAD-Δv",  "—", T.text3],
    ["PLAN-H",   "—", T.text3],
    ["CPU",      "—", T.text3],
  ];

  return (
    <F1HMIShell
      nodeName="web_hmi_bridge"
      utcText={utcText} kstText={kstText} tickText={tickText} netText={netText}
      rosOk={rosOk} rosLabel={rosLabel}
      speed={speedNum} speedLimit={speedLimitNum}
      steeringAngle={typeof state.steering === "number" ? state.steering : 0}
      gearLetter={GEAR_LABEL[state.gear] || "—"}
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
      objs={objsForStage} oddBanner={oddBanner}
      mapPolylines={(map && map.polylines) || []}
      egoEast={(state.ego && state.ego.east) || 0}
      egoNorth={(state.ego && state.ego.north) || 0}
      egoYaw={(state.ego && state.ego.yaw) || 0}
      bagRecording={!!(bag && bag.recording)}
      bagInfo={(bag && bag.info) || ""}
      onBagToggle={conn.publishBagToggle}
      bottom={bottom}
    />
  );
}

window.F1HMIScreen = F1HMIScreen;

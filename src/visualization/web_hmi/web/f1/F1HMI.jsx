/* global React */

// F1 KATECH Telemetry HMI shell. Pure presentational components — every
// visible value is a prop, no hardcoded telemetry. Mounted by F1HMIScreen.

const T = {
  bg0: "#06090c", bg1: "#0a0e13", bg2: "#0f1419", bg3: "#161c24",
  line: "#1f2730", lineHi: "#2a3441",
  text0: "#e6edf3", text1: "#c0c8d2", text2: "#7a8492", text3: "#4a5462",
  cyan: "#00e5ff", cyanDim: "#0095a8",
  amber: "#ffb547", red: "#ff3b3b", green: "#22e09a", magenta: "#ff5ea8",
};

const mono = "JetBrains Mono, ui-monospace, monospace";

// ─── Section header strip ─────────────────────────────────────
function Section({ num, title, right }) {
  return (
    <div style={{
      display: "flex", alignItems: "center", gap: 10,
      padding: "8px 14px",
      borderBottom: `1px solid ${T.line}`,
      background: T.bg1,
      fontFamily: mono, fontSize: 10, letterSpacing: 2,
    }}>
      <span style={{ color: T.cyan, fontWeight: 600 }}>{num}</span>
      <span style={{ color: T.text2 }}>{title}</span>
      <span style={{ marginLeft: "auto", color: T.text3 }}>{right}</span>
    </div>
  );
}

// ─── Half-circle speed gauge with numeric center ─────────────
function SpeedHalf({ value = 0, max = 120, limit = 0 }) {
  const W = 200, H = 130, cx = W / 2, cy = 110, r = 80;
  const t = Math.max(0, Math.min(1, value / max));
  const arc = (sa, ea) => {
    const sx = cx + r * Math.cos((sa * Math.PI) / 180);
    const sy = cy + r * Math.sin((sa * Math.PI) / 180);
    const ex = cx + r * Math.cos((ea * Math.PI) / 180);
    const ey = cy + r * Math.sin((ea * Math.PI) / 180);
    return `M ${sx} ${sy} A ${r} ${r} 0 0 1 ${ex} ${ey}`;
  };
  const ticks = [];
  for (let i = 0; i <= max; i += 10) {
    const a = 180 + (180 * i) / max;
    const major = i % 20 === 0;
    const inner = r - (major ? 12 : 6);
    const x1 = cx + inner * Math.cos((a * Math.PI) / 180);
    const y1 = cy + inner * Math.sin((a * Math.PI) / 180);
    const x2 = cx + r * Math.cos((a * Math.PI) / 180);
    const y2 = cy + r * Math.sin((a * Math.PI) / 180);
    const past = i / max <= t;
    ticks.push(<line key={i} x1={x1} y1={y1} x2={x2} y2={y2}
      stroke={past ? T.cyan : T.line} strokeWidth={major ? 1.4 : 0.8} />);
    if (major) {
      const tx = cx + (r - 22) * Math.cos((a * Math.PI) / 180);
      const ty = cy + (r - 22) * Math.sin((a * Math.PI) / 180) + 3;
      ticks.push(<text key={"l" + i} x={tx} y={ty} fontSize="8"
        fontFamily={mono} textAnchor="middle"
        fill={past ? T.cyan : T.text3}>{i}</text>);
    }
  }
  // limit marker (only when limit > 0)
  let limitMark = null;
  if (limit > 0 && limit <= max) {
    const la = 180 + (180 * limit) / max;
    const lx = cx + (r + 6) * Math.cos((la * Math.PI) / 180);
    const ly = cy + (r + 6) * Math.sin((la * Math.PI) / 180);
    limitMark = <circle cx={lx} cy={ly} r="3" fill={T.amber} />;
  }
  return (
    <svg width={W} height={H} viewBox={`0 0 ${W} ${H}`}>
      <path d={arc(180, 360)} stroke={T.line} strokeWidth="2" fill="none" />
      <path d={arc(180, 180 + 180 * t)} stroke={T.cyan} strokeWidth="2.5" fill="none"
        style={{ filter: "drop-shadow(0 0 5px rgba(0,229,255,0.5))" }} />
      {ticks}
      {limitMark}
      <text x={cx} y={92} textAnchor="middle" fontFamily={mono} fontSize="48" fontWeight="500" fill={T.text0} style={{ fontVariantNumeric: "tabular-nums" }}>{Math.round(value)}</text>
      <text x={cx} y={108} textAnchor="middle" fontFamily={mono} fontSize="9" fill={T.text2} letterSpacing="2">KM / H</text>
      <text x={cx} y={125} textAnchor="middle" fontFamily={mono} fontSize="8" fill={T.amber} letterSpacing="2">LIMIT {limit > 0 ? limit : '—'}</text>
    </svg>
  );
}

// ─── Steering wheel mini ────────────────────────────────────
function SteerDial({ angle = 0, gear = "—" }) {
  const W = 80, H = 88, cx = W / 2, cy = 40, r = 28;
  return (
    <svg width={W} height={H} viewBox={`0 0 ${W} ${H}`}>
      <circle cx={cx} cy={cy} r={r} stroke={T.line} strokeWidth="1.2" fill="none" />
      <circle cx={cx} cy={cy} r={r - 5} stroke={T.line} strokeWidth="0.7" fill="none" />
      <g transform={`rotate(${angle} ${cx} ${cy})`}>
        <line x1={cx} y1={cy - r + 3} x2={cx} y2={cy - r - 5} stroke={T.cyan} strokeWidth="1.6" />
        <circle cx={cx} cy={cy} r="2.4" fill={T.cyan} />
        <line x1={cx - r + 3} y1={cy} x2={cx + r - 3} y2={cy} stroke={T.cyan} strokeWidth="1" opacity="0.5" />
      </g>
      <text x={cx} y={82} textAnchor="middle" fontFamily={mono} fontSize="8" fill={T.text3} letterSpacing="2">GEAR <tspan fill={T.text0}>{gear}</tspan></text>
    </svg>
  );
}

// ─── Traffic light V2X ──────────────────────────────────────
function TrafficLight({ phase = "OFF", remain = 0 }) {
  const lit = { RED: "red", AMBER: "amber", GREEN: "green" }[phase];
  const phaseColor = { RED: T.red, AMBER: T.amber, GREEN: T.green, OFF: T.text3 }[phase] || T.text3;
  return (
    <div style={{ display: "flex", gap: 8, alignItems: "center" }}>
      <div style={{ width: 38, height: 90, background: T.bg0, border: `1px solid ${T.line}`, borderRadius: 5, padding: 4, display: "flex", flexDirection: "column", gap: 3, alignItems: "center" }}>
        {["red", "amber", "green"].map((c) => (
          <div key={c} style={{
            width: 24, height: 24, borderRadius: "50%",
            background: lit === c ? T[c] : T.bg2,
            border: `1px solid ${lit === c ? T[c] : T.line}`,
            boxShadow: lit === c ? `0 0 10px ${T[c]}, 0 0 3px ${T[c]} inset` : "none",
          }} />
        ))}
      </div>
      <div style={{ fontFamily: mono, fontSize: 9, color: T.text3, lineHeight: 1.5 }}>
        <div style={{ letterSpacing: 1 }}>PHASE</div>
        <div style={{ fontSize: 13, color: phaseColor, marginTop: 1 }}>{phase}</div>
        <div style={{ marginTop: 5, letterSpacing: 1 }}>CHANGE IN</div>
        <div style={{ fontSize: 18, color: T.text0, fontVariantNumeric: "tabular-nums", lineHeight: 1 }}>{remain}<span style={{ fontSize: 9, color: T.text3 }}> SEC</span></div>
      </div>
    </div>
  );
}

// ─── Top-down environment scene ─────────────────────────────
// objs: [{id, kind, color, x, y, w, h, dist, spd}] in SVG coordinate space.
// oddBanner: string | null — if set, shows amber warning strip near top.
// mapPolylines: [[[east,north], ...], ...] in EPSG:5179 absolute meters.
// egoEast/egoNorth/egoYaw: ego pose used to transform map into ego frame.
function Environment({
  objs = [], oddBanner = null,
  mapPolylines = [], egoEast = 0, egoNorth = 0, egoYaw = 0,
}) {
  const W = 880, H = 680;
  const ego = { x: W / 2, y: H * 0.62 };
  const ringRX = [120, 230, 340, 450, 560];
  const timeLabels = ["10s", "20s", "30s", "50s"];

  // Map ros_bridge ego-frame meters → SVG coordinate offset.
  // pxPerMeter: ringRX[4]=560 / 100m = 5.6 px/m
  const pxPerMeter = 5.6;
  const placedObjs = (objs || []).map((o) => ({
    ...o,
    sx: ego.x + (o.lateral_m || 0) * pxPerMeter,
    sy: ego.y - (o.forward_m || 0) * pxPerMeter,
  }));

  // Per-polyline AABB cached as long as mapPolylines reference is stable
  // (latched /hmi/map ⇒ ref only changes on a new map publish).
  const mapBboxes = React.useMemo(() => (mapPolylines || []).map((pl) => {
    let minE = Infinity, maxE = -Infinity, minN = Infinity, maxN = -Infinity;
    for (let i = 0; i < pl.length; i++) {
      const e = pl[i][0], n = pl[i][1];
      if (e < minE) minE = e;
      if (e > maxE) maxE = e;
      if (n < minN) minN = n;
      if (n > maxN) maxN = n;
    }
    return { minE, maxE, minN, maxN };
  }), [mapPolylines]);

  // Cull polylines outside ±150 m, transform survivors to ego-frame SVG.
  const mapPath = React.useMemo(() => {
    const pls = mapPolylines || [];
    if (!pls.length) return "";
    const cYaw = Math.cos(egoYaw), sYaw = Math.sin(egoYaw);
    const cull = 150; // metres
    const parts = [];
    for (let p = 0; p < pls.length; p++) {
      const bb = mapBboxes[p];
      if (!bb) continue;
      if (bb.maxE < egoEast - cull || bb.minE > egoEast + cull ||
          bb.maxN < egoNorth - cull || bb.minN > egoNorth + cull) continue;
      const pl = pls[p];
      for (let i = 0; i < pl.length; i++) {
        const dx = pl[i][0] - egoEast;
        const dy = pl[i][1] - egoNorth;
        const fwd = dx * cYaw + dy * sYaw;       // ego forward (+ahead)
        const left = -dx * sYaw + dy * cYaw;     // ego lateral (+left)
        const sx = (ego.x - left * pxPerMeter).toFixed(1);
        const sy = (ego.y - fwd  * pxPerMeter).toFixed(1);
        parts.push((i === 0 ? "M" : "L") + sx + " " + sy);
      }
    }
    return parts.join(" ");
  }, [mapPolylines, mapBboxes, egoEast, egoNorth, egoYaw]);

  return (
    <svg viewBox={`0 0 ${W} ${H}`} width="100%" height="100%" style={{ display: "block" }} preserveAspectRatio="xMidYMid meet">
      {/* grid */}
      <defs>
        <pattern id="grid" width="40" height="40" patternUnits="userSpaceOnUse">
          <path d="M 40 0 L 0 0 0 40" fill="none" stroke={T.line} strokeWidth="0.4" opacity="0.6" />
        </pattern>
      </defs>
      <rect width={W} height={H} fill="url(#grid)" />

      {/* HD-map polylines (A2_LINK, EPSG:5179 → ego-frame) */}
      {mapPath ? (
        <path d={mapPath} fill="none"
          stroke={T.cyan} strokeWidth="1.6" opacity="0.9"
          strokeLinejoin="round" strokeLinecap="round" />
      ) : null}

      {/* concentric distance rings */}
      {ringRX.map((rx, i) => (
        <ellipse key={i} cx={ego.x} cy={ego.y} rx={rx} ry={rx * 0.95} fill="none"
          stroke={T.lineHi} strokeWidth="0.6" strokeDasharray="2 4" opacity="0.55" />
      ))}

      {/* time-ahead labels */}
      {timeLabels.map((lbl, i) => {
        const y = ego.y - (i + 1) * 110;
        return (
          <g key={lbl}>
            <line x1={ego.x - 8} y1={y} x2={ego.x + 8} y2={y} stroke={T.line} strokeWidth="0.8" />
            <text x={ego.x + 14} y={y + 4} fontFamily={mono} fontSize="10" fill={T.text3}>{lbl}</text>
          </g>
        );
      })}

      {/* center cross */}
      <line x1={ego.x} y1={20} x2={ego.x} y2={H - 20} stroke={T.line} strokeWidth="0.6" strokeDasharray="2 4" />
      <line x1={20} y1={ego.y} x2={W - 20} y2={ego.y} stroke={T.line} strokeWidth="0.6" strokeDasharray="2 4" />

      {/* ODD warning banner */}
      {oddBanner ? (
        <g>
          <rect x={ego.x - 220} y={28} width={440} height={36} rx="3"
            fill="rgba(255,181,71,0.08)" stroke={T.amber} strokeWidth="1" />
          <text x={ego.x - 198} y={51} fontFamily={mono} fontSize="11" fill={T.amber} letterSpacing="1">⚠ WARN</text>
          <text x={ego.x - 150} y={51} fontFamily="Inter, Pretendard, sans-serif" fontSize="13" fill={T.text0}>{oddBanner}</text>
        </g>
      ) : null}

      {/* surrounding objects — body+chevron rotated by orientation,
          labels stay axis-aligned. orientation is rad in sensor frame
          (x=forward, y=+left). Screen rotation = -deg(orientation) since
          SVG y is flipped vs sensor y. */}
      {placedObjs.map((o, i) => {
        const rotDeg = -((o.orientation || 0) * 180 / Math.PI);
        const lblOff = Math.max(o.w, o.h) / 2 + 6; // clear rotated bbox
        return (
          <g key={o.id || i}>
            <g transform={`translate(${o.sx},${o.sy}) rotate(${rotDeg.toFixed(2)})`}>
              <rect x={-o.w / 2} y={-o.h / 2} width={o.w} height={o.h} rx="2"
                fill={`${o.color}14`} stroke={o.color} strokeWidth="1.3" />
              <path d={`M ${-3} ${-o.h / 2 + 2} L 0 ${-o.h / 2 - 4} L ${3} ${-o.h / 2 + 2}`}
                fill="none" stroke={o.color} strokeWidth="1.2"
                strokeLinecap="round" strokeLinejoin="round" />
            </g>
            <text x={o.sx + lblOff} y={o.sy - lblOff + 10}
              fontFamily={mono} fontSize="10" fill={o.color} letterSpacing="1">{o.id}  ·  {o.kind}</text>
            <text x={o.sx + lblOff} y={o.sy - lblOff + 24}
              fontFamily={mono} fontSize="9" fill={T.text2}>{o.dist}  {o.spd}</text>
          </g>
        );
      })}

      {/* EGO: IONIQ 5 — 4.635 m × 1.89 m, anchored at rear-axle center.
          Wheelbase 3.0 m + front overhang 0.845 m → 3.845 m ahead of anchor.
          Rear overhang 0.79 m → behind anchor. */}
      {(() => {
        const halfW   = 0.945 * pxPerMeter;   // 1.89 / 2
        const fwdLen  = 3.845 * pxPerMeter;   // wheelbase + front overhang
        const rearLen = 0.79  * pxPerMeter;   // rear overhang
        const frontY  = ego.y - fwdLen;
        const boxH    = fwdLen + rearLen;     // = 4.635 * pxPerMeter
        return (
          <g>
            <rect x={ego.x - halfW} y={frontY} width={halfW * 2} height={boxH} rx="1.5"
              fill={`${T.cyan}1A`} stroke={T.cyan} strokeWidth="1.4"
              style={{ filter: "drop-shadow(0 0 5px rgba(0,229,255,0.55))" }} />
            <path d={`M ${ego.x - 4} ${frontY - 1} L ${ego.x} ${frontY - 7} L ${ego.x + 4} ${frontY - 1}`}
              fill="none" stroke={T.cyan} strokeWidth="1.4" strokeLinecap="round" strokeLinejoin="round" />
            <circle cx={ego.x} cy={ego.y} r="1.6" fill={T.cyan} />
          </g>
        );
      })()}
    </svg>
  );
}

// ─── Helpers ────────────────────────────────────────────────
function Stat({ k, v }) {
  return (
    <div style={{ fontFamily: mono, display: "flex", alignItems: "baseline", gap: 6 }}>
      <span style={{ fontSize: 9, color: T.text3, letterSpacing: 2 }}>{k}</span>
      <span style={{ fontSize: 12, color: T.text0, fontVariantNumeric: "tabular-nums" }}>{v}</span>
    </div>
  );
}

function Dot({ c }) {
  return <span style={{ width: 7, height: 7, borderRadius: "50%", background: c, boxShadow: `0 0 6px ${c}`, display: "inline-block" }} />;
}

// ─── Main shell — receives all data via props ──────────────
function F1HMIShell(props) {
  const {
    // top bar
    nodeName = "web_hmi_bridge",
    utcText = "—",
    kstText = "—",
    tickText = "—",
    netText = "—",
    rosOk = false,
    rosLabel = "ROS · OFFLINE",
    // 01 velocity / steer
    speed = 0, speedLimit = 0, steeringAngle = 0, gearLetter = "—",
    thr = "—", brk = "—", accel = "—",
    // 02 drive mode
    engaged = false,
    vMaxText = "—",
    oddText = "—",
    // 03 traffic
    trafficPhase = "OFF",
    trafficRemain = 0,
    // 04 localization
    laneId = "—", linkId = "—",
    sigE = "—", sigN = "—", sigU = "—",
    hdop = "—", sats = "—", heading = "—",
    rtkLabel = "—", rtkColor = T.text3,
    // 05 system health
    health = [], summary = [T.text3, T.text3, T.text3, T.text3],
    // 06 environment
    objs = [], oddBanner = null,
    mapPolylines = [], egoEast = 0, egoNorth = 0, egoYaw = 0,
    bagRecording = false, bagInfo = "",
    onBagToggle = () => {},
    // bottom strip
    bottom = [],
    // Optional main-area override (e.g. Three.js scene). When provided,
    // replaces the SVG Environment but keeps the section header strip.
    mainContent = null,
  } = props;

  return (
    <div style={{
      width: 1600, height: 900,
      background: T.bg0, color: T.text0,
      fontFamily: "Inter, Pretendard, system-ui, sans-serif",
      display: "grid",
      gridTemplateColumns: "320px 1fr",
      gridTemplateRows: "44px 1fr 56px",
      gridTemplateAreas: `"top top" "left main" "left bottom"`,
      position: "relative",
    }}>
      {/* scanlines */}
      <div style={{ position: "absolute", inset: 0, pointerEvents: "none", zIndex: 100,
        backgroundImage: "repeating-linear-gradient(0deg, rgba(0,229,255,0.012) 0px, rgba(0,229,255,0.012) 1px, transparent 1px, transparent 3px)" }} />

      {/* TOP BAR */}
      <div style={{
        gridArea: "top",
        display: "grid",
        gridTemplateColumns: "auto 1fr auto auto auto auto auto",
        alignItems: "center", gap: 22,
        padding: "0 18px",
        background: T.bg1,
        borderBottom: `1px solid ${T.line}`,
      }}>
        <div style={{ display: "flex", alignItems: "center", gap: 10 }}>
          <div style={{ width: 22, height: 22, background: T.cyan, color: "#000",
            display: "grid", placeItems: "center", fontFamily: mono, fontWeight: 700, fontSize: 11,
            clipPath: "polygon(0 0, 100% 0, 100% 65%, 65% 100%, 0 100%)" }}>K</div>
          <div style={{ fontFamily: mono, fontSize: 12, letterSpacing: 3, color: T.text1 }}>
            KATECH · IONIQ 5
          </div>
          <div style={{ fontFamily: mono, fontSize: 9, color: T.text3, marginLeft: 6, letterSpacing: 1 }}>AD/HMI v3.2 · NODE {nodeName}</div>
        </div>
        <div />
        <Stat k="UTC"  v={utcText} />
        <Stat k="KST"  v={kstText} />
        <Stat k="TICK" v={tickText} />
        <Stat k="NET"  v={netText} />
        <div style={{
          padding: "5px 12px",
          border: `1px solid ${rosOk ? T.green : T.red}66`,
          background: `${rosOk ? T.green : T.red}10`,
          color: rosOk ? T.green : T.red,
          fontFamily: mono, fontSize: 10, letterSpacing: 2,
          display: "flex", alignItems: "center", gap: 8,
        }}>
          <span style={{ width: 7, height: 7, borderRadius: "50%",
            background: rosOk ? T.green : T.red,
            boxShadow: `0 0 6px ${rosOk ? T.green : T.red}` }} />
          {rosLabel}
        </div>
      </div>

      {/* LEFT PANEL */}
      <div style={{
        gridArea: "left",
        background: T.bg0,
        borderRight: `1px solid ${T.line}`,
        display: "flex", flexDirection: "column",
      }}>
        {/* 01 VELOCITY */}
        <Section num="01" title="VELOCITY · STEER" right="LIVE" />
        <div style={{ padding: "12px 14px", display: "grid", gridTemplateColumns: "1fr 1fr", gap: 10 }}>
          <div><SpeedHalf value={speed} max={120} limit={speedLimit} /></div>
          <div style={{ display: "flex", flexDirection: "column", height: 130 }}>
            <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
              <span style={{ fontFamily: mono, fontSize: 9, color: T.text3, letterSpacing: 1 }}>STEER</span>
              <span style={{ fontFamily: mono, fontSize: 14, color: T.text0, fontVariantNumeric: "tabular-nums" }}>{
                typeof steeringAngle === 'number' ? `${steeringAngle >= 0 ? '+' : '−'}${Math.abs(steeringAngle).toFixed(1)}°` : '—'
              }</span>
            </div>
            <div style={{ textAlign: "right", fontFamily: mono, fontSize: 8, color: T.text3, letterSpacing: 1, marginTop: 2 }}>{steeringAngle < 0 ? "← LEFT" : "RIGHT →"}</div>
            <div style={{ marginTop: "auto", display: "flex", justifyContent: "center" }}>
              <SteerDial angle={typeof steeringAngle === 'number' ? steeringAngle : 0} gear={gearLetter} />
            </div>
          </div>
        </div>
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: 1, background: T.line, borderTop: `1px solid ${T.line}`, borderBottom: `1px solid ${T.line}` }}>
          {[["THR", thr], ["BRK", brk], ["ACCEL", accel]].map(([k, v]) => (
            <div key={k} style={{ background: T.bg1, padding: "6px 12px", fontFamily: mono }}>
              <div style={{ fontSize: 9, color: T.text3, letterSpacing: 2 }}>{k}</div>
              <div style={{ fontSize: 14, color: T.text0, fontVariantNumeric: "tabular-nums", marginTop: 2 }}>{v}</div>
            </div>
          ))}
        </div>

        {/* 02 DRIVE MODE  + 03 V2X */}
        <div style={{ display: "grid", gridTemplateColumns: "1.4fr 1fr" }}>
          <div style={{ borderRight: `1px solid ${T.line}` }}>
            <Section num="02" title="DRIVE MODE" />
            <div style={{ padding: "12px 14px" }}>
              <div style={{ fontFamily: mono, fontSize: 9, color: T.text3, letterSpacing: 2 }}>DRIVE MODE</div>
              <div style={{ display: "flex", marginTop: 8, border: `1px solid ${T.line}` }}>
                <div style={{
                  flex: 1, padding: "10px 0", textAlign: "center",
                  fontFamily: mono, fontSize: 13, letterSpacing: 1,
                  color: !engaged ? T.bg0 : T.text2,
                  background: !engaged ? T.cyan : "transparent",
                  fontWeight: !engaged ? 600 : 400,
                }}>MANUAL</div>
                <div style={{
                  flex: 1, padding: "10px 0", textAlign: "center",
                  fontFamily: mono, fontSize: 13, letterSpacing: 1,
                  color: engaged ? T.bg0 : T.text2,
                  background: engaged ? T.cyan : "transparent",
                  fontWeight: engaged ? 600 : 400,
                }}>AUTONOMOUS</div>
              </div>
              <div style={{ marginTop: 12, fontFamily: mono, fontSize: 10, letterSpacing: 1, display: "flex", flexDirection: "column", gap: 4 }}>
                <div style={{ color: engaged ? T.green : T.amber }}>● {engaged ? 'ENGAGED' : 'STANDBY'}</div>
                <div style={{ color: T.cyan }}>● v_max {vMaxText}</div>
                <div style={{ color: oddText === "nominal" ? T.green : T.amber }}>● ODD {oddText}</div>
              </div>
            </div>
          </div>
          <div style={{ display: "flex", flexDirection: "column" }}>
            <Section num="03" title="V2X" />
            <div style={{ padding: "12px 14px", flex: 1, display: "flex", alignItems: "center", justifyContent: "flex-end" }}>
              <TrafficLight phase={trafficPhase} remain={trafficRemain} />
            </div>
          </div>
        </div>

        {/* 04 LOCALIZATION */}
        <Section num="04" title="LOCALIZATION" right={<span style={{ color: rtkColor }}>{rtkLabel}</span>} />
        <div style={{ padding: "10px 14px", fontFamily: mono, fontSize: 11, display: "grid", gridTemplateColumns: "auto 1fr auto 1fr", rowGap: 6, columnGap: 10 }}>
          {[
            ["LANE ID", laneId, "LINK", linkId],
            ["σ-EAST", sigE, "σ-NORTH", sigN],
            ["σ-UP",   sigU, "HDOP",    hdop],
            ["SATS",   sats, "HEADING", heading],
          ].map((row, i) => (
            <React.Fragment key={i}>
              <span style={{ color: T.text3, letterSpacing: 1 }}>{row[0]}</span>
              <span style={{ color: T.text0, fontVariantNumeric: "tabular-nums" }}>{row[1]}</span>
              <span style={{ color: T.text3, letterSpacing: 1 }}>{row[2]}</span>
              <span style={{ color: T.text0, fontVariantNumeric: "tabular-nums" }}>{row[3]}</span>
            </React.Fragment>
          ))}
        </div>

        {/* 05 SYSTEM HEALTH */}
        <Section num="05" title="SYSTEM HEALTH" right={
          <span style={{ display: "flex", gap: 8, alignItems: "center" }}>
            {summary.map((c, i) => <Dot key={i} c={c} />)}
          </span>
        } />
        <div style={{ flex: 1, overflow: "hidden" }}>
          {health.map(([n, info, st, c]) => (
            <div key={n} style={{
              display: "grid", gridTemplateColumns: "12px 80px 1fr auto",
              gap: 10, alignItems: "center",
              padding: "8px 14px",
              borderBottom: `1px solid ${T.line}`,
              fontFamily: mono, fontSize: 11,
            }}>
              <Dot c={c} />
              <span style={{ color: T.text1, letterSpacing: 1 }}>{n}</span>
              <span style={{ color: T.text3 }}>{info}</span>
              <span style={{ color: c }}>{st}</span>
            </div>
          ))}
        </div>
      </div>

      {/* MAIN STAGE */}
      <div style={{ gridArea: "main", position: "relative", background: T.bg0 }}>
        <div style={{ display: "flex", alignItems: "center", padding: "8px 18px",
          fontFamily: mono, fontSize: 10, letterSpacing: 2, color: T.text2,
          borderBottom: `1px solid ${T.line}`, background: T.bg1, gap: 10 }}>
          <span style={{ color: T.cyan }}>06</span>
          <span>ENVIRONMENT · TOP-DOWN</span>
          <span style={{ marginLeft: "auto", color: T.text3, display: "flex", gap: 16 }}>
            <span>SCALE 1:8</span><span>GRID 6</span>
            <span style={{ color: objs.length > 0 ? T.green : T.text3 }}>● {objs.length > 0 ? 'TRACK' : 'IDLE'}</span>
          </span>
        </div>
        {bagRecording ? (
          <div style={{ position: "absolute", top: 36, right: 18, zIndex: 5,
            width: 220, padding: 10,
            border: `1px solid ${T.red}66`, background: `${T.red}10`,
            fontFamily: mono, fontSize: 11 }}>
            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <span style={{ width: 8, height: 8, borderRadius: "50%", background: T.red, boxShadow: `0 0 8px ${T.red}` }} />
              <span style={{ color: T.red, letterSpacing: 2 }}>REC</span>
              <span style={{ marginLeft: "auto", color: T.text2, letterSpacing: 1 }}>ROSBAG</span>
            </div>
            <div style={{ marginTop: 8, color: T.text1, fontSize: 10, wordBreak: "break-all" }}>
              {bagInfo || "—"}
            </div>
            <div onClick={onBagToggle} style={{
              marginTop: 8, padding: "6px 10px",
              background: T.red, color: T.text0,
              textAlign: "center", letterSpacing: 3, fontWeight: 700,
              cursor: "pointer", userSelect: "none",
            }}>STOP</div>
          </div>
        ) : (
          <div onClick={onBagToggle} style={{ position: "absolute", top: 36, right: 18, zIndex: 5,
            width: 220, padding: 10,
            border: `1px solid ${T.line}`, background: T.bg1,
            fontFamily: mono, fontSize: 11, cursor: "pointer", userSelect: "none" }}>
            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <span style={{ width: 8, height: 8, borderRadius: "50%", background: T.text3 }} />
              <span style={{ color: T.text3, letterSpacing: 2 }}>IDLE</span>
              <span style={{ marginLeft: "auto", color: T.text3, letterSpacing: 1 }}>ROSBAG</span>
            </div>
            <div style={{ marginTop: 8, padding: "6px 10px", background: T.bg2, color: T.text1,
              textAlign: "center", letterSpacing: 3, fontWeight: 700 }}>START</div>
          </div>
        )}

        <div style={{ position: "absolute", inset: "36px 0 0 0" }}>
          {mainContent ? mainContent : (
            <Environment
              objs={objs}
              oddBanner={oddBanner}
              mapPolylines={mapPolylines}
              egoEast={egoEast}
              egoNorth={egoNorth}
              egoYaw={egoYaw}
            />
          )}
        </div>
      </div>

      {/* BOTTOM TELEMETRY STRIP */}
      <div style={{
        gridArea: "bottom",
        gridColumn: "2 / 3",
        background: T.bg1,
        borderTop: `1px solid ${T.line}`,
        display: "grid",
        gridTemplateColumns: "repeat(8, 1fr)",
      }}>
        {bottom.map(([k, v, c], i) => (
          <div key={k + i} style={{
            padding: "10px 14px",
            borderRight: i < bottom.length - 1 ? `1px solid ${T.line}` : "none",
            fontFamily: mono,
          }}>
            <div style={{ fontSize: 9, color: T.text3, letterSpacing: 2 }}>{k}</div>
            <div style={{ fontSize: 16, color: c || T.text0, marginTop: 4, fontVariantNumeric: "tabular-nums" }}>{v}</div>
          </div>
        ))}
      </div>
    </div>
  );
}

window.F1HMIShell = F1HMIShell;
window.F1Tokens = T;

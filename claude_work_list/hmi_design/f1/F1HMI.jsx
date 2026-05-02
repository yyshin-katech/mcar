/* global React */

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
function SpeedHalf({ value = 47, max = 120, limit = 50 }) {
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
  // limit marker
  const la = 180 + (180 * limit) / max;
  const lx = cx + (r + 6) * Math.cos((la * Math.PI) / 180);
  const ly = cy + (r + 6) * Math.sin((la * Math.PI) / 180);
  return (
    <svg width={W} height={H} viewBox={`0 0 ${W} ${H}`}>
      <path d={arc(180, 360)} stroke={T.line} strokeWidth="2" fill="none" />
      <path d={arc(180, 180 + 180 * t)} stroke={T.cyan} strokeWidth="2.5" fill="none"
        style={{ filter: "drop-shadow(0 0 5px rgba(0,229,255,0.5))" }} />
      {ticks}
      <circle cx={lx} cy={ly} r="3" fill={T.amber} />
      <text x={cx} y={92} textAnchor="middle" fontFamily={mono} fontSize="48" fontWeight="500" fill={T.text0} style={{ fontVariantNumeric: "tabular-nums" }}>{value}</text>
      <text x={cx} y={108} textAnchor="middle" fontFamily={mono} fontSize="9" fill={T.text2} letterSpacing="2">KM / H</text>
      <text x={cx} y={125} textAnchor="middle" fontFamily={mono} fontSize="8" fill={T.amber} letterSpacing="2">LIMIT {limit}</text>
    </svg>
  );
}

// ─── Steering wheel mini ────────────────────────────────────
function SteerDial({ angle = -12.4 }) {
  const W = 110, H = 130, cx = W / 2, cy = 70, r = 38;
  return (
    <svg width={W} height={H} viewBox={`0 0 ${W} ${H}`}>
      <circle cx={cx} cy={cy} r={r} stroke={T.line} strokeWidth="1.5" fill="none" />
      <circle cx={cx} cy={cy} r={r - 6} stroke={T.line} strokeWidth="0.8" fill="none" />
      <g transform={`rotate(${angle} ${cx} ${cy})`}>
        <line x1={cx} y1={cy - r + 4} x2={cx} y2={cy - r - 6} stroke={T.cyan} strokeWidth="2" />
        <circle cx={cx} cy={cy} r="3" fill={T.cyan} />
        <line x1={cx - r + 4} y1={cy} x2={cx + r - 4} y2={cy} stroke={T.cyan} strokeWidth="1.2" opacity="0.5" />
      </g>
      <text x={cx} y={120} textAnchor="middle" fontFamily={mono} fontSize="9" fill={T.text3} letterSpacing="2">GEAR <tspan fill={T.text0}>D</tspan></text>
    </svg>
  );
}

// ─── Traffic light V2X ──────────────────────────────────────
function TrafficLight({ phase = "RED", remain = 18 }) {
  const lit = { RED: "red", AMBER: "amber", GREEN: "green" }[phase];
  return (
    <div style={{ display: "flex", gap: 14, alignItems: "center" }}>
      <div style={{ width: 56, height: 130, background: T.bg0, border: `1px solid ${T.line}`, borderRadius: 6, padding: 6, display: "flex", flexDirection: "column", gap: 4, alignItems: "center" }}>
        {["red", "amber", "green"].map((c) => (
          <div key={c} style={{
            width: 36, height: 36, borderRadius: "50%",
            background: lit === c ? T[c] : T.bg2,
            border: `1px solid ${lit === c ? T[c] : T.line}`,
            boxShadow: lit === c ? `0 0 14px ${T[c]}, 0 0 4px ${T[c]} inset` : "none",
          }} />
        ))}
      </div>
      <div style={{ fontFamily: mono, fontSize: 10, color: T.text3, lineHeight: 1.6 }}>
        <div style={{ letterSpacing: 2 }}>PHASE</div>
        <div style={{ fontSize: 18, color: T.red, marginTop: 2 }}>{phase}</div>
        <div style={{ marginTop: 8, letterSpacing: 2 }}>CHANGE IN</div>
        <div style={{ fontSize: 28, color: T.text0, fontVariantNumeric: "tabular-nums", lineHeight: 1 }}>{remain}<span style={{ fontSize: 10, color: T.text3 }}> SEC</span></div>
      </div>
    </div>
  );
}

// ─── Top-down environment scene ─────────────────────────────
function Environment() {
  // ego is a glyph at center-bottom area; lateral distance markers as concentric ovals
  // surrounding boxes for vehicles/peds
  const W = 880, H = 680;
  const ego = { x: W / 2, y: H * 0.62 };
  const objs = [
    { id: "#203", k: "TRUCK", c: T.magenta, x: ego.x + 70, y: ego.y - 380, w: 36, h: 80, dist: "42.5m", spd: "+4.0m/s" },
    { id: "#156", k: "CAR",   c: T.magenta, x: ego.x + 30, y: ego.y - 240, w: 32, h: 60, dist: "20.4m", spd: "-2.1m/s" },
    { id: "#142", k: "CAR",   c: T.magenta, x: ego.x - 14, y: ego.y - 130, w: 28, h: 56, dist: "14.3m", spd: "+0.8m/s" },
    { id: "#091", k: "PED",   c: T.amber,   x: ego.x + 60, y: ego.y - 130, w: 14, h: 14, dist: "16.8m", spd: "+0.0m/s" },
    { id: "#088", k: "PED",   c: T.amber,   x: ego.x - 80, y: ego.y - 60,  w: 14, h: 14, dist: "11.1m", spd: "+0.0m/s" },
    { id: "#311", k: "CAR",   c: T.magenta, x: ego.x - 80, y: ego.y + 60,  w: 28, h: 56, dist:  "7.4m", spd: "+0.0m/s" },
  ];
  const ringRX = [120, 230, 340, 450, 560];
  const timeLabels = ["10s", "20s", "30s", "50s"];

  return (
    <svg viewBox={`0 0 ${W} ${H}`} width="100%" height="100%" style={{ display: "block" }} preserveAspectRatio="xMidYMid meet">
      {/* grid */}
      <defs>
        <pattern id="grid" width="40" height="40" patternUnits="userSpaceOnUse">
          <path d="M 40 0 L 0 0 0 40" fill="none" stroke={T.line} strokeWidth="0.4" opacity="0.6" />
        </pattern>
      </defs>
      <rect width={W} height={H} fill="url(#grid)" />

      {/* concentric distance rings (ellipses centered on ego) */}
      {ringRX.map((rx, i) => (
        <ellipse key={i} cx={ego.x} cy={ego.y} rx={rx} ry={rx * 0.95} fill="none"
          stroke={T.lineHi} strokeWidth="0.6" strokeDasharray="2 4" opacity="0.55" />
      ))}

      {/* vertical "time ahead" axis with labels on right */}
      {timeLabels.map((lbl, i) => {
        const y = ego.y - (i + 1) * 110;
        return (
          <g key={lbl}>
            <line x1={ego.x - 8} y1={y} x2={ego.x + 8} y2={y} stroke={T.line} strokeWidth="0.8" />
            <text x={ego.x + 14} y={y + 4} fontFamily={mono} fontSize="10" fill={T.text3}>{lbl}</text>
          </g>
        );
      })}

      {/* center-line cross */}
      <line x1={ego.x} y1={20} x2={ego.x} y2={H - 20} stroke={T.line} strokeWidth="0.6" strokeDasharray="2 4" />
      <line x1={20} y1={ego.y} x2={W - 20} y2={ego.y} stroke={T.line} strokeWidth="0.6" strokeDasharray="2 4" />

      {/* planned path — pink line through ego, forward + a bit back */}
      <path d={`M ${ego.x} ${ego.y - 470} L ${ego.x} ${ego.y + 40}`} stroke={T.magenta} strokeWidth="3"
        strokeLinecap="round" style={{ filter: "drop-shadow(0 0 5px rgba(255,94,168,0.7))" }} opacity="0.85" />

      {/* ODD warning banner */}
      <g>
        <rect x={ego.x - 200} y={28} width={400} height={36} rx="3"
          fill="rgba(255,181,71,0.08)" stroke={T.amber} strokeWidth="1" />
        <text x={ego.x - 178} y={51} fontFamily={mono} fontSize="11" fill={T.amber} letterSpacing="1">⚠ WARN</text>
        <text x={ego.x - 130} y={51} fontFamily="Inter, Pretendard, sans-serif" fontSize="13" fill={T.text0}>ODD 이탈 경고 — 전방 200m 비포장 구간 감지</text>
      </g>

      {/* surrounding objects */}
      {objs.map((o) => (
        <g key={o.id}>
          <rect x={o.x - o.w / 2} y={o.y - o.h / 2} width={o.w} height={o.h} rx="2"
            fill={`${o.c}14`} stroke={o.c} strokeWidth="1.3" />
          <text x={o.x + o.w / 2 + 6} y={o.y - o.h / 2 + 8}
            fontFamily={mono} fontSize="10" fill={o.c} letterSpacing="1">{o.id}  ·  {o.k}</text>
          <text x={o.x + o.w / 2 + 6} y={o.y - o.h / 2 + 22}
            fontFamily={mono} fontSize="9" fill={T.text2}>{o.dist}  {o.spd}</text>
        </g>
      ))}

      {/* EGO car — cyan rounded box with chevron */}
      <g>
        <rect x={ego.x - 12} y={ego.y - 24} width={24} height={48} rx="4"
          fill={`${T.cyan}1A`} stroke={T.cyan} strokeWidth="1.6"
          style={{ filter: "drop-shadow(0 0 6px rgba(0,229,255,0.5))" }} />
        <path d={`M ${ego.x - 5} ${ego.y - 16} L ${ego.x} ${ego.y - 22} L ${ego.x + 5} ${ego.y - 16}`}
          fill="none" stroke={T.cyan} strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" />
      </g>
    </svg>
  );
}

// ─── Main app ──────────────────────────────────────────────
function HMI() {
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
          <div style={{ fontFamily: mono, fontSize: 9, color: T.text3, marginLeft: 6, letterSpacing: 1 }}>AD/HMI v3.2 · NODE pyqt_display</div>
        </div>
        <div />
        <Stat k="UTC"  v="05:32:14" />
        <Stat k="KST"  v="14:32:14" />
        <Stat k="TICK" v="0142.387s" />
        <Stat k="NET"  v="12ms" />
        <div style={{
          padding: "5px 12px", border: `1px solid ${T.green}66`,
          background: `${T.green}10`, color: T.green,
          fontFamily: mono, fontSize: 10, letterSpacing: 2,
          display: "flex", alignItems: "center", gap: 8,
        }}>
          <span style={{ width: 7, height: 7, borderRadius: "50%", background: T.green, boxShadow: `0 0 6px ${T.green}` }} />
          ROS · /ad_can OK
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
          <div><SpeedHalf value={47} max={120} limit={50} /></div>
          <div style={{ position: "relative" }}>
            <div style={{ position: "absolute", top: 6, right: 4, fontFamily: mono, fontSize: 10, color: T.text3, letterSpacing: 1, textAlign: "right" }}>STEER</div>
            <div style={{ position: "absolute", top: 22, right: 4, fontFamily: mono, fontSize: 22, color: T.text0, fontVariantNumeric: "tabular-nums", textAlign: "right" }}>−12.4°</div>
            <div style={{ position: "absolute", top: 50, right: 4, fontFamily: mono, fontSize: 9, color: T.text3, letterSpacing: 1, textAlign: "right" }}>← LEFT</div>
            <SteerDial angle={-12.4} />
          </div>
        </div>
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: 1, background: T.line, borderTop: `1px solid ${T.line}`, borderBottom: `1px solid ${T.line}` }}>
          {[["THR", "38%"], ["BRK", "00%"], ["ACCEL", "0.42 g"]].map(([k, v]) => (
            <div key={k} style={{ background: T.bg1, padding: "6px 12px", fontFamily: mono }}>
              <div style={{ fontSize: 9, color: T.text3, letterSpacing: 2 }}>{k}</div>
              <div style={{ fontSize: 14, color: T.text0, fontVariantNumeric: "tabular-nums", marginTop: 2 }}>{v}</div>
            </div>
          ))}
        </div>

        {/* 02 DRIVE MODE  + 03 V2X */}
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr" }}>
          <div style={{ borderRight: `1px solid ${T.line}` }}>
            <Section num="02" title="DRIVE MODE" />
            <div style={{ padding: "12px 14px" }}>
              <div style={{ fontFamily: mono, fontSize: 9, color: T.text3, letterSpacing: 2 }}>DRIVE MODE</div>
              <div style={{ display: "flex", marginTop: 8, border: `1px solid ${T.line}` }}>
                <div style={{ flex: 1, padding: "6px 0", textAlign: "center", fontFamily: mono, fontSize: 11, color: T.text2, letterSpacing: 1 }}>MANUAL</div>
                <div style={{ flex: 1, padding: "6px 0", textAlign: "center", fontFamily: mono, fontSize: 11, color: T.bg0, background: T.cyan, letterSpacing: 1, fontWeight: 600 }}>AUTONOMOUS</div>
              </div>
              <div style={{ marginTop: 12, fontFamily: mono, fontSize: 10, color: T.text3, letterSpacing: 1 }}>
                <span style={{ color: T.green }}>● ENGAGED</span> &nbsp; v_max 50 &nbsp; ODD
                <div style={{ color: T.text2, marginTop: 4 }}>nominal</div>
              </div>
            </div>
          </div>
          <div>
            <Section num="03" title="V2X SIGNAL" />
            <div style={{ padding: "12px 14px" }}>
              <TrafficLight phase="RED" remain={18} />
            </div>
          </div>
        </div>

        {/* 04 LOCALIZATION */}
        <Section num="04" title="LOCALIZATION" right={<span style={{ color: T.green }}>RTK FIX</span>} />
        <div style={{ padding: "10px 14px", fontFamily: mono, fontSize: 11, display: "grid", gridTemplateColumns: "auto 1fr auto 1fr", rowGap: 6, columnGap: 10 }}>
          {[
            ["LANE ID", "A2_L0142", "LINK", "EXP-44.2"],
            ["σ-EAST", "01.84 cm", "σ-NORTH", "02.10 cm"],
            ["σ-UP",   "04.20 cm", "HDOP",    "0.6"],
            ["SATS",   "32 / 36",  "HEADING", "142.7°"],
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
            <Dot c={T.green} /><Dot c={T.green} /><Dot c={T.amber} /><Dot c={T.green} />
          </span>
        } />
        <div style={{ flex: 1, overflow: "hidden" }}>
          {[
            ["GPS-RTK", "FIX · 2.1cm", "OK", T.green],
            ["K-ADCU",  "SWC nominal", "OK", T.green],
            ["LIDAR",   "3 units · 12.0 Hz", "OK", T.green],
            ["RADAR",   "long+corner", "OK", T.green],
            ["CAMERA",  "6× 30 fps", "OK", T.green],
            ["V2X",     "spat degraded", "WARN", T.amber],
          ].map(([n, info, st, c]) => (
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
            <span style={{ color: T.green }}>● TRACK</span>
          </span>
        </div>
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
            ~/bag_data/2026-04-29_14-32-10
          </div>
          <div style={{
            marginTop: 8, padding: "6px 10px",
            background: T.red, color: T.text0,
            textAlign: "center", letterSpacing: 3, fontWeight: 700,
            cursor: "pointer",
          }}>STOP</div>
          <div style={{ marginTop: 6, display: "flex", justifyContent: "space-between", color: T.text3, fontSize: 10 }}>
            <span>00:04:21</span><span>1.2 GB</span>
          </div>
        </div>

        <div style={{ position: "absolute", inset: "36px 0 0 0" }}>
          <Environment />
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
        {[
          ["EGO-VEL", "47.2 km/h", T.cyan],
          ["Δ-LIM",   "−2.8 km/h", T.amber],
          ["LATERAL", "+0.12 m",   T.text0],
          ["JERK",    "0.04 g/s",  T.text0],
          ["LEAD-D",  "14.3 m",    T.magenta],
          ["LEAD-Δv", "+8.2 m/s",  T.green],
          ["PLAN-H",  "60 m",      T.text0],
          ["CPU",     "34% / 41°C", T.text0],
        ].map(([k, v, c], i) => (
          <div key={k} style={{
            padding: "10px 14px",
            borderRight: i < 7 ? `1px solid ${T.line}` : "none",
            fontFamily: mono,
          }}>
            <div style={{ fontSize: 9, color: T.text3, letterSpacing: 2 }}>{k}</div>
            <div style={{ fontSize: 16, color: c, marginTop: 4, fontVariantNumeric: "tabular-nums" }}>{v}</div>
          </div>
        ))}
      </div>
    </div>
  );
}

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

window.HMI = HMI;

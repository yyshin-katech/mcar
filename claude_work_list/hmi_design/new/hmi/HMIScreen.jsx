/* global React, SpeedGauge, Vehicle, VehicleTop, DetectionOverlay, DetectionOverlayTop, RangeRings, RangeRingsTop, TrafficObjects, PlanPath */

function HMIScreen({
  speed = 47,
  driveMode = "D",
  bagRecording = true,
  vehicleStyle = "solid",       // "solid" | "wireframe"
  topView = false,              // top-down (true) vs isometric (false)
  accent = "amber",             // for canvas variations
  layout = "balanced",          // "balanced" | "left-heavy" | "bottom-heavy" | "split"
  typeface = "sans",            // "sans" | "mono"
}) {
  const accentColors = {
    amber:  { c0: "#FFB547", c1: "#F59E2C", c2: "#C97A14", glow: "rgba(255,181,71,0.18)" },
    cyan:   { c0: "#5EEAD4", c1: "#2DD4BF", c2: "#0F766E", glow: "rgba(94,234,212,0.18)" },
    violet: { c0: "#C4B5FD", c1: "#A78BFA", c2: "#7C3AED", glow: "rgba(196,181,253,0.18)" },
    lime:   { c0: "#D9F99D", c1: "#BEF264", c2: "#65A30D", glow: "rgba(217,249,157,0.18)" },
  }[accent] || { c0: "#FFB547", c1: "#F59E2C", c2: "#C97A14", glow: "rgba(255,181,71,0.18)" };

  const styleVars = {
    "--amber-0": accentColors.c0,
    "--amber-1": accentColors.c1,
    "--amber-2": accentColors.c2,
    "--amber-glow": accentColors.glow,
    "--font-display": typeface === "mono"
      ? '"JetBrains Mono", ui-monospace, monospace'
      : '"Inter", "Pretendard", sans-serif',
  };

  return (
    <div className="hmi" style={styleVars}>
      {/* ---------- Topbar ---------- */}
      <div className="topbar">
        <div className="topbar-left">
          <div className="brand">
            <div className="brand-mark">i5</div>
            <span>IONIQ 5 · HMI</span>
          </div>
          <div className="crumb">SESSION <b>// SEOUL-NORTH-04</b></div>
        </div>
        <div className="topbar-center">
          <div className="system-tag"><span className="dot" />ROS 1 NOETIC · ONLINE</div>
          <div className="system-tag"><span className="dot amber" />AUTONOMY L2+</div>
          <div className="system-tag"><span className="dot" />CAN BUS · 500 KBPS</div>
        </div>
        <div className="topbar-right">
          <div className="crumb">CPU <b>32%</b> · MEM <b>54%</b></div>
          <div className="clock">14:32:08 KST</div>
        </div>
      </div>

      {/* ---------- Left Panel ---------- */}
      <div className="left">
        <div>
          <div className="section-label">Vehicle State</div>
          <SpeedGauge speed={speed} max={120} mode="ECO" />
        </div>

        <div>
          <div className="section-label">Drive Mode</div>
          <div className="drive-row">
            {["P", "R", "N", "D"].map((m) => (
              <div key={m} className={`drive-cell ${m === driveMode ? "active" : ""}`}>{m}</div>
            ))}
          </div>
        </div>

        <div>
          <div className="section-label">Powertrain</div>
          <div className="stat-grid">
            <div className="stat">
              <div className="stat-k">Battery</div>
              <div className="stat-v">78<small>%</small></div>
            </div>
            <div className="stat">
              <div className="stat-k">Range</div>
              <div className="stat-v">312<small>km</small></div>
            </div>
            <div className="stat">
              <div className="stat-k">Motor</div>
              <div className="stat-v">42<small>°C</small></div>
            </div>
            <div className="stat">
              <div className="stat-k">Pack V</div>
              <div className="stat-v">682<small>V</small></div>
            </div>
          </div>
        </div>

        <div style={{ marginTop: "auto" }}>
          <div className="section-label">Heading · GNSS</div>
          <div className="stat-grid">
            <div className="stat">
              <div className="stat-k">Lat</div>
              <div className="stat-v" style={{ fontSize: 14, fontFamily: "var(--font-mono)" }}>37.5642°N</div>
            </div>
            <div className="stat">
              <div className="stat-k">Lon</div>
              <div className="stat-v" style={{ fontSize: 14, fontFamily: "var(--font-mono)" }}>127.0016°E</div>
            </div>
          </div>
        </div>
      </div>

      {/* ---------- Center Stage ---------- */}
      <div className="stage">
        {!topView && <div className="iso-grid" />}
        {topView ? <RangeRingsTop /> : <RangeRings />}
        {topView ? <DetectionOverlayTop /> : <DetectionOverlay />}
        {topView && <PlanPath />}
        {topView && <TrafficObjects />}

        <div className="stage-corner tl">FRONT · LIDAR + CAM</div>
        <div className="compass">N</div>
        <div className="stage-corner bl">SCALE · 1 : 200</div>
        <div className="stage-corner br">PROJ · UTM 52N</div>

        {/* Bag rec banner */}
        <div className={`bag-banner ${bagRecording ? "recording" : ""}`}>
          <div className="rec-dot" />
          <div className="bag-text">
            <div className="bag-title">{bagRecording ? "REC · ROSBAG" : "BAG · IDLE"}</div>
            <div className="bag-name">drive_2026-05-01_14-21-37.bag</div>
          </div>
          <div className="bag-time">00:10:31</div>
        </div>

        <div className="vehicle" style={topView ? { transform: "none", filter: "none", zIndex: 6 } : undefined}>
          {topView
            ? <VehicleTop wireframe={vehicleStyle === "wireframe"} accent={accentColors.c0} />
            : <Vehicle wireframe={vehicleStyle === "wireframe"} />}
        </div>
      </div>

      {/* ---------- Right Panel ---------- */}
      <div className="right">
        <div className="module">
          <div className="section-label">Mission · Objectives</div>
          <div className="objective-list">
            <div className="objective done">
              <div className="obj-mark" />
              <div className="obj-text">Calibrate IMU + GPS sync</div>
              <div className="obj-meta">14:18</div>
            </div>
            <div className="objective done">
              <div className="obj-mark" />
              <div className="obj-text">Depart from depot</div>
              <div className="obj-meta">14:22</div>
            </div>
            <div className="objective active">
              <div className="obj-mark" />
              <div className="obj-text">Loop · north corridor</div>
              <div className="obj-meta">2.1 km</div>
            </div>
            <div className="objective">
              <div className="obj-mark" />
              <div className="obj-text">Return waypoint α-04</div>
              <div className="obj-meta">—</div>
            </div>
            <div className="objective">
              <div className="obj-mark" />
              <div className="obj-text">Stop & finalize bag</div>
              <div className="obj-meta">—</div>
            </div>
          </div>
        </div>

        <div className="module">
          <div className="section-label">Sensors</div>
          <div>
            <div className="sensor">
              <div className="sensor-name">/velodyne_points</div>
              <div className="sensor-rate">10.0 Hz</div>
              <div className="sensor-status"><span className="dot" />OK</div>
            </div>
            <div className="sensor">
              <div className="sensor-name">/camera/front/image</div>
              <div className="sensor-rate">30.0 Hz</div>
              <div className="sensor-status"><span className="dot" />OK</div>
            </div>
            <div className="sensor">
              <div className="sensor-name">/imu/data</div>
              <div className="sensor-rate">100.0 Hz</div>
              <div className="sensor-status"><span className="dot" />OK</div>
            </div>
            <div className="sensor">
              <div className="sensor-name">/gnss/fix</div>
              <div className="sensor-rate">10.0 Hz</div>
              <div className="sensor-status"><span className="dot" />OK</div>
            </div>
            <div className="sensor">
              <div className="sensor-name">/can/vehicle_state</div>
              <div className="sensor-rate">50.0 Hz</div>
              <div className="sensor-status"><span className="dot amber" />LAG</div>
            </div>
            <div className="sensor">
              <div className="sensor-name">/radar/long</div>
              <div className="sensor-rate">20.0 Hz</div>
              <div className="sensor-status"><span className="dot" />OK</div>
            </div>
          </div>
        </div>

        <div className="module" style={{ marginTop: "auto" }}>
          <div className="section-label">Disk · Storage</div>
          <div className="stat" style={{ padding: "12px 14px" }}>
            <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
              <div className="stat-k">NVMe 0</div>
              <div style={{ fontFamily: "var(--font-mono)", fontSize: 12, color: "var(--text-1)" }}>
                412 / 960 <span style={{ color: "var(--text-3)" }}>GB</span>
              </div>
            </div>
            <div style={{
              marginTop: 8, height: 4, background: "var(--bg-3)", borderRadius: 2, overflow: "hidden"
            }}>
              <div style={{ width: "43%", height: "100%", background: "var(--amber-0)" }} />
            </div>
          </div>
        </div>
      </div>

      {/* ---------- Bottom Bar ---------- */}
      <div className="bottom">
        <div className="alert-tag">
          <div className="alert-k">Active Alert</div>
          <div className="alert-v">
            Lane assist disengaged — <span className="ko">차선 보조 해제됨</span>
          </div>
        </div>
        <div />
        <div className="action-row">
          <button className="btn danger">
            <span>STOP REC</span>
            <span className="kbd">⌘ S</span>
          </button>
          <button className="btn">
            <span>MARK</span>
            <span className="kbd">M</span>
          </button>
          <button className="btn primary">
            <span>NEW WAYPOINT</span>
            <span className="kbd">↵</span>
          </button>
        </div>
      </div>
    </div>
  );
}

window.HMIScreen = HMIScreen;

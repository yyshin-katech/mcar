/* global React, SpeedGauge, Vehicle, VehicleTop, DetectionOverlay, DetectionOverlayTop, RangeRings, RangeRingsTop, TrafficObjects, PlanPath,
          useRosConnection, useRosState, useDiagnostics, useTopicHz, useObjects, usePopup, useTraffic, useBag, buildTrafficObjs */

const GEAR_LABEL = { 1: 'P', 2: 'R', 3: 'N', 4: 'D' };

const SENSOR_ROWS = [
  { key: 'gps',   topic: '/ublox/navpvt' },
  { key: 'adcu',  topic: '/sensors/v_can' },
  { key: 'lidar', topic: '/track_Multi_RS' },
  { key: 'radar', topic: '/sensors/radar' },
  { key: 'v2x',   topic: '/v2x_msg' },
  { key: 'hmi',   topic: '/hmi/state' },
  { key: 'vcu',   topic: '/sensors/v_can' },
  { key: 'cam',   topic: '/cam_objects' },
  { key: 'ipc',   topic: '/ipc/heartbeat' },
];

function useClock() {
  const [now, setNow] = React.useState(() => new Date());
  React.useEffect(() => {
    const t = setInterval(() => setNow(new Date()), 1000);
    return () => clearInterval(t);
  }, []);
  return now;
}

function formatKstClock(date) {
  const fmt = new Intl.DateTimeFormat('ko-KR', {
    timeZone: 'Asia/Seoul',
    hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false,
  });
  return fmt.format(date) + ' KST';
}

// Convert EPSG:5179 (Korea TM) → rough WGS84 for header display only.
// This is intentionally approximate (single-point linearization around Seoul).
// Replace with proj4js if higher precision needed.
function epsg5179ToWgs(east, north) {
  if (!east && !north) return { lat: 0, lon: 0 };
  // Linearization origin: e=200000, n=500000 (Seoul-ish). 1 deg lat ≈ 111 km.
  const lat0 = 37.5642, lon0 = 127.0016;
  const e0 = 200000, n0 = 500000;
  const lat = lat0 + (north - n0) / 110946;
  const lon = lon0 + (east  - e0) / (111320 * Math.cos(lat0 * Math.PI / 180));
  return { lat, lon };
}

// ---- Top-bar pieces ----

function TopBar({ autonomous, connected }) {
  const now = useClock();
  return (
    <div className="topbar">
      <div className="topbar-left">
        <div className="brand">
          <div className="brand-mark">i5</div>
          <span>IONIQ EV · HMI</span>
        </div>
        <div className="crumb">SESSION <b>// MCAR-LIVE</b></div>
      </div>
      <div className="topbar-center">
        <div className="system-tag">
          <span className={`dot${connected ? '' : ' red'}`} />
          ROS 1 NOETIC · {connected ? 'ONLINE' : 'OFFLINE'}
        </div>
        <div className="system-tag">
          <span className={`dot${autonomous ? ' amber' : ''}`} style={!autonomous ? { background: '#5A616B', boxShadow: 'none' } : undefined} />
          AUTONOMY {autonomous ? 'L2+' : 'STBY'}
        </div>
        <div className="system-tag"><span className="dot" />CAN BUS · 500 KBPS</div>
      </div>
      <div className="topbar-right">
        <div className="crumb">CPU <b>—</b> · MEM <b>—</b></div>
        <div className="clock">{formatKstClock(now)}</div>
      </div>
    </div>
  );
}

// ---- Right-panel pieces ----

function statusToDot(code) {
  // BaseHmiStateController emits 0=normal, 1=warning, 2=fail, 3=unknown.
  if (code === 0) return { cls: 'dot', label: 'OK' };
  if (code === 1) return { cls: 'dot amber', label: 'LAG' };
  if (code === 2) return { cls: 'dot red', label: 'FAIL' };
  return { cls: 'dot', label: '—' };
}

function SensorList({ diagnostics, topicHz }) {
  const status = (diagnostics && diagnostics.status) || {};
  return (
    <div>
      {SENSOR_ROWS.map(row => {
        const d = statusToDot(status[row.key]);
        const hz = (topicHz && typeof topicHz[row.key] === 'number')
          ? `${topicHz[row.key].toFixed(1)} Hz`
          : '— Hz';
        return (
          <div className="sensor" key={row.key}>
            <div className="sensor-name">{row.topic}</div>
            <div className="sensor-rate">{hz}</div>
            <div className="sensor-status"><span className={d.cls} />{d.label}</div>
          </div>
        );
      })}
    </div>
  );
}

// ---- Bottom alert + actions ----

function severityColorVar(severity) {
  if (severity === 'fatal' || severity === 'error') return 'var(--red)';
  if (severity === 'warn') return 'var(--amber-0)';
  return 'var(--text-1)';
}

function BottomBar({ popup, bag, onBagToggle }) {
  const text = (popup && popup.text) || '시스템 정상';
  const severity = (popup && popup.severity) || 'info';
  const recording = bag && bag.recording;
  return (
    <div className="bottom">
      <div className="alert-tag">
        <div className="alert-k">Active Alert</div>
        <div className="alert-v">
          <span className="ko" style={{ color: severityColorVar(severity) }}>{text}</span>
        </div>
      </div>
      <div />
      <div className="action-row">
        <button className="btn danger" onClick={onBagToggle}>
          <span>{recording ? 'STOP REC' : 'START REC'}</span>
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
  );
}

// ---- Main screen ----

function HMIScreen({
  vehicleStyle = 'solid',
  topView = false,
  accent = 'amber',
  typeface = 'sans',
}) {
  const { connected, publishBagToggle } = useRosConnection();
  const state = useRosState();
  const diagnostics = useDiagnostics();
  const topicHz = useTopicHz();
  const objects = useObjects();
  const popup = usePopup();
  const bag = useBag();

  const accentColors = {
    amber:  { c0: '#FFB547', c1: '#F59E2C', c2: '#C97A14', glow: 'rgba(255,181,71,0.18)' },
    cyan:   { c0: '#5EEAD4', c1: '#2DD4BF', c2: '#0F766E', glow: 'rgba(94,234,212,0.18)' },
    violet: { c0: '#C4B5FD', c1: '#A78BFA', c2: '#7C3AED', glow: 'rgba(196,181,253,0.18)' },
    lime:   { c0: '#D9F99D', c1: '#BEF264', c2: '#65A30D', glow: 'rgba(217,249,157,0.18)' },
  }[accent] || { c0: '#FFB547', c1: '#F59E2C', c2: '#C97A14', glow: 'rgba(255,181,71,0.18)' };

  const styleVars = {
    '--amber-0': accentColors.c0,
    '--amber-1': accentColors.c1,
    '--amber-2': accentColors.c2,
    '--amber-glow': accentColors.glow,
    '--font-display': typeface === 'mono'
      ? '"JetBrains Mono", ui-monospace, monospace'
      : '"Inter", "Pretendard", sans-serif',
  };

  const speed = Math.max(0, Math.round((state.speed || 0)));
  const gear = GEAR_LABEL[state.gear] || '—';
  const autonomous = state.mode === 1;
  const aeb = !!state.aeb;
  const modeLabel = aeb ? 'AEB' : (autonomous ? 'AUTO' : 'MANUAL');
  const speedLimit = state.speed_limit || 0;
  const linkId = state.link_id || 0;
  const laneLabel = state.lane_label || '—';
  const onOdd = !!state.on_odd;

  const ego = state.ego || { east: 0, north: 0, yaw: 0 };
  const wgs = epsg5179ToWgs(ego.east, ego.north);

  const trafficObjs = React.useMemo(() => buildTrafficObjs(objects), [objects]);

  return (
    <div className="hmi" style={styleVars}>
      <TopBar autonomous={autonomous} connected={connected} />

      {/* ---------- Left Panel ---------- */}
      <div className="left">
        <div>
          <div className="section-label">Vehicle State</div>
          <SpeedGauge speed={speed} max={120} mode={modeLabel} />
        </div>

        <div>
          <div className="section-label">Drive Mode</div>
          <div className="drive-row">
            {['P', 'R', 'N', 'D'].map((m) => (
              <div key={m} className={`drive-cell ${m === gear ? 'active' : ''}`}>{m}</div>
            ))}
          </div>
        </div>

        <div>
          <div className="section-label">Path · ODD</div>
          <div className="stat-grid">
            <div className="stat">
              <div className="stat-k">Speed Lim</div>
              <div className="stat-v">{speedLimit || '—'}<small>km/h</small></div>
            </div>
            <div className="stat">
              <div className="stat-k">On ODD</div>
              <div className="stat-v" style={{ color: onOdd ? 'var(--green)' : 'var(--red)' }}>
                {onOdd ? 'YES' : 'NO'}
              </div>
            </div>
            <div className="stat">
              <div className="stat-k">Link</div>
              <div className="stat-v" style={{ fontSize: 14, fontFamily: 'var(--font-mono)' }}>
                {linkId || '—'}
              </div>
            </div>
            <div className="stat">
              <div className="stat-k">Lane</div>
              <div className="stat-v">{laneLabel}</div>
            </div>
          </div>
        </div>

        <div style={{ marginTop: 'auto' }}>
          <div className="section-label">Heading · GNSS</div>
          <div className="stat-grid">
            <div className="stat">
              <div className="stat-k">Lat</div>
              <div className="stat-v" style={{ fontSize: 14, fontFamily: 'var(--font-mono)' }}>
                {wgs.lat ? wgs.lat.toFixed(4) + '°N' : '—'}
              </div>
            </div>
            <div className="stat">
              <div className="stat-k">Lon</div>
              <div className="stat-v" style={{ fontSize: 14, fontFamily: 'var(--font-mono)' }}>
                {wgs.lon ? wgs.lon.toFixed(4) + '°E' : '—'}
              </div>
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
        {topView && <TrafficObjects objs={trafficObjs} />}

        <div className="stage-corner tl">FRONT · LIDAR + CAM</div>
        <div className="compass">N</div>
        <div className="stage-corner bl">SCALE · 1 : 200</div>
        <div className="stage-corner br">EPSG · 5179</div>

        <div className={`bag-banner ${bag && bag.recording ? 'recording' : ''}`}>
          <div className="rec-dot" />
          <div className="bag-text">
            <div className="bag-title">{bag && bag.recording ? 'REC · ROSBAG' : 'BAG · IDLE'}</div>
            <div className="bag-name">{(bag && bag.info) || '—'}</div>
          </div>
          <div className="bag-time">{(bag && bag.recording) ? '● LIVE' : '—'}</div>
        </div>

        <div className="vehicle" style={topView ? { transform: 'none', filter: 'none', zIndex: 6 } : undefined}>
          {topView
            ? <VehicleTop wireframe={vehicleStyle === 'wireframe'} accent={accentColors.c0} />
            : <Vehicle wireframe={vehicleStyle === 'wireframe'} />}
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
              <div className="obj-meta">—</div>
            </div>
            <div className="objective done">
              <div className="obj-mark" />
              <div className="obj-text">Depart from depot</div>
              <div className="obj-meta">—</div>
            </div>
            <div className="objective active">
              <div className="obj-mark" />
              <div className="obj-text">{autonomous ? 'Autonomous corridor' : 'Manual drive'}</div>
              <div className="obj-meta">{linkId || '—'}</div>
            </div>
            <div className="objective">
              <div className="obj-mark" />
              <div className="obj-text">Return waypoint</div>
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
          <SensorList diagnostics={diagnostics} topicHz={topicHz} />
        </div>

        <div className="module" style={{ marginTop: 'auto' }}>
          <div className="section-label">Disk · Storage</div>
          <div className="stat" style={{ padding: '12px 14px' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'baseline' }}>
              <div className="stat-k">NVMe 0</div>
              <div style={{ fontFamily: 'var(--font-mono)', fontSize: 12, color: 'var(--text-1)' }}>
                — / — <span style={{ color: 'var(--text-3)' }}>GB</span>
              </div>
            </div>
            <div style={{
              marginTop: 8, height: 4, background: 'var(--bg-3)', borderRadius: 2, overflow: 'hidden',
            }}>
              <div style={{ width: '0%', height: '100%', background: 'var(--amber-0)' }} />
            </div>
          </div>
        </div>
      </div>

      <BottomBar popup={popup} bag={bag} onBagToggle={publishBagToggle} />
    </div>
  );
}

window.HMIScreen = HMIScreen;

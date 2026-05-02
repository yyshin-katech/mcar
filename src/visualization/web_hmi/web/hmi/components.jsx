/* global React */

// ============================================
// Speed Gauge — circular arc + numeric center
// ============================================
function SpeedGauge({ speed = 47, max = 120, mode = "iso" }) {
  const size = 272;
  const cx = size / 2;
  const cy = size / 2;
  const r = 116;
  const startAngle = 135; // degrees, 0=right, CCW
  const endAngle = 405;   // 270 deg sweep
  const sweep = endAngle - startAngle;
  const valueAngle = startAngle + (speed / max) * sweep;

  const polar = (angle, radius) => {
    const rad = ((angle - 90) * Math.PI) / 180;
    return [cx + radius * Math.cos(rad), cy + radius * Math.sin(rad)];
  };

  const arcPath = (a0, a1, radius) => {
    const [x0, y0] = polar(a0, radius);
    const [x1, y1] = polar(a1, radius);
    const large = a1 - a0 > 180 ? 1 : 0;
    return `M ${x0} ${y0} A ${radius} ${radius} 0 ${large} 1 ${x1} ${y1}`;
  };

  // tick marks every 10 km/h
  const ticks = [];
  for (let v = 0; v <= max; v += 10) {
    const a = startAngle + (v / max) * sweep;
    const isMajor = v % 20 === 0;
    const inner = isMajor ? r - 14 : r - 8;
    const outer = r;
    const [x0, y0] = polar(a, inner);
    const [x1, y1] = polar(a, outer);
    const isPast = v <= speed;
    ticks.push(
      <line
        key={v}
        x1={x0} y1={y0} x2={x1} y2={y1}
        stroke={isPast ? "#FFB547" : "#313740"}
        strokeWidth={isMajor ? 1.5 : 1}
        opacity={isPast ? 0.95 : 0.6}
      />
    );
  }

  // major number labels
  const labels = [0, 40, 80, 120].map((v) => {
    const a = startAngle + (v / max) * sweep;
    const [x, y] = polar(a, r - 28);
    return (
      <text
        key={v}
        x={x} y={y}
        fill="#5A616B"
        fontSize="10"
        fontFamily="JetBrains Mono, monospace"
        textAnchor="middle"
        dominantBaseline="middle"
        letterSpacing="1"
      >
        {v}
      </text>
    );
  });

  return (
    <div className="gauge-wrap">
      <svg className="gauge-svg" width={size} height={size} viewBox={`0 0 ${size} ${size}`}>
        {/* background track */}
        <path
          d={arcPath(startAngle, endAngle, r)}
          fill="none"
          stroke="#1C1F23"
          strokeWidth="2"
        />
        {/* progress arc */}
        <path
          d={arcPath(startAngle, valueAngle, r)}
          fill="none"
          stroke="url(#amberGrad)"
          strokeWidth="3"
          strokeLinecap="round"
        />
        {/* glow under arc */}
        <path
          d={arcPath(startAngle, valueAngle, r)}
          fill="none"
          stroke="#FFB547"
          strokeWidth="8"
          strokeLinecap="round"
          opacity="0.12"
          filter="blur(4px)"
        />

        {ticks}
        {labels}

        <defs>
          <linearGradient id="amberGrad" x1="0%" y1="0%" x2="100%" y2="0%">
            <stop offset="0%" stopColor="#C97A14" />
            <stop offset="100%" stopColor="#FFB547" />
          </linearGradient>
        </defs>
      </svg>

      <div className="gauge-center">
        <div className="gauge-value">{speed}</div>
        <div className="gauge-unit">KM / H</div>
        <div className="gauge-label">{mode.toUpperCase()} • DRIVE</div>
      </div>
    </div>
  );
}

// ============================================
// Vehicle (3D isometric Ioniq5 silhouette)
// ============================================
function Vehicle({ wireframe = false }) {
  const stroke = "#FFB547";
  const fillBody = wireframe ? "none" : "#2D3239";
  const fillTop  = wireframe ? "none" : "#373D45";
  const fillWindow = wireframe ? "none" : "#15171A";

  return (
    <svg width="380" height="220" viewBox="0 0 380 220" className="vehicle-svg">
      <defs>
        <linearGradient id="bodyGrad" x1="0%" y1="0%" x2="0%" y2="100%">
          <stop offset="0%" stopColor="#3A4048" />
          <stop offset="100%" stopColor="#1F2329" />
        </linearGradient>
        <linearGradient id="topGrad" x1="0%" y1="0%" x2="0%" y2="100%">
          <stop offset="0%" stopColor="#454B54" />
          <stop offset="100%" stopColor="#2A2F36" />
        </linearGradient>
      </defs>

      {/* shadow under car */}
      {!wireframe && (
        <ellipse cx="190" cy="200" rx="150" ry="14" fill="#000" opacity="0.5" />
      )}

      {/* lower body (side panel) */}
      <path
        d="M 40 145 L 60 110 L 100 95 L 280 92 L 330 108 L 350 145 L 340 175 L 50 175 Z"
        fill={wireframe ? "none" : "url(#bodyGrad)"}
        stroke={wireframe ? stroke : "#0E0F11"}
        strokeWidth={wireframe ? 1.5 : 1}
      />

      {/* roof / greenhouse */}
      <path
        d="M 110 95 L 135 60 L 250 60 L 275 95 Z"
        fill={wireframe ? "none" : "url(#topGrad)"}
        stroke={wireframe ? stroke : "#0E0F11"}
        strokeWidth={wireframe ? 1.5 : 1}
      />

      {/* windshield */}
      <path
        d="M 138 65 L 158 92 L 232 92 L 247 65 Z"
        fill={wireframe ? "none" : fillWindow}
        stroke={wireframe ? stroke : "none"}
        strokeWidth="1"
        opacity={wireframe ? 1 : 0.85}
      />

      {/* side windows */}
      <path
        d="M 113 92 L 135 67 L 156 67 L 156 92 Z"
        fill={wireframe ? "none" : fillWindow}
        stroke={wireframe ? stroke : "none"}
        strokeWidth="1"
      />
      <path
        d="M 162 67 L 220 67 L 220 92 L 162 92 Z"
        fill={wireframe ? "none" : fillWindow}
        stroke={wireframe ? stroke : "none"}
        strokeWidth="1"
      />
      <path
        d="M 226 67 L 248 67 L 270 92 L 226 92 Z"
        fill={wireframe ? "none" : fillWindow}
        stroke={wireframe ? stroke : "none"}
        strokeWidth="1"
      />

      {/* belt line */}
      {!wireframe && (
        <line x1="60" y1="110" x2="330" y2="108" stroke="#0E0F11" strokeWidth="1.5" />
      )}

      {/* signature pixel light bar — front */}
      <rect x="298" y="120" width="34" height="6" fill={stroke} opacity={wireframe ? 1 : 0.95} />
      <rect x="298" y="128" width="34" height="2" fill={stroke} opacity="0.4" />
      {/* rear pixel light */}
      <rect x="48" y="120" width="20" height="5" fill="#F87171" opacity={wireframe ? 0.6 : 0.7} />

      {/* wheels */}
      <g>
        <ellipse cx="100" cy="172" rx="28" ry="11" fill="#0E0F11" />
        <ellipse cx="100" cy="170" rx="24" ry="9"
          fill={wireframe ? "none" : "#15171A"}
          stroke={wireframe ? stroke : "#2D3239"}
          strokeWidth="1.5" />
        <ellipse cx="100" cy="170" rx="10" ry="4"
          fill={wireframe ? "none" : "#2D3239"}
          stroke={wireframe ? stroke : "none"}
          strokeWidth="1" />
      </g>
      <g>
        <ellipse cx="290" cy="172" rx="28" ry="11" fill="#0E0F11" />
        <ellipse cx="290" cy="170" rx="24" ry="9"
          fill={wireframe ? "none" : "#15171A"}
          stroke={wireframe ? stroke : "#2D3239"}
          strokeWidth="1.5" />
        <ellipse cx="290" cy="170" rx="10" ry="4"
          fill={wireframe ? "none" : "#2D3239"}
          stroke={wireframe ? stroke : "none"}
          strokeWidth="1" />
      </g>

      {/* door cut lines */}
      {!wireframe && (
        <>
          <line x1="158" y1="92" x2="158" y2="170" stroke="#0E0F11" strokeWidth="1" opacity="0.7" />
          <line x1="220" y1="92" x2="220" y2="170" stroke="#0E0F11" strokeWidth="1" opacity="0.7" />
        </>
      )}

      {/* heading arrow */}
      <path
        d="M 360 145 L 376 152 L 360 159 Z"
        fill={stroke}
        opacity="0.7"
      />
    </svg>
  );
}

// ============================================
// Detection arc — front cone + side fans
// ============================================
function DetectionOverlay() {
  return (
    <svg
      className="detect-overlay"
      viewBox="-350 -350 700 700"
      width="700" height="700"
      style={{ position: "absolute", left: "50%", top: "55%", transform: "translate(-50%, -50%) rotateX(58deg)", pointerEvents: "none" }}
    >
      {/* front sensor cone */}
      <path
        d="M 0 0 L 280 -80 A 290 290 0 0 0 280 80 Z"
        fill="rgba(255,181,71,0.10)"
        stroke="rgba(255,181,71,0.35)"
        strokeWidth="1"
      />
      {/* rear cone */}
      <path
        d="M 0 0 L -180 -55 A 188 188 0 0 0 -180 55 Z"
        fill="rgba(96,165,250,0.06)"
        stroke="rgba(96,165,250,0.25)"
        strokeWidth="1"
      />
      {/* side scanlines */}
      <line x1="0" y1="0" x2="0" y2="-260" stroke="rgba(255,181,71,0.2)" strokeWidth="0.5" strokeDasharray="3 4" />
      <line x1="0" y1="0" x2="0" y2="260" stroke="rgba(255,181,71,0.2)" strokeWidth="0.5" strokeDasharray="3 4" />
    </svg>
  );
}

// ============================================
// Range Rings (concentric distance markers)
// ============================================
function RangeRings() {
  const rings = [
    { r: 80,  label: "10m", solid: false },
    { r: 160, label: "25m", solid: true  },
    { r: 240, label: "50m", solid: false },
    { r: 320, label: "100m", solid: false },
  ];
  return (
    <div className="range-rings" style={{ width: 1, height: 1 }}>
      {rings.map((ring, i) => (
        <React.Fragment key={i}>
          <div
            className={`range-ring${ring.solid ? " solid" : ""}`}
            style={{ width: ring.r * 2, height: ring.r * 2 }}
          />
          <div
            className="range-label"
            style={{
              left: ring.r,
              top: 0,
              transform: `translate(-50%, -50%) rotateX(-58deg)`,
            }}
          >
            {ring.label}
          </div>
        </React.Fragment>
      ))}
    </div>
  );
}

// ============================================
// VehicleTop — top-down stylized box + chevron
// (matches reference: simple cyan/amber rounded
//  rectangle with directional chevron marker)
// ============================================
function VehicleTop({ wireframe = false, accent = "#FFB547" }) {
  // unified simple style — wireframe just removes the fill
  const fill = wireframe ? "none" : "rgba(255,181,71,0.10)";
  return (
    <svg width="56" height="110" viewBox="-28 -55 56 110" className="vehicle-top-svg">
      {/* body — rounded rectangle */}
      <rect
        x="-22" y="-46" width="44" height="92" rx="6"
        fill={fill}
        stroke={accent}
        strokeWidth="1.5"
      />
      {/* heading chevron (front, pointing up) */}
      <path
        d="M -10 -34 L 0 -44 L 10 -34"
        fill="none"
        stroke={accent}
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
      {/* center dot (vehicle origin) */}
      <circle cx="0" cy="0" r="2" fill={accent} />
      {/* axle hint lines */}
      <line x1="-22" y1="-22" x2="22" y2="-22" stroke={accent} strokeWidth="0.6" opacity="0.4" />
      <line x1="-22" y1="22"  x2="22" y2="22"  stroke={accent} strokeWidth="0.6" opacity="0.4" />
    </svg>
  );
}

// ============================================
// TrafficObjects — surrounding detected entities
// (CAR / TRUCK / PED boxes with ID + dist/speed)
// ============================================
function TrafficObjects({ objs: objsProp }) {
  const fallback = [
    { id: "#203", kind: "TRUCK",  x:  60, y: -200, w: 36, h: 72, color: "#7C3AED", dist: "42.5m", spd: "+4.0m/s" },
    { id: "#156", kind: "CAR",    x:  90, y:  -90, w: 28, h: 50, color: "#7C3AED", dist: "20.4m", spd: "−2.1m/s" },
    { id: "#091", kind: "PED",    x:  60, y:  -20, w: 12, h: 16, color: "#FFB547", dist: "14.3m", spd: "+0.0m/s" },
    { id: "#142", kind: "CAR",    x:  20, y:    0, w: 24, h: 44, color: "#F472B6", dist: "8.2m",  spd: "+0.0m/s" },
    { id: "#088", kind: "PED",    x: -50, y:   30, w: 12, h: 16, color: "#FFB547", dist: "11.1m", spd: "+0.0m/s" },
    { id: "#311", kind: "CAR",    x: -90, y:   90, w: 28, h: 50, color: "#F472B6", dist: "7.4m",  spd: "+0.0m/s" },
  ];
  const objs = (objsProp && objsProp.length) ? objsProp : fallback;
  return (
    <svg
      style={{
        position: "absolute",
        left: "50%", top: "50%",
        transform: "translate(-50%, -50%)",
        pointerEvents: "none",
      }}
      viewBox="-300 -300 600 600"
      width="600" height="600"
    >
      {objs.map((o) => (
        <g key={o.id}>
          <rect
            x={o.x - o.w / 2}
            y={o.y - o.h / 2}
            width={o.w}
            height={o.h}
            rx="2"
            fill={o.color}
            fillOpacity="0.08"
            stroke={o.color}
            strokeWidth="1.2"
          />
          {/* label */}
          <text
            x={o.x + o.w / 2 + 4}
            y={o.y - o.h / 2 + 6}
            fill={o.color}
            fontSize="7"
            fontFamily="JetBrains Mono, monospace"
            letterSpacing="0.5"
          >
            {o.id} · {o.kind}
          </text>
          <text
            x={o.x + o.w / 2 + 4}
            y={o.y - o.h / 2 + 14}
            fill="#8A9099"
            fontSize="6"
            fontFamily="JetBrains Mono, monospace"
          >
            {o.dist} · {o.spd}
          </text>
        </g>
      ))}
    </svg>
  );
}

// ============================================
// PlanPath — central planned trajectory line
// ============================================
function PlanPath() {
  return (
    <svg
      style={{
        position: "absolute",
        left: "50%", top: "50%",
        transform: "translate(-50%, -50%)",
        pointerEvents: "none",
      }}
      viewBox="-300 -300 600 600"
      width="600" height="600"
    >
      <line x1="0" y1="0" x2="0" y2="-280" stroke="#F472B6" strokeWidth="1.2" opacity="0.85" />
      <line x1="0" y1="0" x2="0" y2="60"   stroke="#F472B6" strokeWidth="1.2" opacity="0.5" />
      {/* tick marks every ~20 units */}
      {[-40, -80, -120, -160, -200, -240].map((y) => (
        <line key={y} x1="-4" y1={y} x2="4" y2={y} stroke="#F472B6" strokeWidth="1" opacity="0.7" />
      ))}
    </svg>
  );
}

// ============================================
// DetectionOverlayTop — radar fans for top view
// ============================================
function DetectionOverlayTop() {
  return (
    <svg
      className="detect-overlay-top"
      viewBox="-300 -300 600 600"
      width="600" height="600"
      style={{ position: "absolute", left: "50%", top: "50%", transform: "translate(-50%, -50%)", pointerEvents: "none" }}
    >
      {/* front cone (forward = up = -y) */}
      <path
        d="M 0 0 L 200 -260 A 330 330 0 0 0 -200 -260 Z"
        fill="rgba(255,181,71,0.10)"
        stroke="rgba(255,181,71,0.35)"
        strokeWidth="1"
      />
      {/* rear cone */}
      <path
        d="M 0 0 L 130 200 A 240 240 0 0 1 -130 200 Z"
        fill="rgba(96,165,250,0.06)"
        stroke="rgba(96,165,250,0.25)"
        strokeWidth="1"
      />
      {/* side scan lines (left/right) */}
      <line x1="0" y1="0" x2="-260" y2="0" stroke="rgba(255,181,71,0.18)" strokeWidth="0.5" strokeDasharray="3 4" />
      <line x1="0" y1="0" x2="260" y2="0" stroke="rgba(255,181,71,0.18)" strokeWidth="0.5" strokeDasharray="3 4" />
      <line x1="0" y1="-260" x2="0" y2="260" stroke="rgba(255,181,71,0.12)" strokeWidth="0.5" strokeDasharray="3 4" />
    </svg>
  );
}

// ============================================
// RangeRingsTop — flat concentric rings
// ============================================
function RangeRingsTop() {
  const rings = [
    { r: 80,  label: "10m", solid: false },
    { r: 160, label: "25m", solid: true  },
    { r: 240, label: "50m", solid: false },
    { r: 320, label: "100m", solid: false },
  ];
  return (
    <div style={{ position: "absolute", left: "50%", top: "50%", width: 1, height: 1, pointerEvents: "none" }}>
      {rings.map((ring, i) => (
        <React.Fragment key={i}>
          <div
            style={{
              position: "absolute",
              left: "50%", top: "50%",
              transform: "translate(-50%, -50%)",
              width: ring.r * 2, height: ring.r * 2,
              border: `1px ${ring.solid ? "solid" : "dashed"} ${ring.solid ? "rgba(255,181,71,0.22)" : "rgba(200,204,210,0.12)"}`,
              borderRadius: "50%",
            }}
          />
          <div
            style={{
              position: "absolute",
              left: ring.r, top: 0,
              transform: "translate(-50%, -50%)",
              fontFamily: "JetBrains Mono, monospace",
              fontSize: 10,
              letterSpacing: "0.15em",
              color: "#5A616B",
              background: "#0E0F11",
              padding: "0 6px",
            }}
          >
            {ring.label}
          </div>
        </React.Fragment>
      ))}
    </div>
  );
}

Object.assign(window, { SpeedGauge, Vehicle, VehicleTop, DetectionOverlay, DetectionOverlayTop, RangeRings, RangeRingsTop, TrafficObjects, PlanPath });

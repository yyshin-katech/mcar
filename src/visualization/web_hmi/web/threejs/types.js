/* global window */

// Object type palette — bbox color per derived type.
const OBJ_PALETTE = {
  car:        '#ff5ea8',
  truck:      '#7c3aed',
  motorcycle: '#22e09a',
  pedestrian: '#ffb547',
  other:      '#7a8492',
  unknown:    '#7a8492',
};

// Object type → bbox dims override (size_y is height; ego x/y come from msg).
const OBJ_DIMS = {
  car:        { h: 1.5 },
  truck:      { h: 2.5 },
  motorcycle: { h: 1.0 },
  pedestrian: { h: 1.7, radius: 0.3 },
  other:      { h: 1.0 },
  unknown:    { h: 1.0 },
};

// valid_level enum (from sensing/can/lib/can_pub_func.cpp:641-674).
const KNOWN_TYPES = new Set([
  'Car', 'Truck', 'Motorcycle', 'Pedestrian', 'Other', 'Unknown', 'Invalid',
]);

// Derive normalized type from /track_Multi_RS object.
// Priority 1: parse first comma-separated token of valid_level if it's a known type.
// Priority 2 (fallback): mirror pyqt_hmi rule (status==1 → pedestrian, else car).
function deriveType(obj) {
  const lvl = (obj && obj.valid_level) || '';
  const head = String(lvl).split(',')[0].trim();
  if (KNOWN_TYPES.has(head)) {
    if (head === 'Invalid' || head === 'Unknown') return 'unknown';
    return head.toLowerCase();
  }
  return obj && obj.status === 1 ? 'pedestrian' : 'car';
}

// HD map layer palette (cyan-on-dark, NGII-inspired).
const LAYER_STYLE = {
  A1_NODE:                    { color: '#5cf2ff', size: 0.4,  kind: 'point' },
  A2_LINK:                    { color: '#00e5ff', width: 1.0, kind: 'polyline' },
  A3_DRIVEWAYSECTION:         { color: '#4d7fa3', alpha: 0.18, kind: 'polygon' },
  A4_SUBSIDIARYSECTION:       { color: '#3a5f7d', alpha: 0.16, kind: 'polygon' },
  A5_PARKINGLOT:              { color: '#5a8fb4', alpha: 0.20, kind: 'polygon' },
  B1_SAFETYSIGN:              { color: '#fff352', alpha: 0.40, kind: 'polygon' },
  B2_SURFACELINEMARK:         { color: '#cfd8e3', width: 1.0, kind: 'polyline' },
  B3_SURFACEMARK:             { color: '#94a3b8', alpha: 0.28, kind: 'polygon' },
  C1_TRAFFICLIGHT:            { color: '#ff5252', size: 0.6,  kind: 'point' },
  C3_VEHICLEPROTECTIONSAFETY: { color: '#ef4444', width: 1.0, kind: 'polyline' },
  C4_SPEEDBUMP:               { color: '#fb923c', alpha: 0.45, kind: 'polygon' },
  C5_HEIGHTBARRIER:           { color: '#a78bfa', width: 1.0, kind: 'polyline' },
  C6_POSTPOINT:               { color: '#94a3b8', size: 0.3,  kind: 'point' },
};

// Default layer visibility (mirrors plan's ControlPanel defaults).
const DEFAULT_LAYER_VIS = {
  A1_NODE: true,  A2_LINK: true,  A3_DRIVEWAYSECTION: true,
  A4_SUBSIDIARYSECTION: false,  A5_PARKINGLOT: false,
  B1_SAFETYSIGN: true,  B2_SURFACELINEMARK: true,
  B3_SURFACEMARK: false,  C1_TRAFFICLIGHT: true,
  C3_VEHICLEPROTECTIONSAFETY: false,  C4_SPEEDBUMP: false,
  C5_HEIGHTBARRIER: false,  C6_POSTPOINT: false,
};

window.OBJ_PALETTE = OBJ_PALETTE;
window.OBJ_DIMS = OBJ_DIMS;
window.KNOWN_TYPES = KNOWN_TYPES;
window.deriveType = deriveType;
window.LAYER_STYLE = LAYER_STYLE;
window.DEFAULT_LAYER_VIS = DEFAULT_LAYER_VIS;

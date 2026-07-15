// /ws/ingest handler (spec §4.3).

const state = require('./state');
const { broadcastUpdate } = require('./ws_subscribe');

const MAX_MSG_BYTES = 8 * 1024; // §7.1 8 KiB cap
const ID_RE = /^[A-Za-z0-9_-]{1,32}$/;

function isFiniteNum(x) {
  return typeof x === 'number' && Number.isFinite(x);
}

function validate(payload) {
  if (!payload || typeof payload !== 'object') return 'not_object';
  if (payload.type !== 'bsm') return `bad_type:${payload.type}`;

  const id = payload.vehicle_id;
  if (typeof id !== 'string' || id.length === 0) return 'missing_vehicle_id';
  if (!ID_RE.test(id)) return 'bad_vehicle_id_format';

  if (!Number.isFinite(payload.ts_unix_ms)) return 'missing_ts';

  if (!isFiniteNum(payload.lat)) return 'missing_lat';
  if (!isFiniteNum(payload.lon)) return 'missing_lon';
  if (!isFiniteNum(payload.heading_deg)) return 'missing_heading';
  if (!isFiniteNum(payload.speed_mps)) return 'missing_speed';

  if (payload.lat < -90 || payload.lat > 90) return 'lat_out_of_range';
  if (payload.lon < -180 || payload.lon > 180) return 'lon_out_of_range';

  return null;
}

let lastVehicleIdByWs = new WeakMap();

function attach(ws, req) {
  const ip = (req && req.socket && req.socket.remoteAddress) || 'unknown';
  console.log(`[ingest] connected from ${ip}`);

  ws.on('message', (data, isBinary) => {
    try {
      const buf = isBinary ? data : data; // ws gives Buffer for text frames too
      if (buf && buf.length && buf.length > MAX_MSG_BYTES) {
        console.warn(`[ingest] drop: msg too large (${buf.length} > ${MAX_MSG_BYTES})`);
        return;
      }
      let text;
      try {
        text = buf.toString('utf8');
      } catch (e) {
        console.warn(`[ingest] drop: cannot decode utf8: ${e.message}`);
        return;
      }

      let payload;
      try {
        payload = JSON.parse(text);
      } catch (e) {
        console.warn(`[ingest] drop: JSON parse failed: ${e.message}`);
        return;
      }

      const err = validate(payload);
      if (err) {
        console.warn(`[ingest] drop: ${err}`);
        return;
      }

      const stored = { ...payload, server_recv_ts_ms: Date.now() };
      state.set(payload.vehicle_id, stored);
      lastVehicleIdByWs.set(ws, payload.vehicle_id);
      broadcastUpdate(stored);
    } catch (e) {
      console.warn(`[ingest] unexpected error: ${e.message}`);
    }
  });

  ws.on('close', () => {
    const id = lastVehicleIdByWs.get(ws);
    console.log(`[ingest] disconnected (vehicle_id=${id || '?'})`);
  });

  ws.on('error', (err) => {
    console.warn(`[ingest] ws error: ${err.message}`);
  });
}

module.exports = { attach };

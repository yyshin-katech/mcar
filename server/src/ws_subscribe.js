// /ws/subscribe handler + broadcast (spec §3.2, §4.4).

const state = require('./state');

const STALE_THRESHOLD_MS = Number(process.env.STALE_THRESHOLD_MS) || 10000;

// Track all subscriber connections for broadcast.
const subscribers = new Set();

function attach(ws, req) {
  subscribers.add(ws);

  const ip = (req && req.socket && req.socket.remoteAddress) || 'unknown';
  console.log(`[subscribe] connected from ${ip} (subscribers=${subscribers.size})`);

  // Send initial snapshot (spec §3.2 (a)).
  try {
    const now = Date.now();
    const all = state.all();
    // Compute stale flag at send time.
    const vehiclesOut = {};
    for (const [id, v] of Object.entries(all)) {
      const stale = (now - (v.server_recv_ts_ms || 0)) > STALE_THRESHOLD_MS;
      vehiclesOut[id] = { ...v, stale };
    }
    const snapshot = {
      type: 'snapshot',
      server_ts_unix_ms: now,
      vehicles: vehiclesOut,
    };
    ws.send(JSON.stringify(snapshot));
  } catch (err) {
    console.warn(`[subscribe] snapshot send failed: ${err.message}`);
  }

  ws.on('message', () => {
    // v1: client -> server messages ignored.
  });

  ws.on('close', () => {
    subscribers.delete(ws);
    console.log(`[subscribe] disconnected (subscribers=${subscribers.size})`);
  });

  ws.on('error', (err) => {
    console.warn(`[subscribe] ws error: ${err.message}`);
    subscribers.delete(ws);
  });
}

// Broadcast update to all subscribers (spec §3.2 (b)).
function broadcastUpdate(vehiclePayload) {
  const frame = JSON.stringify({
    type: 'update',
    server_ts_unix_ms: Date.now(),
    vehicle: { ...vehiclePayload, stale: false },
  });
  for (const ws of subscribers) {
    if (ws.readyState !== ws.OPEN) continue;
    try {
      ws.send(frame);
    } catch (err) {
      // Spec §7.2: ignore individual client send failures and close.
      console.warn(`[subscribe] broadcast send failed: ${err.message}`);
      try { ws.close(); } catch (_) {}
      subscribers.delete(ws);
    }
  }
}

function subscriberCount() {
  return subscribers.size;
}

module.exports = { attach, broadcastUpdate, subscriberCount, STALE_THRESHOLD_MS };

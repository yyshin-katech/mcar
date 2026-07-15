// REST routes (spec §3.3, §4.1).

const express = require('express');
const state = require('./state');
const { STALE_THRESHOLD_MS } = require('./ws_subscribe');

function withStale(v, now) {
  const stale = (now - (v.server_recv_ts_ms || 0)) > STALE_THRESHOLD_MS;
  return { ...v, stale };
}

function build(startTimeMs) {
  const router = express.Router();

  router.get('/api/vehicles', (req, res) => {
    const now = Date.now();
    const out = {};
    for (const [id, v] of Object.entries(state.all())) {
      out[id] = withStale(v, now);
    }
    res.json({ server_ts_unix_ms: now, vehicles: out });
  });

  router.get('/api/vehicles/:id', (req, res) => {
    const v = state.get(req.params.id);
    if (!v) {
      res.status(404).json({ error: 'not_found' });
      return;
    }
    const now = Date.now();
    res.json({ server_ts_unix_ms: now, vehicle: withStale(v, now) });
  });

  router.get('/api/health', (req, res) => {
    res.json({
      status: 'ok',
      uptime_s: Math.floor((Date.now() - startTimeMs) / 1000),
      vehicle_count: state.size(),
    });
  });

  return router;
}

module.exports = { build };

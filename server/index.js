// Vehicle Tracker v1 entry — Express + ws single port (spec §4).

const path = require('path');
const http = require('http');
const express = require('express');
const { WebSocketServer } = require('ws');

const wsIngest = require('./src/ws_ingest');
const wsSubscribe = require('./src/ws_subscribe');
const routes = require('./src/routes');

const PORT = Number(process.env.PORT) || 8080;
const STALE_THRESHOLD_MS = Number(process.env.STALE_THRESHOLD_MS) || 10000;
const HOST = '0.0.0.0';
const FRONTEND_DIR = path.resolve(__dirname, '..', 'frontend');

const startTimeMs = Date.now();
const app = express();

// Permissive CORS (spec §4: "*") — manual headers (no extra dep).
app.use((req, res, next) => {
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type');
  if (req.method === 'OPTIONS') {
    res.sendStatus(204);
    return;
  }
  next();
});

// REST API.
app.use(routes.build(startTimeMs));

// Static frontend (spec §4.1).
app.use(express.static(FRONTEND_DIR));

// HTTP server + WS upgrade dispatch on the same port (spec §1, §4.1).
const server = http.createServer(app);

const wssIngest = new WebSocketServer({ noServer: true });
const wssSubscribe = new WebSocketServer({ noServer: true });

wssIngest.on('connection', wsIngest.attach);
wssSubscribe.on('connection', wsSubscribe.attach);

server.on('upgrade', (req, socket, head) => {
  const { url } = req;
  if (url === '/ws/ingest') {
    wssIngest.handleUpgrade(req, socket, head, (ws) => {
      wssIngest.emit('connection', ws, req);
    });
  } else if (url === '/ws/subscribe') {
    wssSubscribe.handleUpgrade(req, socket, head, (ws) => {
      wssSubscribe.emit('connection', ws, req);
    });
  } else {
    socket.destroy();
  }
});

server.listen(PORT, HOST, () => {
  console.log(`[server] listening on ${HOST}:${PORT}  (stale_threshold=${STALE_THRESHOLD_MS}ms)`);
});

// Graceful shutdown (helps verifier dry-run).
function shutdown(signal) {
  console.log(`[server] received ${signal}, shutting down`);
  server.close(() => process.exit(0));
  setTimeout(() => process.exit(1), 5000).unref();
}
process.on('SIGINT', () => shutdown('SIGINT'));
process.on('SIGTERM', () => shutdown('SIGTERM'));

module.exports = { app, server };

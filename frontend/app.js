// Vehicle Tracker v1 — frontend (spec §5).

(function () {
  'use strict';

  // ----- Map (spec §5.3) ----------------------------------------------------
  const map = L.map('map', {
    center: [37.38113, 126.72394], // 시화/오이도
    zoom: 15,
    maxZoom: 19,
  });
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '© OpenStreetMap contributors',
  }).addTo(map);

  // ----- State --------------------------------------------------------------
  const vehicles = {};          // vehicle_id -> latest VehicleState (incl. server_recv_ts_ms)
  const markers = {};           // vehicle_id -> L.marker
  let clockOffsetMs = 0;        // Date.now() - server_ts_unix_ms (spec §5.7)
  let clockOffsetCalibrated = false;

  // ----- DOM ---------------------------------------------------------------
  const elStatus = document.getElementById('conn-status');
  const elList = document.getElementById('vehicle-list');
  const elCount = document.getElementById('vehicle-count');

  function setStatus(state, label) {
    elStatus.className = `status ${state}`;
    elStatus.textContent = label;
  }

  // ----- Marker rendering (spec §5.4) ---------------------------------------
  function makeIcon(headingDeg, stale) {
    const cls = stale ? 'vehicle-marker stale' : 'vehicle-marker';
    // Triangle pointing up; rotated by heading_deg (0 = North).
    const html = `
      <div class="${cls}" style="transform: rotate(${Number(headingDeg) || 0}deg);">
        <svg viewBox="0 0 28 28" xmlns="http://www.w3.org/2000/svg">
          <polygon class="arrow" points="14,2 24,26 14,20 4,26" />
        </svg>
      </div>`;
    return L.divIcon({
      className: 'vehicle-divicon',
      html,
      iconSize: [28, 28],
      iconAnchor: [14, 14],
    });
  }

  function popupHtml(v, lastSeenSec) {
    const speedKph = (Number(v.speed_mps) || 0) * 3.6;
    return `
      <div>
        <div><strong>${escapeHtml(v.vehicle_id)}</strong></div>
        <div>lat: ${Number(v.lat).toFixed(6)}</div>
        <div>lon: ${Number(v.lon).toFixed(6)}</div>
        <div>heading: ${Number(v.heading_deg).toFixed(1)}°</div>
        <div>speed: ${speedKph.toFixed(1)} km/h</div>
        <div>last_seen: ${lastSeenSec.toFixed(1)} s</div>
      </div>`;
  }

  function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g, (c) => ({
      '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
    })[c]);
  }

  function lastSeenSec(v) {
    if (!v || typeof v.server_recv_ts_ms !== 'number') return 0;
    const ms = (Date.now() - clockOffsetMs) - v.server_recv_ts_ms;
    return Math.max(0, ms / 1000);
  }

  function isStale(v) {
    const s = lastSeenSec(v);
    if (s > 10) return true;
    return !!v.stale;
  }

  function upsertMarker(v) {
    const id = v.vehicle_id;
    const stale = isStale(v);
    const icon = makeIcon(v.heading_deg, stale);
    let m = markers[id];
    if (!m) {
      m = L.marker([v.lat, v.lon], { icon }).addTo(map);
      m.on('click', () => {
        m.bindPopup(popupHtml(v, lastSeenSec(v))).openPopup();
      });
      markers[id] = m;
    } else {
      m.setLatLng([v.lat, v.lon]);
      m.setIcon(icon);
      if (m.isPopupOpen && m.isPopupOpen()) {
        m.setPopupContent(popupHtml(v, lastSeenSec(v)));
      }
    }
  }

  function removeMarker(id) {
    const m = markers[id];
    if (m) {
      map.removeLayer(m);
      delete markers[id];
    }
  }

  // ----- Sidebar (spec §5.5) ------------------------------------------------
  function renderList() {
    const ids = Object.keys(vehicles).sort();
    elCount.textContent = `(${ids.length})`;
    elList.innerHTML = '';
    for (const id of ids) {
      const v = vehicles[id];
      const stale = isStale(v);
      const li = document.createElement('li');
      li.dataset.id = id;
      if (stale) li.classList.add('stale');
      const speedKph = (Number(v.speed_mps) || 0) * 3.6;
      const seen = lastSeenSec(v).toFixed(1);
      li.innerHTML = `
        <div class="vid">${escapeHtml(id)}</div>
        <div class="meta">${speedKph.toFixed(1)} km/h &middot; ${seen}s ago</div>`;
      li.addEventListener('click', () => {
        if (Number.isFinite(v.lat) && Number.isFinite(v.lon)) {
          map.setView([v.lat, v.lon], 17);
        }
      });
      elList.appendChild(li);
    }
  }

  // ----- Apply incoming state ----------------------------------------------
  function applyVehicle(v) {
    if (!v || typeof v.vehicle_id !== 'string') return;
    vehicles[v.vehicle_id] = v;
    upsertMarker(v);
  }

  function applySnapshot(msg) {
    if (!clockOffsetCalibrated && typeof msg.server_ts_unix_ms === 'number') {
      clockOffsetMs = Date.now() - msg.server_ts_unix_ms;
      clockOffsetCalibrated = true;
    }
    // Clear and rebuild.
    for (const id of Object.keys(vehicles)) {
      removeMarker(id);
      delete vehicles[id];
    }
    if (msg.vehicles && typeof msg.vehicles === 'object') {
      for (const [id, v] of Object.entries(msg.vehicles)) {
        applyVehicle(v);
      }
    }
    renderList();
  }

  function applyUpdate(msg) {
    if (msg.vehicle) {
      applyVehicle(msg.vehicle);
      renderList();
    }
  }

  // ----- WebSocket (spec §5.6) ---------------------------------------------
  let ws = null;
  let backoffMs = 1000;
  const MAX_BACKOFF = 30000;
  let reconnectTimer = null;

  function wsUrl() {
    const proto = (location.protocol === 'https:') ? 'wss:' : 'ws:';
    return `${proto}//${location.host}/ws/subscribe`;
  }

  function connect() {
    setStatus('connecting', 'connecting...');
    try {
      ws = new WebSocket(wsUrl());
    } catch (e) {
      scheduleReconnect();
      return;
    }

    ws.onopen = () => {
      backoffMs = 1000;
      setStatus('connected', 'connected');
    };
    ws.onmessage = (ev) => {
      let msg;
      try {
        msg = JSON.parse(ev.data);
      } catch (e) {
        return;
      }
      if (!msg || typeof msg !== 'object') return;
      if (msg.type === 'snapshot') applySnapshot(msg);
      else if (msg.type === 'update') applyUpdate(msg);
    };
    ws.onerror = () => {
      // onclose will follow.
    };
    ws.onclose = () => {
      setStatus('disconnected', `disconnected - retry in ${(backoffMs/1000)|0}s`);
      scheduleReconnect();
    };
  }

  function scheduleReconnect() {
    if (reconnectTimer) return;
    reconnectTimer = setTimeout(() => {
      reconnectTimer = null;
      backoffMs = Math.min(backoffMs * 2, MAX_BACKOFF);
      connect();
    }, backoffMs);
  }

  // ----- Stale refresh loop (spec §5.7) -------------------------------------
  setInterval(() => {
    let dirty = false;
    for (const id of Object.keys(vehicles)) {
      const v = vehicles[id];
      const stale = isStale(v);
      // Re-skin marker if stale-ness changed (cheap rerender via setIcon).
      const m = markers[id];
      if (m) {
        const wantsStale = stale;
        const isCurrentlyStale = m._lastStale === true;
        if (wantsStale !== isCurrentlyStale) {
          m.setIcon(makeIcon(v.heading_deg, wantsStale));
          m._lastStale = wantsStale;
          dirty = true;
        }
      }
    }
    // Always refresh list (last_seen counter ticks).
    renderList();
  }, 1000);

  connect();
})();

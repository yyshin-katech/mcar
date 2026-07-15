// In-memory vehicle state store (spec §4.2).
// Single Node event loop -> no lock needed. No TTL in v1.

const vehicles = new Map();

function set(id, payload) {
  vehicles.set(id, payload);
}

function get(id) {
  return vehicles.get(id);
}

function has(id) {
  return vehicles.has(id);
}

function all() {
  // Return as plain object (JSON-friendly).
  const out = {};
  for (const [id, v] of vehicles) {
    out[id] = v;
  }
  return out;
}

function del(id) {
  return vehicles.delete(id);
}

function size() {
  return vehicles.size;
}

module.exports = { set, get, has, all, delete: del, size };

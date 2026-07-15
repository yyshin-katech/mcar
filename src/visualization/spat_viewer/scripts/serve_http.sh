#!/usr/bin/env bash
# spat_viewer/scripts/serve_http.sh
# args: $1=port, $2=web_root
#
# Serves the static Leaflet viewer (index.html + data/*.json) on the given port.
# Bind 0.0.0.0 so the page is reachable from the LAN if needed.

set -euo pipefail

PORT="${1:-8080}"
WEB_ROOT="${2:-$(dirname "$0")/../web}"

if [[ ! -d "$WEB_ROOT" ]]; then
  echo "[serve_http.sh] web_root '$WEB_ROOT' does not exist" >&2
  exit 1
fi

cd "$WEB_ROOT"
echo "[serve_http.sh] serving '$WEB_ROOT' on http://0.0.0.0:${PORT}/"
exec python3 -m http.server "$PORT" --bind 0.0.0.0

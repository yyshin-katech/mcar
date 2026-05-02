/* global React, ROSLIB */

// ============================================
// rosbridge connection + 6 hooks for /hmi/* JSON
// ============================================

const RosContext = React.createContext({
  connected: false,
  ros: null,
  connectedSince: 0,
  lastMessageAgeMs: Infinity,
  markActivity: () => {},
  publishModeRequest: () => {},
  publishBagToggle: () => {},
});

function RosProvider({ url, children }) {
  const [connected, setConnected] = React.useState(false);
  const [connectedSince, setConnectedSince] = React.useState(0);
  const [lastMessageAgeMs, setLastMessageAgeMs] = React.useState(Infinity);
  const rosRef = React.useRef(null);
  const cmdsRef = React.useRef({});
  const lastMsgAtRef = React.useRef(0);

  React.useEffect(() => {
    let cancelled = false;
    let backoff = 1000; // 1s → 30s
    let retryTimer = null;

    const connect = () => {
      if (cancelled) return;
      const ros = new ROSLIB.Ros({ url });
      rosRef.current = ros;

      ros.on('connection', () => {
        if (cancelled) return;
        backoff = 1000;
        setConnected(true);
        setConnectedSince(Date.now());

        cmdsRef.current.modeRequest = new ROSLIB.Topic({
          ros, name: '/hmi/cmd/mode_request', messageType: 'std_msgs/Bool',
        });
        cmdsRef.current.bagToggle = new ROSLIB.Topic({
          ros, name: '/hmi/cmd/bag_toggle', messageType: 'std_msgs/Empty',
        });
      });

      ros.on('close', () => {
        if (cancelled) return;
        setConnected(false);
        setConnectedSince(0);
        lastMsgAtRef.current = 0;
        cmdsRef.current = {};
        retryTimer = setTimeout(connect, backoff);
        backoff = Math.min(backoff * 2, 30000);
      });

      ros.on('error', () => {
        // 'close' fires after error, so reconnect logic stays in close.
      });
    };

    connect();
    return () => {
      cancelled = true;
      if (retryTimer) clearTimeout(retryTimer);
      if (rosRef.current) {
        try { rosRef.current.close(); } catch (e) { /* ignore */ }
      }
    };
  }, [url]);

  // Sync lastMessageAgeMs from ref to state once per second.
  React.useEffect(() => {
    const id = setInterval(() => {
      const t = lastMsgAtRef.current;
      setLastMessageAgeMs(t ? (Date.now() - t) : Infinity);
    }, 500);
    return () => clearInterval(id);
  }, []);

  const markActivity = React.useCallback(() => {
    lastMsgAtRef.current = Date.now();
  }, []);

  const publishModeRequest = React.useCallback((value) => {
    const t = cmdsRef.current.modeRequest;
    if (!t) return;
    t.publish(new ROSLIB.Message({ data: !!value }));
  }, []);

  const publishBagToggle = React.useCallback(() => {
    const t = cmdsRef.current.bagToggle;
    if (!t) return;
    t.publish(new ROSLIB.Message({}));
  }, []);

  const value = React.useMemo(() => ({
    connected,
    ros: rosRef.current,
    connectedSince,
    lastMessageAgeMs,
    markActivity,
    publishModeRequest,
    publishBagToggle,
  }), [connected, connectedSince, lastMessageAgeMs, markActivity, publishModeRequest, publishBagToggle]);

  return <RosContext.Provider value={value}>{children}</RosContext.Provider>;
}

function useRosConnection() {
  return React.useContext(RosContext);
}

// Generic JSON-string subscriber hook.
// rosbridge delivers std_msgs/String as { data: '...json...' }.
function useJsonTopic(topicName, defaultValue) {
  const { connected, ros, markActivity } = useRosConnection();
  const [value, setValue] = React.useState(defaultValue);

  React.useEffect(() => {
    if (!ros || !connected) return undefined;
    const topic = new ROSLIB.Topic({
      ros, name: topicName, messageType: 'std_msgs/String',
    });
    const onMsg = (msg) => {
      if (markActivity) markActivity();
      try {
        setValue(JSON.parse(msg.data));
      } catch (e) {
        // ignore malformed payloads
      }
    };
    topic.subscribe(onMsg);
    return () => {
      try { topic.unsubscribe(onMsg); } catch (e) { /* ignore */ }
    };
  }, [ros, connected, topicName, markActivity]);

  return value;
}

// Topic staleness based on /hmi/topic_hz snapshot.
// graceHz: minimum acceptable Hz; below this → stale.
// hzKey: key in /hmi/topic_hz (e.g. 'gps', 'adcu', 'lidar', …).
function useTopicStale(hzKey, graceHz = 0.5) {
  const hz = useTopicHz();
  const v = hz[hzKey];
  if (v === undefined) return true;
  return v < graceHz;
}

// ---- Domain hooks ----

const STATE_DEFAULT = {
  speed: 0, gear: 0, mode: 0, aeb: false, steering: 0,
  ego: { east: 0, north: 0, yaw: 0 },
  gps: { rtk: 0, lon_std: 0, lat_std: 0 },
  speed_limit: 0, link_id: 0, lane_label: '', on_odd: 0,
  road_state: 0, selected_mode: 0,
};

function useRosState() {
  return useJsonTopic('/hmi/state', STATE_DEFAULT);
}

const DIAG_DEFAULT = {
  status: { gps: 0, adcu: 0, lidar: 0, radar: 0,
            v2x: 0, hmi: 0, vcu: 0, cam: 0, ipc: 0 },
};

function useDiagnostics() {
  return useJsonTopic('/hmi/diagnostics', DIAG_DEFAULT);
}

function useTopicHz() {
  return useJsonTopic('/hmi/topic_hz', {});
}

function useObjects() {
  return useJsonTopic('/hmi/objects', { count: 0, data: [] });
}

function usePopup() {
  return useJsonTopic('/hmi/popup', { text: '', severity: 'info' });
}

function useTraffic() {
  return useJsonTopic('/hmi/traffic', { color: 0, time_decisec: 0, look_at: null });
}

function useBag() {
  return useJsonTopic('/hmi/bag', { recording: false, info: '' });
}

// ---- Helpers exposed for HMIScreen ----

// Map ego-frame meters → stage SVG units.
// Stage TrafficObjects viewBox is -300..300 (600px). RangeRings: 100m == r=320,
// but the SVG itself spans 300 — the rings are ratio-rendered via CSS.
// We pick 100 m = 280 px (slightly inside viewBox) so labels stay visible.
function metersToStage(x_m, y_m, scale = 2.8) {
  // ROS REP-103 convention: x=forward, y=left. Stage convention: forward = -y, left = -x.
  return { sx: -y_m * scale, sy: -x_m * scale };
}

function objectColor(type) {
  if (type === 'pedestrian') return '#FFB547';
  if (type === 'truck' || type === 'bus') return '#7C3AED';
  return '#F472B6';
}

function objectKind(type) {
  if (type === 'pedestrian') return 'PED';
  if (type === 'truck') return 'TRUCK';
  if (type === 'bus') return 'BUS';
  return 'CAR';
}

function buildTrafficObjs(objects) {
  const list = (objects && objects.data) ? objects.data : [];
  return list.slice(0, 24).map((o) => {
    const { sx, sy } = metersToStage(o.x || 0, o.y || 0);
    const w = Math.max(8, (o.width  || 1.8) * 2.8);
    const h = Math.max(10, (o.length || 4.4) * 2.8);
    const dist = Math.hypot(o.x || 0, o.y || 0).toFixed(1) + 'm';
    const vmag = Math.hypot(o.vx || 0, o.vy || 0);
    const sign = ((o.vx || 0) >= 0) ? '+' : '-';
    const spd = `${sign}${vmag.toFixed(1)}m/s`;
    return {
      id: '#' + String(o.id ?? '?').padStart(3, '0'),
      kind: objectKind(o.type),
      x: sx, y: sy, w, h,
      color: objectColor(o.type),
      dist, spd,
    };
  });
}

Object.assign(window, {
  RosProvider, useRosConnection,
  useRosState, useDiagnostics, useTopicHz, useTopicStale,
  useObjects, usePopup, useTraffic, useBag,
  buildTrafficObjs,
});

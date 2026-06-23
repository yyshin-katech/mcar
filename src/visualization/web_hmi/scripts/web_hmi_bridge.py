#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""web_hmi_bridge — aggregates HmiStateController state into JSON topics.

Reuses ``BaseHmiStateController`` from the pyqt_hmi package. Subclasses it
to plug in:
- ``_emit(name, *args)`` → buffers state in dicts keyed by signal name
- ``_start_tick(period_ms, fn)`` → wraps in ``rospy.Timer``
- ``_schedule_mode_pulse_reset(ms)`` → one-shot ``rospy.Timer``

A second ``rospy.Timer`` (10 Hz) publishes the buffered state to:
  /hmi/state         std_msgs/String   (JSON, 10 Hz)
  /hmi/diagnostics   std_msgs/String   (JSON, 10 Hz)
  /hmi/objects       std_msgs/String   (JSON, on object updates, capped 10 Hz)
  /hmi/popup         std_msgs/String   (JSON, on change only)
  /hmi/traffic       std_msgs/String   (JSON, on change only)
  /hmi/bag           std_msgs/String   (JSON, on change only)
  /hmi/topic_hz      std_msgs/String   (JSON, 1 Hz)
  /hmi/ego_pose      std_msgs/String   (JSON, ~50 Hz raw, rviz-grade follow)

Inbound:
  /hmi/cmd/mode_request   std_msgs/Bool    → controller.request_mode(b.data)
  /hmi/cmd/bag_toggle     std_msgs/Empty   → controller.toggle_bag()
  /hmi/cmd/bag_lidar      std_msgs/Bool    → controller.set_bag_include_lidar(b.data)
"""
import json
import os
import sys
import time
from collections import defaultdict, deque

import rospy
from std_msgs.msg import Bool, Empty, String

try:
    import shapefile  # pyshp — optional, only used to publish /hmi/map
except ImportError:
    shapefile = None

try:
    from pyproj import Transformer
except ImportError:
    Transformer = None

# Make pyqt_hmi.scripts importable for BaseHmiStateController.
_PYQT_HMI_SCRIPTS = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    "..", "..", "pyqt_hmi", "scripts",
)
sys.path.insert(0, os.path.normpath(_PYQT_HMI_SCRIPTS))

from utils.hmi_state import BaseHmiStateController  # noqa: E402


class _HzTracker:
    """Per-topic incoming Hz, computed over a sliding window."""

    def __init__(self, window=2.0):
        self._stamps = defaultdict(lambda: deque(maxlen=200))
        self._window = window

    def tick(self, key):
        now = time.monotonic()
        cutoff = now - self._window
        d = self._stamps[key]
        while d and d[0] < cutoff:
            d.popleft()
        d.append(now)

    def snapshot(self):
        out = {}
        for key, d in self._stamps.items():
            if len(d) < 2:
                out[key] = 0.0
                continue
            dt = d[-1] - d[0]
            out[key] = round(((len(d) - 1) / dt) if dt > 0 else 0.0, 1)
        return out


class WebHmiBridge(BaseHmiStateController):
    """Connects BaseHmiStateController to rospy.Timer + JSON publishers."""

    def __init__(self):
        # Publishers
        self._pubs = {
            'state':       rospy.Publisher('/hmi/state',       String, queue_size=2),
            'diagnostics': rospy.Publisher('/hmi/diagnostics', String, queue_size=2),
            'objects':     rospy.Publisher('/hmi/objects',     String, queue_size=2),
            'popup':       rospy.Publisher('/hmi/popup',       String, queue_size=4, latch=True),
            'traffic':     rospy.Publisher('/hmi/traffic',     String, queue_size=4, latch=True),
            'bag':         rospy.Publisher('/hmi/bag',         String, queue_size=4, latch=True),
            'topic_hz':    rospy.Publisher('/hmi/topic_hz',    String, queue_size=2),
            'map':         rospy.Publisher('/hmi/map',         String, queue_size=1, latch=True),
            # 50 Hz raw ego pose stream (separate from /hmi/state's 10 Hz
            # snapshot) so the Three.js camera follow can run at rviz cadence.
            'ego_pose':    rospy.Publisher('/hmi/ego_pose',    String, queue_size=4),
        }

        # Last published payloads — used for deduplication on event topics
        self._last = {'popup': None, 'traffic': None, 'bag': None}

        # Hz tracking
        self._hz = _HzTracker(window=2.0)

        # Object publish throttle: 10 Hz cap
        self._objects_last_pub_t = 0.0
        self._objects_min_dt = 0.1
        self._objects_dirty = False

        # Mode pulse timer handle
        self._mode_pulse_timer = None

        # Now build the controller (subscribes to ROS, starts tick)
        super().__init__()

        # Inbound command subscribers
        rospy.Subscriber('/hmi/cmd/mode_request', Bool, self._on_mode_request)
        rospy.Subscriber('/hmi/cmd/bag_toggle', Empty, self._on_bag_toggle)
        rospy.Subscriber('/hmi/cmd/bag_lidar', Bool, self._on_bag_lidar)

        # Bag dir from rosparam if provided
        bag_dir = rospy.get_param('~bag_dir', None)
        if bag_dir:
            self.set_bag_dir(bag_dir)

        # 10 Hz publisher tick — pushes /hmi/state + /hmi/diagnostics snapshots
        self._publish_timer = rospy.Timer(rospy.Duration(0.1), self._publish_periodic)

        # 1 Hz Hz-tracker publisher
        self._hz_timer = rospy.Timer(rospy.Duration(1.0), self._publish_hz)

        # One-shot map publish (latched)
        self._publish_map_once()

        # Latch an explicit IDLE on /hmi/bag so subscribers don't fall back
        # to the JSX prop default and don't get a stale REC from a co-played
        # bag's latched message before the first real toggle.
        self._emit('bag_state_changed', self.bag_recording, self.bag_info)

        rospy.loginfo("web_hmi_bridge: ready, publishing on /hmi/*")

    # ─── BaseHmiStateController hook implementations ──────────────
    def _emit(self, name, *args):
        """Translate controller signals into bridge-side reactions."""
        if name == 'topic_event':
            self._hz.tick(args[0])
            return

        if name == 'objects_changed':
            # Throttle to 10 Hz
            now = time.monotonic()
            if now - self._objects_last_pub_t >= self._objects_min_dt:
                self._publish_objects(args[0])
                self._objects_last_pub_t = now
                self._objects_dirty = False
            else:
                self._objects_dirty = True
            return

        if name == 'popup_changed':
            payload = {'text': args[0], 'severity': args[1]}
            self._publish_dedup('popup', payload)
            return

        if name == 'traffic_changed':
            payload = {
                'color': int(args[0]),  # 0=none, 1=green, 2=amber, 3=red
                'time_decisec': int(args[1]),
                'look_at': {
                    'intersection_id': int(self.look_at_intersection_id),
                    'signal_group_id': int(self.look_at_signal_group_id),
                },
            }
            self._publish_dedup('traffic', payload)
            return

        if name == 'bag_state_changed':
            payload = {'recording': bool(args[0]), 'info': args[1]}
            self._publish_dedup('bag', payload)
            return

        if name == 'ego_pose_changed':
            # Fired every /localization/to_control_team callback (~50 Hz),
            # bypassing the 10 Hz snapshot timer so the web client can match
            # rviz-grade camera follow smoothness.
            self._pubs['ego_pose'].publish(String(data=json.dumps({
                'east':  round(float(args[0]), 3),
                'north': round(float(args[1]), 3),
                'yaw':   round(float(args[2]), 4),
            }, ensure_ascii=False)))
            return

        # Other signals are folded into the periodic /hmi/state snapshot —
        # no per-event publish needed (they are covered by the 10 Hz tick).

    def _start_tick(self, period_ms, callback):
        """Run the controller's 100 ms tick via rospy.Timer."""
        self._controller_tick = rospy.Timer(
            rospy.Duration(period_ms / 1000.0),
            lambda _evt: callback(),
        )

    def _schedule_mode_pulse_reset(self, ms):
        if self._mode_pulse_timer is not None:
            try:
                self._mode_pulse_timer.shutdown()
            except Exception:  # noqa: BLE001
                pass
        self._mode_pulse_timer = rospy.Timer(
            rospy.Duration(ms / 1000.0),
            lambda _evt: self._reset_mode_pulse(),
            oneshot=True,
        )

    # ─── publishers ───────────────────────────────────────────────
    def _publish_periodic(self, _evt):
        # /hmi/state snapshot
        self._pubs['state'].publish(String(data=json.dumps({
            'speed': round(float(self.current_speed), 2),
            'gear': int(self.gear_status),
            'mode': int(self.autonomous_mode),
            'aeb': bool(self.aeb_flag),
            'steering': round(float(self.steering_angle), 2),
            'ego': {
                'east': round(self.host_east, 3),
                'north': round(self.host_north, 3),
                'yaw': round(self.host_yaw, 4),
            },
            'gps': {
                'rtk': int(self.gps_rtk_code),
                'lon_std': round(self.gps_lon_std, 4),
                'lat_std': round(self.gps_lat_std, 4),
            },
            'speed_limit': int(self.speed_limit),
            'link_id': int(self.link_id) if self.link_id else 0,
            'lane_label': str(self.lane_label or ''),
            'on_odd': int(self.on_odd),
            'road_state': int(self.road_state),
            'selected_mode': int(self.selected_mode),
        }, ensure_ascii=False)))

        # /hmi/diagnostics snapshot
        self._pubs['diagnostics'].publish(String(data=json.dumps({
            'status': dict(self.diag_status),
        }, ensure_ascii=False)))

        # Late objects publish if throttled previously
        if self._objects_dirty:
            self._publish_objects(self.objects)
            self._objects_last_pub_t = time.monotonic()
            self._objects_dirty = False

    def _publish_objects(self, objects):
        # Sort by ego-frame distance so the frontend renders close objects
        # first. Upstream percept_topic_matcher already caps at 14.
        objects = sorted(
            objects,
            key=lambda o: (o.get('x', 0.0) ** 2 + o.get('y', 0.0) ** 2),
        )
        self._pubs['objects'].publish(String(data=json.dumps({
            'count': len(objects),
            'data': objects,
        }, ensure_ascii=False)))

    def _publish_hz(self, _evt):
        self._pubs['topic_hz'].publish(String(data=json.dumps(
            self._hz.snapshot(), ensure_ascii=False)))

    def _publish_map_once(self):
        """Load shapefile polylines (EPSG:5179) and latch on /hmi/map."""
        map_shp = rospy.get_param('~map_shp', '')
        if not map_shp:
            return
        if shapefile is None:
            rospy.logwarn("web_hmi_bridge: pyshp not installed; /hmi/map disabled")
            return
        if not os.path.isfile(map_shp):
            rospy.logwarn("web_hmi_bridge: map_shp not found: %s", map_shp)
            return
        try:
            polylines = []
            sf = shapefile.Reader(map_shp)
            # Optional reprojection: read sibling .prj; if it's not EPSG:5179
            # (e.g. senario3 = WGS_1984_UTM_Zone_52N → EPSG:32652), reproject.
            tx = None
            prj_path = os.path.splitext(map_shp)[0] + ".prj"
            if Transformer is not None and os.path.isfile(prj_path):
                with open(prj_path) as f:
                    wkt = f.read()
                if "UTM_Zone_52N" in wkt or "UTM zone 52N" in wkt:
                    tx = Transformer.from_crs("EPSG:32652", "EPSG:5179", always_xy=True)
            for shp in sf.shapes():
                if not shp.points:
                    continue
                # 1 cm precision is plenty for visualization; reduces payload.
                if tx is None:
                    polylines.append([[round(p[0], 2), round(p[1], 2)] for p in shp.points])
                else:
                    polylines.append([[round(e, 2), round(n, 2)]
                                      for e, n in (tx.transform(p[0], p[1]) for p in shp.points)])
            payload = json.dumps({'polylines': polylines}, ensure_ascii=False,
                                 separators=(',', ':'))
            self._pubs['map'].publish(String(data=payload))
            rospy.loginfo("web_hmi_bridge: /hmi/map latched — %d polylines, %d points (%d KB)",
                          len(polylines),
                          sum(len(p) for p in polylines),
                          len(payload) // 1024)
        except Exception as e:  # noqa: BLE001
            rospy.logerr("web_hmi_bridge: failed to load %s: %s", map_shp, e)

    def _publish_dedup(self, key, payload):
        s = json.dumps(payload, ensure_ascii=False)
        if self._last.get(key) == s:
            return
        self._last[key] = s
        self._pubs[key].publish(String(data=s))

    # ─── inbound commands ─────────────────────────────────────────
    def _on_mode_request(self, msg):
        self.request_mode(bool(msg.data))

    def _on_bag_toggle(self, _msg):
        self.toggle_bag()

    def _on_bag_lidar(self, msg):
        # True: LiDAR/인지 토픽 포함 저장, False: 제외 저장. 녹화 시작 시 반영.
        self.set_bag_include_lidar(msg.data)


def main():
    rospy.init_node('web_hmi_bridge', disable_signals=False)
    bridge = WebHmiBridge()
    try:
        rospy.spin()
    finally:
        try:
            bridge.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == '__main__':
    main()

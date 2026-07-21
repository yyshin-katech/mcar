#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""web_hmi_threejs_bridge — Three.js HMI variant data bridge.

Reads K-City HD map shapefiles (UTM Zone 52N) and publishes them as
EPSG:5179 polylines/polygons/points on a latched JSON topic. Subsequent
steps (S5+) add /track_Multi_RS and /percept_topic subscribers.

Topics published:
  /hmi/threejs/map      std_msgs/String (latched, one shot)
  /hmi/threejs/tracks   std_msgs/String (10 Hz, added in S5+)

Parameters:
  ~mapdir               directory containing K_CITY_2025/*.shp (default:
                        $(find gps_system_localizer)/mapfiles/K_CITY_2025)
"""
import json
import math
import os

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

try:
    import shapefile  # pyshp
except ImportError:
    shapefile = None

try:
    from pyproj import Transformer
except ImportError:
    Transformer = None

try:
    from perception_ros_msg.msg import RsPerceptionMsg
except ImportError:
    RsPerceptionMsg = None

try:
    from mmc_msgs.msg import to_control_team_from_local_msg
except ImportError:
    to_control_team_from_local_msg = None

# /percept_topic carries object metadata + cloud_indices, but its lidarframe.
# scan_pointcloud is empty on this bag (has_pointcloud=False). The actual
# point cloud lives on /fusion_lidar_points (sensor_msgs/PointCloud2, ~1.5 Hz,
# height=1 width=230400 in /base_link). cloud_indices index into that flat
# array. We cache the latest fusion cloud as a (N,3) float32 numpy slice and
# fan it out across the 10 Hz percept stream.
# Per-track cap. Lowered from 4096 to 256 because a single /hmi/threejs/tracks
# emission at the original cap was ~2.5 MB and rosbridge throttled the stream
# to ~1 Hz, making track boxes appear to jump in 1-second steps.
PERCEPT_MAX_POINTS_PER_TRACK = 256

# Confidence floor: perception emits a long tail of single-frame ghost tracks
# (~34% of unique IDs in the live bag) with confidence ≤0.80, vs stable
# tracks at ~0.999. Filtering here prevents the frontend from briefly
# rendering and disposing those ghosts (visible flicker).
PERCEPT_MIN_CONFIDENCE = 0.9


# coreinfo.type → lowercase string. ObjectType enum: 2=PED, 3=BIC → 보행자류.
# 그 외(0=UNKNOW, 1=CONE, 4=CAR, 5=TRUCK_BUS, 6=ULTRA_VEHICLE)는 "car".
def percept_type_str(type_int):
    return "pedestrian" if type_int in (2, 3) else "car"


# Source CRS for K_CITY_2025 (.prj says WGS_1984_UTM_Zone_52N).
SRC_EPSG = "EPSG:32652"
DST_EPSG = "EPSG:5179"

# Layer name -> kind. HDMap_Oido_New (MOLIT HD맵 11 표준 레이어, EPSG:32652).
# shape kinds verified via pyshp on /shp_map/HDMap_Oido_New/.
LAYERS_ALL = {
    "A1_NODE":                    "point",     # POINTZ     —    2463 nodes
    "A2_LINK":                    "polyline",  # POLYLINEZ  —    2544 lanelinks
    "A3_DRIVEWAYSECTION":         "polygon",   # POLYGONZ   —       8 driveway sections
    "B1_SAFETYSIGN":              "point",     # POINTZ     —     811 signs
    "B2_SURFACELINEMARK":         "polyline",  # POLYLINEZ  —    4101 lane marks
    "B3_SURFACEMARK":             "polygon",   # POLYGONZ   —    1374 surface marks
    "C1_TRAFFICLIGHT":            "point",     # POINTZ     —     625 traffic lights
    "C3_VEHICLEPROTECTIONSAFETY": "polyline",  # POLYLINEZ  —    1153 guardrails
    "C4_SPEEDBUMP":               "polygon",   # POLYGONZ   —      30 speed bumps
    "C5_HEIGHTBARRIER":           "polyline",  # POLYLINEZ  —      16 height barriers
    "C6_POSTPOINT":               "point",     # POINTZ     —     702 post points
}


class WebHmiThreejsBridge:
    def __init__(self):
        self._pub_map = rospy.Publisher(
            "/hmi/threejs/map", String, queue_size=1, latch=True,
        )
        # Latched planned-route ribbon (global_nav_hmi). Additive — the map/
        # track publishing below is unchanged.
        self._pub_route = rospy.Publisher(
            "/hmi/threejs/route", String, queue_size=1, latch=True,
        )
        # Track publishing is owned by web_hmi_threejs_tracks_cpp when this
        # param is false (default). The C++ node is faster on the percept hot
        # path; keeping Python's advertiser around would create a dual
        # publisher (last-advertise-wins + queue contention).
        self._publish_tracks = bool(rospy.get_param("~publish_tracks", False))
        if self._publish_tracks:
            self._pub_tracks = rospy.Publisher(
                "/hmi/threejs/tracks", String, queue_size=2,
            )
        else:
            self._pub_tracks = None
        self._mapdir = rospy.get_param(
            "~mapdir",
            os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                "..", "..", "..", "localization", "gps_system_localizer",
                "mapfiles", "K_CITY_2025",
            ),
        )
        # Pre-extracted route JSON (extract_route.py output). Read-only at
        # runtime; latched once on /hmi/threejs/route.
        self._route_json = rospy.get_param(
            "~route_json",
            os.path.join(os.path.dirname(os.path.realpath(__file__)),
                         "route_senario_20260623.json"),
        )
        self._tracks_count = 0
        self._with_points_count = 0
        self._cloud_xyz = None  # (N,3) float32, latest /fusion_lidar_points
        self._cloud_n = 0
        # Last ego pose at percept emit time. Tracks are produced in ego
        # frame, so the frontend must transform them with the *same* ego
        # pose used at emission — not the live 50 Hz /hmi/ego_pose. Otherwise
        # tracks slide ~1 m every perception tick (10 Hz) as ego drifts in
        # between. We snapshot the latest /localization/to_control_team here
        # and include it in each /hmi/threejs/tracks payload.
        self._last_ego = None  # (east, north, yaw) or None until first cb
        self._publish_map_once()
        self._publish_route_once()

        if self._publish_tracks:
            if to_control_team_from_local_msg is not None:
                self._sub_local = rospy.Subscriber(
                    "/localization/to_control_team",
                    to_control_team_from_local_msg,
                    self._on_local, queue_size=4,
                )

            if RsPerceptionMsg is None:
                rospy.logerr(
                    "web_hmi_threejs_bridge: RsPerceptionMsg unavailable; "
                    "/hmi/threejs/tracks will not be published"
                )
            else:
                self._sub_cloud = rospy.Subscriber(
                    "/fusion_lidar_points", PointCloud2,
                    self._on_cloud, queue_size=1, buff_size=2 ** 26,
                )
                self._sub_percept = rospy.Subscriber(
                    "/percept_topic", RsPerceptionMsg,
                    self._on_percept, queue_size=1, buff_size=2 ** 24,
                )
                rospy.Timer(rospy.Duration(5.0), self._log_publish_rate)
        else:
            rospy.loginfo(
                "web_hmi_threejs_bridge: ~publish_tracks=false — "
                "/hmi/threejs/tracks is owned by web_hmi_threejs_tracks_cpp"
            )

    def _on_local(self, msg):
        self._last_ego = (
            float(msg.host_east), float(msg.host_north), float(msg.host_yaw),
        )

    def _on_cloud(self, msg):
        """Decode PointCloud2 → cached (N,3) float32 array.

        fields: x@0,y@4,z@8 (Float32); point_step=32. Reading the whole
        buffer as float32 yields 8 floats per point, of which the first 3
        are xyz. We avoid copying the full array by returning a view.
        """
        try:
            n = msg.width * msg.height
            if n == 0 or msg.point_step < 12:
                return
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            # reshape into (n, point_step), then slice xyz bytes contiguously
            stride = msg.point_step
            if buf.size != n * stride:
                return
            arr = buf.reshape(n, stride)[:, 0:12].copy()
            self._cloud_xyz = np.frombuffer(arr, dtype=np.float32).reshape(n, 3)
            self._cloud_n = n
        except (ValueError, AttributeError):
            pass

    def _on_percept(self, msg):
        """Build /hmi/threejs/tracks from /percept_topic + cached cloud."""
        try:
            lf = msg.lidarframe
            stamp = float(lf.timestamp.data)
        except AttributeError:
            return

        cloud = self._cloud_xyz
        n_cloud = self._cloud_n
        candidates = []
        for obj in lf.objects.objects:
            try:
                ci = obj.coreinfo
                tid = int(ci.trakcer_id.data)
            except AttributeError:
                continue
            try:
                conf = float(ci.exist_confidence.data)
                if conf < PERCEPT_MIN_CONFIDENCE:
                    continue
                type_int = int(ci.type.data)
                t = {
                    "id":     tid,
                    "tid":    tid,
                    "type":   percept_type_str(type_int),
                    "x":      float(ci.center.x.data),
                    "y":      float(ci.center.y.data),
                    "vx":     float(ci.velocity.x.data),
                    "vy":     float(ci.velocity.y.data),
                    "size_x": float(ci.size.x.data),
                    "size_y": float(ci.size.y.data),
                    "orientation": math.atan2(
                        float(ci.direction.y.data),
                        float(ci.direction.x.data),
                    ),
                    "confidence": conf,
                }
            except AttributeError:
                continue
            candidates.append((t, obj))

        # Sort by ego-frame distance (nearest first) so the frontend renders
        # close objects first. Upstream percept_topic_matcher caps at 14, so
        # the survivor set stays bounded.
        candidates.sort(key=lambda p: p[0]["x"] * p[0]["x"] + p[0]["y"] * p[0]["y"])

        tracks = []
        with_points = 0
        for t, obj in candidates:
            if cloud is not None and obj.hassupplmentinfo.data:
                indices = obj.supplementinfo.cloud_indices
                if indices:
                    pts = self._slice_points(cloud, indices, n_cloud,
                                             PERCEPT_MAX_POINTS_PER_TRACK)
                    if pts:
                        t["points"] = pts
                        with_points += 1
            tracks.append(t)

        self._tracks_count += len(tracks)
        self._with_points_count += with_points

        # Pair every emit with the ego pose used to produce these ego-frame
        # coords, so the frontend transforms tracks with a snapshot that
        # matches them (vs. the live 50 Hz pose, which drifts ~1 m/tick).
        body = {"stamp": stamp, "tracks": tracks}
        if self._last_ego is not None:
            e, n, y = self._last_ego
            body["ego_at_emit"] = {"east": e, "north": n, "yaw": y}
        payload = json.dumps(body, ensure_ascii=False, separators=(",", ":"))
        self._pub_tracks.publish(String(data=payload))

    @staticmethod
    def _slice_points(cloud_xyz, indices, n_cloud, cap):
        """Slice cached (N,3) cloud by std_msgs/Int32[] indices.

        Returns flat [x0,y0,z0, x1,y1,z1, ...] (Python list of floats,
        rounded to mm). Drops out-of-range indices. Caps at `cap` points.
        """
        n_idx = len(indices)
        if n_idx == 0:
            return []
        # Cap BEFORE iterating: cloud_indices can be tens of thousands long
        # per track, and std_msgs/Int32 .data attribute access is the
        # dominant per-callback cost. Capping first keeps the loop bounded
        # regardless of input size.
        n_take = n_idx if n_idx <= cap else cap
        sliced = indices[:n_take]
        first = sliced[0]
        if hasattr(first, "data"):
            raw = [int(s.data) for s in sliced]
        else:
            raw = [int(s) for s in sliced]
        idx_arr = np.asarray(raw, dtype=np.int64)
        valid = (idx_arr >= 0) & (idx_arr < n_cloud)
        if not valid.any():
            return []
        idx_arr = idx_arr[valid]
        pts = cloud_xyz[idx_arr]
        # Drop NaN/Inf rows: JSON spec rejects NaN tokens and Three.js
        # BufferGeometry breaks on non-finite vertices.
        finite = np.isfinite(pts).all(axis=1)
        if not finite.any():
            return []
        pts = pts[finite]
        # rosbridge JSON: round to 3 decimals (mm) to shrink payload.
        pts = np.round(pts, 3)
        return pts.reshape(-1).tolist()

    def _log_publish_rate(self, _evt):
        if self._tracks_count == 0:
            return
        rate = 100.0 * self._with_points_count / max(1, self._tracks_count)
        rospy.loginfo(
            "web_hmi_threejs_bridge: %d tracks last 5s, %d (%.0f%%) with points",
            self._tracks_count, self._with_points_count, rate,
        )
        self._tracks_count = 0
        self._with_points_count = 0

    def _publish_map_once(self):
        if shapefile is None:
            rospy.logerr("web_hmi_threejs_bridge: pyshp not installed")
            return
        if Transformer is None:
            rospy.logerr("web_hmi_threejs_bridge: pyproj not installed")
            return
        if not os.path.isdir(self._mapdir):
            rospy.logerr("web_hmi_threejs_bridge: mapdir not found: %s",
                         self._mapdir)
            return

        tx = Transformer.from_crs(SRC_EPSG, DST_EPSG, always_xy=True)

        layers = {}
        for layer_name, kind in LAYERS_ALL.items():
            shp_path = os.path.join(self._mapdir, layer_name + ".shp")
            if not os.path.isfile(shp_path):
                rospy.logwarn("web_hmi_threejs_bridge: missing %s", shp_path)
                continue
            if kind == "polyline":
                data = self._load_polylines(shp_path, tx)
            elif kind == "polygon":
                data = self._load_polygons(shp_path, tx)
            elif kind == "point":
                data = self._load_points(shp_path, tx)
            else:
                rospy.logwarn(
                    "web_hmi_threejs_bridge: unknown kind %s for %s",
                    kind, layer_name,
                )
                continue
            layers[layer_name] = {"kind": kind, "data": data}

        origin = self._compute_origin(layers)
        layers = self._shift_origin(layers, origin)

        payload = json.dumps(
            {"epsg": 5179, "origin": origin, "layers": layers},
            ensure_ascii=False, separators=(",", ":"),
        )
        self._pub_map.publish(String(data=payload))
        feats = {n: len(v["data"]) for n, v in layers.items()}
        rospy.loginfo(
            "web_hmi_threejs_bridge: /hmi/threejs/map latched — "
            "%d layers, %d KB. Features: %s",
            len(layers), len(payload) // 1024, feats,
        )

    def _publish_route_once(self):
        """Latch pre-extracted route (EPSG:5179 abs coords) on /hmi/threejs/route."""
        if not os.path.isfile(self._route_json):
            rospy.logwarn("web_hmi_threejs_bridge: route_json not found: %s",
                          self._route_json)
            return
        try:
            with open(self._route_json, "r", encoding="utf-8") as f:
                data = f.read()
            json.loads(data)  # validate
            self._pub_route.publish(String(data=data))
            rospy.loginfo(
                "web_hmi_threejs_bridge: /hmi/threejs/route latched (%d B)",
                len(data),
            )
        except (OSError, ValueError) as e:
            rospy.logerr("web_hmi_threejs_bridge: route load failed: %s", e)

    @staticmethod
    def _xform_xy(tx, pts):
        out = []
        for x, y in pts:
            e, n = tx.transform(x, y)
            out.append([round(e, 2), round(n, 2)])
        return out

    @classmethod
    def _load_polylines(cls, shp_path, tx):
        """Each shape's parts split into separate polylines."""
        sf = shapefile.Reader(shp_path)
        out = []
        for shp in sf.shapes():
            if not shp.points:
                continue
            parts = list(shp.parts) if shp.parts else [0]
            parts.append(len(shp.points))
            for i in range(len(parts) - 1):
                seg = shp.points[parts[i]:parts[i + 1]]
                line = cls._xform_xy(tx, seg)
                if len(line) >= 2:
                    out.append(line)
        return out

    @classmethod
    def _load_polygons(cls, shp_path, tx):
        """Outer ring only (first part) per polygon shape."""
        sf = shapefile.Reader(shp_path)
        out = []
        for shp in sf.shapes():
            if not shp.points:
                continue
            parts = list(shp.parts) if shp.parts else [0]
            end = parts[1] if len(parts) > 1 else len(shp.points)
            ring = shp.points[parts[0]:end]
            poly = cls._xform_xy(tx, ring)
            if len(poly) >= 3:
                out.append(poly)
        return out

    @classmethod
    def _load_points(cls, shp_path, tx):
        sf = shapefile.Reader(shp_path)
        out = []
        for shp in sf.shapes():
            if not shp.points:
                continue
            x, y = shp.points[0]
            e, n = tx.transform(x, y)
            out.append([round(e, 2), round(n, 2)])
        return out

    @staticmethod
    def _compute_origin(layers):
        """Return [east0, north0] = mean of one vertex per feature."""
        es, ns = [], []
        for layer in layers.values():
            kind = layer["kind"]
            for feat in layer["data"]:
                if kind == "point":
                    es.append(feat[0])
                    ns.append(feat[1])
                elif feat:
                    es.append(feat[0][0])
                    ns.append(feat[0][1])
        if not es:
            return [0.0, 0.0]
        return [round(sum(es) / len(es), 2), round(sum(ns) / len(ns), 2)]

    @staticmethod
    def _shift_origin(layers, origin):
        e0, n0 = origin
        out = {}
        for name, layer in layers.items():
            kind = layer["kind"]
            shifted = []
            for feat in layer["data"]:
                if kind == "point":
                    shifted.append([round(feat[0] - e0, 2),
                                    round(feat[1] - n0, 2)])
                else:
                    shifted.append(
                        [[round(p[0] - e0, 2), round(p[1] - n0, 2)]
                         for p in feat]
                    )
            out[name] = {"kind": kind, "data": shifted}
        return out


def main():
    rospy.init_node("web_hmi_threejs_bridge", disable_signals=False)
    WebHmiThreejsBridge()
    rospy.spin()


if __name__ == "__main__":
    main()

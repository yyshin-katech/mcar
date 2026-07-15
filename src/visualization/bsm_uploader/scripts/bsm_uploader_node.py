#!/usr/bin/env python3
"""BSM uploader node (spec §6).

Subscribes to /sensors/gps/inspva (NovAtel Inspva) and /sensors/v_can
(katech_custom_msgs/v_can_msg), aggregates the latest cached state on a
10 Hz timer, serializes to JSON (spec §3.1) and forwards over WebSocket
to the EC2 tracker server.
"""

import json
import math
import threading
import time

import rospy

from novatel_gps_msgs.msg import Inspva
from katech_custom_msgs.msg import v_can_msg

try:
    import websocket  # pip: websocket-client
except ImportError as exc:  # pragma: no cover - import-time hint
    raise SystemExit(
        "Missing python websocket-client. "
        "Install with: sudo apt install python3-websocket"
    ) from exc


GEAR_MAP = {1: "P", 2: "R", 3: "N", 4: "D"}
TURN_SIGNAL_MAP = {0: "off", 1: "left", 2: "right", 3: "hazard"}


def _is_finite(x):
    try:
        return math.isfinite(float(x))
    except (TypeError, ValueError):
        return False


class WebSocketSender(threading.Thread):
    """Background sender (spec §6.6).

    rospy callbacks enqueue payloads (drop-newest-wins via _pending slot —
    we only care about the latest sample), the thread connects with
    exponential backoff and ws.send-s.
    """

    def __init__(self, url, reconnect_min_s=1.0, reconnect_max_s=30.0):
        super().__init__(daemon=True)
        self.url = url
        self._min_s = float(reconnect_min_s)
        self._max_s = float(reconnect_max_s)
        self._stop = threading.Event()
        self._cv = threading.Condition()
        self._pending = None  # latest serialized JSON string
        self._first_sent = False
        self._last_warn_ts = 0.0

    def submit(self, payload_str):
        with self._cv:
            self._pending = payload_str
            self._cv.notify()

    def stop(self):
        self._stop.set()
        with self._cv:
            self._cv.notify_all()

    def _warn_throttled(self, msg):
        # 2-second throttle (spec §6.6).
        now = time.monotonic()
        if now - self._last_warn_ts >= 2.0:
            rospy.logwarn(msg)
            self._last_warn_ts = now

    def run(self):
        backoff = self._min_s
        ws = None
        while not self._stop.is_set():
            # Connect.
            try:
                rospy.loginfo("[bsm_uploader] connecting to %s", self.url)
                ws = websocket.create_connection(self.url, timeout=5.0)
                rospy.loginfo("[bsm_uploader] connected")
                backoff = self._min_s
            except Exception as exc:  # noqa: BLE001
                self._warn_throttled(
                    "[bsm_uploader] connect failed (%s); retry in %.1fs"
                    % (exc, backoff)
                )
                self._stop.wait(timeout=backoff)
                backoff = min(self._max_s, max(self._min_s, backoff * 2.0))
                continue

            # Send loop.
            try:
                while not self._stop.is_set():
                    with self._cv:
                        # Wait until something to send (or stop).
                        while self._pending is None and not self._stop.is_set():
                            self._cv.wait(timeout=1.0)
                        if self._stop.is_set():
                            break
                        payload = self._pending
                        self._pending = None
                    if payload is None:
                        continue
                    ws.send(payload)
                    if not self._first_sent:
                        rospy.loginfo(
                            "[bsm_uploader] first send ok (url=%s, bytes=%d)",
                            self.url,
                            len(payload),
                        )
                        self._first_sent = True
            except Exception as exc:  # noqa: BLE001
                self._warn_throttled(
                    "[bsm_uploader] send/recv error (%s); reconnecting" % exc
                )
            finally:
                try:
                    if ws is not None:
                        ws.close()
                except Exception:  # noqa: BLE001
                    pass
                ws = None

            # Backoff before reconnect.
            if not self._stop.is_set():
                self._stop.wait(timeout=backoff)
                backoff = min(self._max_s, max(self._min_s, backoff * 2.0))


class BsmUploaderNode(object):
    def __init__(self):
        rospy.init_node("bsm_uploader_node", anonymous=False)

        self.server_url = rospy.get_param(
            "~server_url", "ws://13.209.88.22:8081/ws/ingest"
        )
        self.vehicle_id = rospy.get_param("~vehicle_id", "EV01")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 10.0))
        self.inspva_topic = rospy.get_param("~inspva_topic", "/sensors/gps/inspva")
        self.v_can_topic = rospy.get_param("~v_can_topic", "/sensors/v_can")
        self.reconnect_min_s = float(rospy.get_param("~reconnect_min_s", 1.0))
        self.reconnect_max_s = float(rospy.get_param("~reconnect_max_s", 30.0))
        self.drop_when_no_inspva = bool(
            rospy.get_param("~drop_when_no_inspva", True)
        )

        if self.publish_rate_hz <= 0:
            rospy.logwarn(
                "[bsm_uploader] invalid publish_rate_hz=%.3f; defaulting to 10.0",
                self.publish_rate_hz,
            )
            self.publish_rate_hz = 10.0

        self._lock = threading.Lock()
        self._latest_inspva = None
        self._latest_vcan = None

        self._sender = WebSocketSender(
            url=self.server_url,
            reconnect_min_s=self.reconnect_min_s,
            reconnect_max_s=self.reconnect_max_s,
        )
        self._sender.start()

        rospy.Subscriber(self.inspva_topic, Inspva, self._on_inspva, queue_size=10)
        rospy.Subscriber(self.v_can_topic, v_can_msg, self._on_v_can, queue_size=10)

        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self._on_timer
        )

        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo(
            "[bsm_uploader] started vehicle_id=%s rate=%.1fHz url=%s"
            " inspva=%s v_can=%s drop_when_no_inspva=%s",
            self.vehicle_id,
            self.publish_rate_hz,
            self.server_url,
            self.inspva_topic,
            self.v_can_topic,
            self.drop_when_no_inspva,
        )

    # ----- Callbacks ---------------------------------------------------------

    def _on_inspva(self, msg):
        with self._lock:
            self._latest_inspva = msg

    def _on_v_can(self, msg):
        with self._lock:
            self._latest_vcan = msg

    # ----- Timer -------------------------------------------------------------

    def _on_timer(self, _event):
        with self._lock:
            inspva = self._latest_inspva
            vcan = self._latest_vcan

        if inspva is None:
            if self.drop_when_no_inspva:
                return
            payload = self._build_payload_no_inspva(vcan)
        else:
            payload = self._build_payload(inspva, vcan)

        if payload is None:
            return
        try:
            text = json.dumps(payload, allow_nan=False)
        except ValueError as exc:
            rospy.logwarn_throttle(
                2.0, "[bsm_uploader] JSON serialize failed: %s" % exc
            )
            return
        self._sender.submit(text)

    # ----- Payload builders --------------------------------------------------

    def _build_payload(self, inspva, vcan):
        # INSPVA -> JSON units are degree / m / m/s already (spec §6.5).
        lat = float(inspva.latitude)
        lon = float(inspva.longitude)
        if not (_is_finite(lat) and _is_finite(lon)):
            rospy.logwarn_throttle(
                2.0, "[bsm_uploader] non-finite lat/lon, drop"
            )
            return None
        if lat < -90.0 or lat > 90.0 or lon < -180.0 or lon > 180.0:
            rospy.logwarn_throttle(
                2.0, "[bsm_uploader] lat/lon out of range, drop"
            )
            return None

        elev_m = float(inspva.height) if _is_finite(inspva.height) else 0.0
        vN = float(inspva.north_velocity) if _is_finite(inspva.north_velocity) else 0.0
        vE = float(inspva.east_velocity) if _is_finite(inspva.east_velocity) else 0.0
        speed_mps = math.sqrt(vN * vN + vE * vE)

        az = float(inspva.azimuth) if _is_finite(inspva.azimuth) else 0.0
        heading_deg = az % 360.0
        if not math.isfinite(heading_deg):
            heading_deg = 0.0

        payload = {
            "type": "bsm",
            "vehicle_id": self.vehicle_id,
            "ts_unix_ms": int(time.time() * 1000),
            "lat": lat,
            "lon": lon,
            "elev_m": elev_m,
            "heading_deg": heading_deg,
            "speed_mps": speed_mps,
        }
        payload.update(self._vcan_fields(vcan))
        return payload

    def _build_payload_no_inspva(self, vcan):
        # All location fields null (spec §6.4 drop_when_no_inspva=false).
        payload = {
            "type": "bsm",
            "vehicle_id": self.vehicle_id,
            "ts_unix_ms": int(time.time() * 1000),
            "lat": None,
            "lon": None,
            "elev_m": None,
            "heading_deg": None,
            "speed_mps": None,
        }
        payload.update(self._vcan_fields(vcan))
        return payload

    def _vcan_fields(self, vcan):
        if vcan is None:
            return {
                "gear": "?",
                "steering_deg": None,
                "accel_long_mps2": None,
                "accel_lat_mps2": None,
                "yaw_rate_dps": None,
                "brake_applied": None,
                "turn_signal": "?",
            }
        gear_status = int(getattr(vcan, "gear_status", 0))
        ts_status = int(getattr(vcan, "turn_signal_status", -1))
        brake = getattr(vcan, "brake_pedal_pos", None)
        try:
            brake_val = float(brake) if brake is not None else None
        except (TypeError, ValueError):
            brake_val = None
        return {
            "gear": GEAR_MAP.get(gear_status, "?"),
            "steering_deg": _safe_float(getattr(vcan, "steering_angle", None)),
            "accel_long_mps2": _safe_float(getattr(vcan, "long_acceleration", None)),
            "accel_lat_mps2": _safe_float(getattr(vcan, "lat_acceleration", None)),
            "yaw_rate_dps": _safe_float(getattr(vcan, "yaw_rate", None)),
            "brake_applied": (brake_val is not None and brake_val > 0.05)
                              if brake_val is not None else None,
            "turn_signal": TURN_SIGNAL_MAP.get(ts_status, "?"),
        }

    # ----- Shutdown ----------------------------------------------------------

    def _on_shutdown(self):
        rospy.loginfo("[bsm_uploader] shutting down")
        self._sender.stop()


def _safe_float(x):
    if x is None:
        return None
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(v):
        return None
    return v


def main():
    BsmUploaderNode()
    rospy.spin()


if __name__ == "__main__":
    main()

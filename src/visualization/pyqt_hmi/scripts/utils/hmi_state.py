# -*- coding: utf-8 -*-
"""HMI state controller — owns ROS subscriptions and emits framework signals.

Two layers:
- ``BaseHmiStateController`` — Qt-free. All ROS callbacks, diagnostic
  debounce, popup decision, bag subprocess, mode pulse logic. Subclasses
  plug in ``_emit(name, *args)``, ``_start_tick(period_ms, callback)``,
  ``_schedule_mode_pulse_reset(ms)``.
- ``HmiStateController(BaseHmiStateController, QObject)`` — thin Qt adapter
  exposing the original 16 ``pyqtSignal`` s. Sole consumer is
  ``MainWindowA1`` — its ``_wire_controller`` keeps working unchanged.

The web bridge (``web_hmi_bridge.py``) uses the Base directly with a
rospy-Timer based scheduler.
"""
import datetime
import os
import re
import signal
import subprocess

import rospy
from PyQt5.QtCore import QObject, QTimer, pyqtSignal

from std_msgs.msg import UInt8
from katech_diagnostic_msgs.msg import (cam_diagnostic_msg,
                                        cpt7_gps_diagnostic_msg,
                                        hmi_diagnostic_msg,
                                        ipc_diagnostic_msg,
                                        k_adcu_diagnostic_msg,
                                        lidar_diagnostic_msg,
                                        radar_diagnostic_msg,
                                        v2x_diagnostic_msg,
                                        vcu_diagnostic_msg)
from katech_custom_msgs.msg import ioniq5_ad_can_msg, v_can_msg
from mmc_msgs.msg import chassis_msg, to_control_team_from_local_msg
from v2x_msgs.msg import intersection_array_msg
from perception_ros_msg.msg import object_array_msg


GPS_STD_WARN_M = 0.05  # 5 cm precision threshold (mirrors legacy)
DIAG_MISS_THRESHOLD = 10  # 10 ticks × 100 ms = 1 s
DIAG_MISS_THRESHOLD_V2X = 30  # 30 ticks × 100 ms = 3 s (V2X is slower)


class BaseHmiStateController:
    """Framework-agnostic HMI state. Subclass to plug in emit + timers."""

    DIAG_KEYS = ['gps', 'adcu', 'lidar', 'radar', 'v2x', 'hmi', 'vcu', 'cam', 'ipc']

    SIGNAL_NAMES = (
        'speed_changed', 'gear_changed', 'mode_changed', 'aeb_changed',
        'steering_changed', 'ego_pose_changed', 'objects_changed',
        'traffic_changed', 'diag_changed', 'gps_changed',
        'speed_limit_changed', 'link_lane_changed', 'odd_changed',
        'popup_changed', 'bag_state_changed', 'topic_event',
    )

    def __init__(self):
        # diag state
        self.diag_status = {k: 2 for k in self.DIAG_KEYS}
        self.diag_flags = {k: {'received': False, 'miss_cnt': 0} for k in self.DIAG_KEYS}

        # vehicle state
        self.current_speed = 0.0
        self.gear_status = 0
        self.autonomous_mode = 0
        self.aeb_flag = 0
        self.steering_angle = 0.0

        # GPS / localization
        self.gps_rtk_code = 0
        self.gps_lon_std = 0.0
        self.gps_lat_std = 0.0
        self.adcu_swc_code = 0
        self.lidar_center_code = 0
        self.lidar_right_code = 0
        self.lidar_left_code = 0
        self.v2x_stat_code = 0
        self.vcu_stat_code = 0
        self.ipc_swc_code = 0
        self.speed_limit = 0
        self.link_id = 0
        self.lane_label = ""
        self.on_odd = 0
        self.road_state = 0

        # ego pose
        self.host_east = 0.0
        self.host_north = 0.0
        self.host_yaw = 0.0

        # traffic
        self.look_at_intersection_id = 0
        self.look_at_signal_group_id = 0
        self.traffic_light_color = 0
        self.traffic_light_time = 0

        # objects
        self.objects = []

        # bag
        self.bag_process = None
        self.bag_recording = False
        self.bag_info = ""
        self.bag_dir = os.path.expanduser("~/bag_data")
        # LiDAR/인지 토픽 포함 여부 (True: 전체 -a 저장, False: 아래 토픽 제외).
        # 용량 큰 LiDAR raw packet / 인지 rviz 토픽 13개.
        self.bag_include_lidar = True
        self.optional_record_topics = [
            "/left/rslidar_packets_difop",
            "/middle/rslidar_packets",
            "/middle/rslidar_packets_difop",
            "/percept_background_rviz",
            "/percept_cluster_rviz",
            "/percept_ground_rviz",
            "/percept_non_ground_rviz",
            "/percept_origin_rviz",
            "/percept_sematic_rviz",
            "/percept_topic",
            "/perception_info_rviz",
            "/perception_pre_known_rviz",
            "/right/rslidar_packets_difop",
        ]

        # mode publisher + pulse
        self.selected_mode = 0
        self._mode_pub = rospy.Publisher('/vehicle/mode_command', UInt8, queue_size=1)

        # subscribe
        self._init_subscribers()

        # 100 ms tick — diag debounce, popup, mode publish
        self._start_tick(100, self._periodic_update)

    # ─── hooks: subclass MUST implement ──────────────────────────
    def _emit(self, name, *args):  # pragma: no cover - abstract
        raise NotImplementedError

    def _start_tick(self, period_ms, callback):  # pragma: no cover
        raise NotImplementedError

    def _schedule_mode_pulse_reset(self, ms):  # pragma: no cover
        """Schedule one-shot reset of selected_mode after `ms`. Cancels any prior."""
        raise NotImplementedError

    # ─── ROS subscribers ─────────────────────────────────────────
    def _init_subscribers(self):
        rospy.Subscriber("/diagnostic/cpt7_gps", cpt7_gps_diagnostic_msg, self._cb_gps)
        rospy.Subscriber("/diagnostic/adcu", k_adcu_diagnostic_msg, self._cb_adcu)
        rospy.Subscriber("/diagnostic/lidar", lidar_diagnostic_msg, self._cb_lidar)
        rospy.Subscriber("/diagnostic/radar", radar_diagnostic_msg, self._cb_radar)
        rospy.Subscriber("/diagnostic/v2x", v2x_diagnostic_msg, self._cb_v2x)
        rospy.Subscriber("/diagnostic/hmi", hmi_diagnostic_msg, self._cb_hmi)
        rospy.Subscriber("/diagnostic/vcu", vcu_diagnostic_msg, self._cb_vcu)
        rospy.Subscriber("/diagnostic/cam", cam_diagnostic_msg, self._cb_cam)
        rospy.Subscriber("/diagnostic/ipc", ipc_diagnostic_msg, self._cb_ipc)
        rospy.Subscriber("/sensors/chassis", chassis_msg, self._cb_chassis)
        rospy.Subscriber("/sensors/ioniq5_ad_can", ioniq5_ad_can_msg, self._cb_ad_can)
        rospy.Subscriber("/sensors/v_can", v_can_msg, self._cb_v_can)
        rospy.Subscriber("/localization/to_control_team",
                         to_control_team_from_local_msg, self._cb_local)
        rospy.Subscriber("/spat_merged",
                         intersection_array_msg, self._cb_traffic)
        rospy.Subscriber("/track_Multi_RS", object_array_msg, self._cb_objects)

    # ─── diagnostic callbacks ────────────────────────────────────
    def _cb_gps(self, msg):
        self.diag_flags['gps']['received'] = True
        self.gps_rtk_code = msg.GPSRTK_StatCode
        self.gps_lon_std = msg.lon_std
        self.gps_lat_std = msg.lat_std
        self._emit('gps_changed', self.gps_rtk_code, self.gps_lon_std, self.gps_lat_std)
        self._emit('topic_event', 'gps')

    def _cb_adcu(self, msg):
        self.diag_flags['adcu']['received'] = True
        self.adcu_swc_code = msg.ADCU_SWC_StatCode
        self._emit('topic_event', 'adcu')

    def _cb_lidar(self, msg):
        self.diag_flags['lidar']['received'] = True
        self.lidar_center_code = msg.LIDAR_Center_StatCode
        self.lidar_right_code = msg.LIDAR_Right_StatCode
        self.lidar_left_code = msg.LIDAR_Left_StatCode
        self._emit('topic_event', 'lidar')

    def _cb_radar(self, _msg):
        self.diag_flags['radar']['received'] = True
        self._emit('topic_event', 'radar')

    def _cb_v2x(self, msg):
        self.diag_flags['v2x']['received'] = True
        self.v2x_stat_code = msg.V2X_StatCode
        self._emit('topic_event', 'v2x')

    def _cb_hmi(self, _msg):
        self.diag_flags['hmi']['received'] = True
        self._emit('topic_event', 'hmi')

    def _cb_vcu(self, msg):
        self.diag_flags['vcu']['received'] = True
        self.vcu_stat_code = msg.VCU_StatCode
        self._emit('topic_event', 'vcu')

    def _cb_cam(self, _msg):
        self.diag_flags['cam']['received'] = True
        self._emit('topic_event', 'cam')

    def _cb_ipc(self, msg):
        self.diag_flags['ipc']['received'] = True
        self.ipc_swc_code = msg.IPC_SWC_StatCode
        self._emit('topic_event', 'ipc')

    # ─── vehicle / sensors callbacks ─────────────────────────────
    def _cb_ad_can(self, msg):
        if msg.autonomous_mode != self.autonomous_mode:
            self.autonomous_mode = msg.autonomous_mode
            self._emit('mode_changed', self.autonomous_mode)

    def _cb_v_can(self, msg):
        self.steering_angle = msg.steering_angle
        self._emit('steering_changed', float(msg.steering_angle))
        avg = (msg.wheel_speed_fl + msg.wheel_speed_fr +
               msg.wheel_speed_rl + msg.wheel_speed_rr) / 4.0
        self.current_speed = avg * 3.6
        self._emit('speed_changed', float(self.current_speed))
        if msg.gear_status != self.gear_status:
            self.gear_status = msg.gear_status
            self._emit('gear_changed', int(self.gear_status))

    # DBC Curr_gear → HMI gear code mapping
    _GEAR_MAP = {0: 1, 5: 4, 6: 3, 7: 2}  # 0=P→1, 5=D→4, 6=N→3, 7=R→2

    def _cb_chassis(self, msg):
        # speed (km/h)
        self.current_speed = float(msg.vcu_VS)
        self._emit('speed_changed', self.current_speed)
        # steering angle
        self.steering_angle = -float(msg.vcu_SAS_Angle)
        self._emit('steering_changed', self.steering_angle)
        # gear (DBC: 0=P,5=D,6=N,7=R → HMI: 1=P,2=R,3=N,4=D)
        new_gear = self._GEAR_MAP.get(int(getattr(msg, 'Curr_gear', 0)), 0)
        if new_gear != self.gear_status:
            self.gear_status = new_gear
            self._emit('gear_changed', self.gear_status)
        # autonomous mode
        new_mode = int(msg.vcu_ADMDStatus)
        if new_mode != self.autonomous_mode:
            self.autonomous_mode = new_mode
            self._emit('mode_changed', self.autonomous_mode)
        # AEB
        new_aeb = bool(getattr(msg, 'AEB_flag', 0))
        if new_aeb != bool(self.aeb_flag):
            self.aeb_flag = 1 if new_aeb else 0
            self._emit('aeb_changed', new_aeb)

    def _cb_local(self, msg):
        if msg.Speed_Limit != self.speed_limit:
            self.speed_limit = msg.Speed_Limit
            self._emit('speed_limit_changed', int(self.speed_limit))
        if msg.LINK_ID != self.link_id:
            self.link_id = msg.LINK_ID
            lane = getattr(msg, 'lane_name', None) or getattr(msg, 'lane_id', None) or ''
            self.lane_label = lane
            self._emit('link_lane_changed', self.link_id, self.lane_label)
        if msg.On_ODD != self.on_odd or msg.Road_State != self.road_state:
            self.on_odd = msg.On_ODD
            self.road_state = msg.Road_State
            self._emit('odd_changed', int(self.on_odd), int(self.road_state))

        self.look_at_intersection_id = msg.look_at_IntersectionID
        self.look_at_signal_group_id = msg.look_at_signalGroupID

        self.host_east = float(msg.host_east)
        self.host_north = float(msg.host_north)
        self.host_yaw = float(msg.host_yaw)
        self._emit('ego_pose_changed', self.host_east, self.host_north, self.host_yaw)

    def _cb_traffic(self, msg):
        if self.look_at_intersection_id == 0:
            if self.traffic_light_color != 0 or self.traffic_light_time != 0:
                self.traffic_light_color = 0
                self.traffic_light_time = 0
                self._emit('traffic_changed', 0, 0)
            return
        for intersection in msg.data:
            if intersection.IntersectionID != self.look_at_intersection_id:
                continue
            movement = intersection.Movements
            if (self.look_at_signal_group_id != 0 and
                    movement.SignalGroupID != self.look_at_signal_group_id):
                continue
            self.traffic_light_time = movement.TimeChangeDetails
            phase = movement.MovementPhaseStatus
            # SAE J2735 MovementPhaseState → display color code.
            # Mapping aligned with stat_display.cpp (case 3/5/6/7/8).
            #   color 1 = GREEN, 2 = AMBER, 3 = RED.
            if phase == 5 or phase == 6:     # permissive/protected movement allowed
                color = 1
            elif phase == 7 or phase == 8:   # permissive/protected clearance
                color = 2
            elif phase == 3:                 # stop_And_Remain
                color = 3
            else:
                color = 0
            self.traffic_light_color = color
            self._emit('traffic_changed', int(color), int(self.traffic_light_time))
            return

    def _cb_objects(self, msg):
        objects = []
        for obj in msg.data:
            obj_type = 'pedestrian' if obj.status == 1 else 'car'
            objects.append({
                'id': obj.id,
                'x': obj.x,
                'y': obj.y,
                'width': obj.size_y if obj.size_y > 0 else 1.0,
                'length': obj.size_x if obj.size_x > 0 else 1.0,
                'vx': obj.vx,
                'vy': obj.vy,
                'orientation': obj.orientation,
                'type': obj_type,
            })
        self.objects = objects
        self._emit('objects_changed', objects)

    # ─── periodic update — diagnostic + popup ─────────────────────
    def _evaluate_diag(self, name, received):
        d = self.diag_flags[name]
        threshold = DIAG_MISS_THRESHOLD_V2X if name == 'v2x' else DIAG_MISS_THRESHOLD
        if received:
            d['miss_cnt'] = 0
        else:
            d['miss_cnt'] += 1
            if d['miss_cnt'] > threshold:
                return 2

        if name == 'gps':
            if self.gps_rtk_code < 2:
                return 1
            if self.gps_lon_std > GPS_STD_WARN_M or self.gps_lat_std > GPS_STD_WARN_M:
                return 1
        elif name == 'lidar':
            if 1 in (self.lidar_center_code, self.lidar_right_code, self.lidar_left_code):
                return 1
        # V2X: StatCode==1 is transient, ignore it (only flag persistent errors)
        elif name == 'v2x' and self.v2x_stat_code >= 2:
            return 1
        elif name == 'vcu' and self.vcu_stat_code == 1:
            return 1
        elif name == 'ipc' and self.ipc_swc_code == 1:
            return 1
        elif name == 'adcu' and self.adcu_swc_code == 1:
            return 1
        return 0

    def _periodic_update(self):
        # diagnostics
        new_status = {}
        changed = False
        for name, d in self.diag_flags.items():
            received = d['received']
            d['received'] = False
            new = self._evaluate_diag(name, received)
            new_status[name] = new
            if self.diag_status.get(name) != new:
                changed = True
            self.diag_status[name] = new
        if changed:
            self._emit('diag_changed', dict(self.diag_status))

        # selected mode publish (1 s pulse semantics maintained in pulse method)
        m = UInt8()
        m.data = self.selected_mode
        self._mode_pub.publish(m)

        # popup decision (mirrors legacy update_popup)
        self._publish_popup()

    def _publish_popup(self):
        diag_label_map = {'gps': 'GPS', 'adcu': 'ADCU', 'lidar': 'LiDAR',
                          'radar': 'Radar', 'v2x': 'V2X', 'hmi': 'HMI',
                          'cam': 'CAM', 'vcu': 'VCU', 'ipc': 'IPC'}
        abnormal = [diag_label_map[k] for k, v in self.diag_status.items() if v != 0]
        n = len(abnormal)

        if n == 1:
            text = f"{abnormal[0]} 센서 고장"
            sev = "warn"
        elif n >= 2:
            text = f"시스템 고장 ({n}개 시스템 오류)"
            sev = "error"
        elif self.road_state == 1:
            text = "전방 ODD 이탈 경고"
            sev = "warn"
        elif self.aeb_flag == 1:
            text = "전방 추돌 경고"
            sev = "error"
        elif self.on_odd == 1:
            text = "ODD 이탈 !!!!"
            sev = "error"
        else:
            text = ""
            sev = "info"
        self._emit('popup_changed', text, sev)

    # ─── mode request (public API) ───────────────────────────────
    def request_mode(self, autonomous):
        if autonomous:
            self.selected_mode = 1
            rospy.loginfo("HMI: mode change → Autonomous (1s pulse)")
            self._schedule_mode_pulse_reset(1000)
        else:
            self.selected_mode = 0
            rospy.loginfo("HMI: mode change → Manual")

    def _reset_mode_pulse(self):
        self.selected_mode = 0

    # ─── bag recording (public API) ──────────────────────────────
    def set_bag_dir(self, path):
        self.bag_dir = path or os.path.expanduser("~/bag_data")

    def set_bag_include_lidar(self, flag):
        self.bag_include_lidar = bool(flag)

    def toggle_bag(self):
        if self.bag_recording:
            self.stop_bag()
        else:
            self.start_bag()

    def start_bag(self):
        if self.bag_recording:
            return
        os.makedirs(self.bag_dir, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        prefix = os.path.join(self.bag_dir, ts)
        cmd = ["rosbag", "record", "-a"]
        # 토글 OFF면 LiDAR/인지 토픽을 제외 (-x 정규식, $ 앵커로 정확 매칭).
        if not self.bag_include_lidar:
            exclude_regex = "(" + "|".join(
                re.escape(t) + "$" for t in self.optional_record_topics) + ")"
            cmd += ["-x", exclude_regex]
        cmd += ["--split", "--size=10240", "-o", prefix]
        self.bag_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.bag_recording = True
        self.bag_info = ts
        rospy.loginfo("HMI: bag recording started: %s", prefix)
        self._emit('bag_state_changed', True, ts)

    def stop_bag(self):
        if self.bag_process:
            try:
                os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
                self.bag_process.wait(timeout=5)
            except Exception as e:  # noqa: BLE001
                rospy.logwarn("HMI: bag stop error: %s", e)
            self.bag_process = None
        self.bag_recording = False
        self.bag_info = ""
        rospy.loginfo("HMI: bag recording stopped")
        self._emit('bag_state_changed', False, "")

    def shutdown(self):
        try:
            self.stop_bag()
        except Exception:  # noqa: BLE001
            pass


class HmiStateController(BaseHmiStateController, QObject):
    """Qt adapter — the original 16 pyqtSignals are preserved verbatim."""

    speed_changed       = pyqtSignal(float)            # km/h
    gear_changed        = pyqtSignal(int)              # 1=P, 2=R, 3=N, 4=D
    mode_changed        = pyqtSignal(int)              # autonomous_mode 0/1
    aeb_changed         = pyqtSignal(bool)
    steering_changed    = pyqtSignal(float)            # deg
    ego_pose_changed    = pyqtSignal(float, float, float)  # east, north, yaw
    objects_changed     = pyqtSignal(list)
    traffic_changed     = pyqtSignal(int, int)         # color (0-3), time_decisec
    diag_changed        = pyqtSignal(dict)             # {name: status}
    gps_changed         = pyqtSignal(int, float, float)  # rtk_code, lon_std, lat_std
    speed_limit_changed = pyqtSignal(int)
    link_lane_changed   = pyqtSignal(object, object)   # link_id, lane label
    odd_changed         = pyqtSignal(int, int)         # on_odd, road_state
    popup_changed       = pyqtSignal(str, str)         # text, severity
    bag_state_changed   = pyqtSignal(bool, str)        # recording, info
    topic_event         = pyqtSignal(str)              # key, for Hz tracking

    def __init__(self, parent=None):
        QObject.__init__(self, parent)
        self._tick = None
        self._mode_pulse_timer = None
        BaseHmiStateController.__init__(self)

    def _emit(self, name, *args):
        getattr(self, name).emit(*args)

    def _start_tick(self, period_ms, callback):
        self._tick = QTimer(self)
        self._tick.timeout.connect(callback)
        self._tick.start(period_ms)

    def _schedule_mode_pulse_reset(self, ms):
        if self._mode_pulse_timer:
            self._mode_pulse_timer.stop()
        self._mode_pulse_timer = QTimer(self)
        self._mode_pulse_timer.setSingleShot(True)
        self._mode_pulse_timer.timeout.connect(self._reset_mode_pulse)
        self._mode_pulse_timer.start(ms)

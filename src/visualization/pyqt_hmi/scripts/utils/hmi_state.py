# -*- coding: utf-8 -*-
"""HmiStateController — owns ROS subscriptions and emits Qt signals.

Logic is copied from widgets/main_window.py to avoid touching the legacy
window. Both windows can share this controller in a follow-up refactor.
"""
import datetime
import os
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


class HmiStateController(QObject):
    """Single source of truth for the A-1 HMI runtime state."""

    # ─── signals (consumed by MainWindowA1) ──────────────────────
    speed_changed       = pyqtSignal(float)            # km/h
    gear_changed        = pyqtSignal(int)              # 1=P, 2=R, 3=N, 4=D
    mode_changed        = pyqtSignal(int)              # autonomous_mode 0/1
    aeb_changed         = pyqtSignal(bool)
    steering_changed    = pyqtSignal(float)            # deg
    ego_pose_changed    = pyqtSignal(float, float, float)  # east, north, yaw
    objects_changed     = pyqtSignal(list)
    traffic_changed     = pyqtSignal(int, int)         # color (0-3), time_decisec
    diag_changed        = pyqtSignal(dict)             # {name: status}
    gps_changed         = pyqtSignal(int, float, float) # rtk_code, lon_std, lat_std
    speed_limit_changed = pyqtSignal(int)
    link_lane_changed   = pyqtSignal(object, object)    # link_id, lane label
    odd_changed         = pyqtSignal(int, int)          # on_odd, road_state
    popup_changed       = pyqtSignal(str, str)          # text, severity
    bag_state_changed   = pyqtSignal(bool, str)         # recording, info
    topic_event         = pyqtSignal(str)               # key, for Hz tracking

    DIAG_KEYS = ['gps', 'adcu', 'lidar', 'radar', 'v2x', 'hmi', 'vcu', 'cam', 'ipc']

    def __init__(self, parent=None):
        super().__init__(parent)

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

        # traffic
        self.look_at_intersection_id = 0
        self.look_at_signal_group_id = 0
        self.traffic_light_color = 0
        self.traffic_light_time = 0

        # bag
        self.bag_process = None
        self.bag_recording = False
        self.bag_dir = os.path.expanduser("~/bag_data")

        # mode publisher
        self.selected_mode = 0
        self._mode_pub = rospy.Publisher('/vehicle/mode_command', UInt8, queue_size=1)
        self._mode_pulse_timer = None

        # subscribe
        self._init_subscribers()

        # 100 ms tick — diag debounce, popup, mode publish, traffic light
        self._tick = QTimer(self)
        self._tick.timeout.connect(self._periodic_update)
        self._tick.start(100)

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
        rospy.Subscriber("/katri_v2x_node/katri_spat",
                         intersection_array_msg, self._cb_traffic)
        rospy.Subscriber("/track_Multi_RS", object_array_msg, self._cb_objects)

    # ─── diagnostic callbacks ────────────────────────────────────
    def _cb_gps(self, msg):
        self.diag_flags['gps']['received'] = True
        self.gps_rtk_code = msg.GPSRTK_StatCode
        self.gps_lon_std = msg.lon_std
        self.gps_lat_std = msg.lat_std
        self.gps_changed.emit(self.gps_rtk_code, self.gps_lon_std, self.gps_lat_std)
        self.topic_event.emit('gps')

    def _cb_adcu(self, msg):
        self.diag_flags['adcu']['received'] = True
        self.adcu_swc_code = msg.ADCU_SWC_StatCode
        self.topic_event.emit('adcu')

    def _cb_lidar(self, msg):
        self.diag_flags['lidar']['received'] = True
        self.lidar_center_code = msg.LIDAR_Center_StatCode
        self.lidar_right_code = msg.LIDAR_Right_StatCode
        self.lidar_left_code = msg.LIDAR_Left_StatCode
        self.topic_event.emit('lidar')

    def _cb_radar(self, _msg):
        self.diag_flags['radar']['received'] = True
        self.topic_event.emit('radar')

    def _cb_v2x(self, msg):
        self.diag_flags['v2x']['received'] = True
        self.v2x_stat_code = msg.V2X_StatCode
        self.topic_event.emit('v2x')

    def _cb_hmi(self, _msg):
        self.diag_flags['hmi']['received'] = True
        self.topic_event.emit('hmi')

    def _cb_vcu(self, msg):
        self.diag_flags['vcu']['received'] = True
        self.vcu_stat_code = msg.VCU_StatCode
        self.topic_event.emit('vcu')

    def _cb_cam(self, _msg):
        self.diag_flags['cam']['received'] = True
        self.topic_event.emit('cam')

    def _cb_ipc(self, msg):
        self.diag_flags['ipc']['received'] = True
        self.ipc_swc_code = msg.IPC_SWC_StatCode
        self.topic_event.emit('ipc')

    # ─── vehicle / sensors callbacks ─────────────────────────────
    def _cb_ad_can(self, msg):
        if msg.autonomous_mode != self.autonomous_mode:
            self.autonomous_mode = msg.autonomous_mode
            self.mode_changed.emit(self.autonomous_mode)

    def _cb_v_can(self, msg):
        self.steering_angle = msg.steering_angle
        self.steering_changed.emit(float(msg.steering_angle))
        avg = (msg.wheel_speed_fl + msg.wheel_speed_fr +
               msg.wheel_speed_rl + msg.wheel_speed_rr) / 4.0
        self.current_speed = avg * 3.6
        self.speed_changed.emit(float(self.current_speed))
        if msg.gear_status != self.gear_status:
            self.gear_status = msg.gear_status
            self.gear_changed.emit(int(self.gear_status))

    def _cb_chassis(self, msg):
        # NOTE: legacy reads `vehicle_speed` via getattr (returns 0 if absent),
        # which would clobber the v_can path. We only honour AEB here so the
        # speed gauge stays driven by wheel speeds.
        new_aeb = bool(getattr(msg, 'AEB_flag', 0))
        if new_aeb != bool(self.aeb_flag):
            self.aeb_flag = 1 if new_aeb else 0
            self.aeb_changed.emit(new_aeb)

    def _cb_local(self, msg):
        if msg.Speed_Limit != self.speed_limit:
            self.speed_limit = msg.Speed_Limit
            self.speed_limit_changed.emit(int(self.speed_limit))
        if msg.LINK_ID != self.link_id:
            self.link_id = msg.LINK_ID
            lane = getattr(msg, 'lane_name', None) or getattr(msg, 'lane_id', None) or ''
            self.lane_label = lane
            self.link_lane_changed.emit(self.link_id, self.lane_label)
        if msg.On_ODD != self.on_odd or msg.Road_State != self.road_state:
            self.on_odd = msg.On_ODD
            self.road_state = msg.Road_State
            self.odd_changed.emit(int(self.on_odd), int(self.road_state))

        self.look_at_intersection_id = msg.look_at_IntersectionID
        self.look_at_signal_group_id = msg.look_at_signalGroupID

        self.ego_pose_changed.emit(float(msg.host_east),
                                   float(msg.host_north),
                                   float(msg.host_yaw))

    def _cb_traffic(self, msg):
        if self.look_at_intersection_id == 0:
            if self.traffic_light_color != 0 or self.traffic_light_time != 0:
                self.traffic_light_color = 0
                self.traffic_light_time = 0
                self.traffic_changed.emit(0, 0)
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
            if phase == 3:
                color = 1   # green
            elif phase == 8:
                color = 2   # amber
            elif phase == 6:
                color = 3   # red
            else:
                color = 0
            self.traffic_light_color = color
            self.traffic_changed.emit(int(color), int(self.traffic_light_time))
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
        self.objects_changed.emit(objects)

    # ─── periodic update — diagnostic + popup ─────────────────────
    def _evaluate_diag(self, name, received):
        d = self.diag_flags[name]
        if received:
            d['miss_cnt'] = 0
        else:
            d['miss_cnt'] += 1
            if d['miss_cnt'] > DIAG_MISS_THRESHOLD:
                return 2

        if name == 'gps':
            if self.gps_rtk_code < 2:
                return 1
            if self.gps_lon_std > GPS_STD_WARN_M or self.gps_lat_std > GPS_STD_WARN_M:
                return 1
        elif name == 'lidar':
            if 1 in (self.lidar_center_code, self.lidar_right_code, self.lidar_left_code):
                return 1
        elif name == 'v2x' and self.v2x_stat_code == 1:
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
            self.diag_changed.emit(dict(self.diag_status))

        # selected mode publish (1 s pulse semantics maintained in pulse method)
        m = UInt8(); m.data = self.selected_mode
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
        self.popup_changed.emit(text, sev)

    # ─── mode request (public API) ───────────────────────────────
    def request_mode(self, autonomous):
        if autonomous:
            self.selected_mode = 1
            rospy.loginfo("HMI A-1: mode change → Autonomous (1s pulse)")
            if self._mode_pulse_timer:
                self._mode_pulse_timer.stop()
            self._mode_pulse_timer = QTimer(self)
            self._mode_pulse_timer.setSingleShot(True)
            self._mode_pulse_timer.timeout.connect(self._reset_mode_pulse)
            self._mode_pulse_timer.start(1000)
        else:
            self.selected_mode = 0
            rospy.loginfo("HMI A-1: mode change → Manual")

    def _reset_mode_pulse(self):
        self.selected_mode = 0

    # ─── bag recording (public API) ──────────────────────────────
    def set_bag_dir(self, path):
        self.bag_dir = path or os.path.expanduser("~/bag_data")

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
        self.bag_process = subprocess.Popen(
            ["rosbag", "record", "-a", "--split", "--size=10240", "-o", prefix],
            preexec_fn=os.setsid,
        )
        self.bag_recording = True
        rospy.loginfo("HMI A-1: bag recording started: %s", prefix)
        self.bag_state_changed.emit(True, ts)

    def stop_bag(self):
        if self.bag_process:
            try:
                os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
                self.bag_process.wait(timeout=5)
            except Exception as e:  # noqa: BLE001
                rospy.logwarn("HMI A-1: bag stop error: %s", e)
            self.bag_process = None
        self.bag_recording = False
        rospy.loginfo("HMI A-1: bag recording stopped")
        self.bag_state_changed.emit(False, "")

    def shutdown(self):
        try:
            self.stop_bag()
        except Exception:  # noqa: BLE001
            pass

# -*- coding: utf-8 -*-
"""A-1 main window — topbar / left / stage / right / bottom layout."""
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QFrame, QHBoxLayout, QLabel, QMainWindow,
                             QSizePolicy, QSpacerItem, QVBoxLayout, QWidget)

from utils.theme import (BOTTOM_H, LEFT_W, RIGHT_W, TOPBAR_H, WIN_H, WIN_W,
                         labeled_mono_font)
from widgets_a1.bag_banner import BagBanner
from widgets_a1.bottom_bar import BottomBar
from widgets_a1.drive_mode import DriveModeRow
from widgets_a1.sensor_list import SensorList
from widgets_a1.speed_gauge import SpeedGauge
from widgets_a1.stat_card import StatGrid
from widgets_a1.topbar import TopBar
from widgets_a1.vehicle_view_a1 import VehicleViewA1


class MainWindowA1(QMainWindow):
    """The A-1 HMI main window. Owns no ROS subscriptions — bind to controller."""

    def __init__(self, controller):
        super().__init__()
        self._controller = controller
        self.setWindowTitle("Ioniq5 HMI · A-1")
        self.setMinimumSize(1280, 720)
        self.resize(WIN_W, WIN_H)

        self._build_ui()
        self._wire_controller()

    # ─── layout assembly ─────────────────────────────────────────
    def _build_ui(self):
        root = QWidget()
        v = QVBoxLayout(root)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)

        # topbar
        self.topbar = TopBar()
        v.addWidget(self.topbar, 0)

        # middle: 3-column
        mid = QFrame()
        mh = QHBoxLayout(mid)
        mh.setContentsMargins(0, 0, 0, 0)
        mh.setSpacing(0)
        mh.addWidget(self._build_left(), 0)
        mh.addWidget(self._build_stage(), 1)
        mh.addWidget(self._build_right(), 0)
        v.addWidget(mid, 1)

        # bottom
        self.bottom = BottomBar()
        v.addWidget(self.bottom, 0)

        self.setCentralWidget(root)

    def _build_left(self):
        panel = QFrame()
        panel.setObjectName("leftPanel")
        panel.setFixedWidth(LEFT_W)

        v = QVBoxLayout(panel)
        v.setContentsMargins(20, 16, 20, 16)
        v.setSpacing(12)

        # ─ section: speed
        v.addWidget(self._section_label("SPEED"))
        self.speed_gauge = SpeedGauge()
        v.addWidget(self.speed_gauge, 0, Qt.AlignHCenter)

        # ─ section: gear
        v.addWidget(self._section_label("GEAR"))
        self.drive_mode = DriveModeRow()
        v.addWidget(self.drive_mode)

        # ─ section: stats
        v.addWidget(self._section_label("VEHICLE STATS"))
        self.stats = StatGrid(columns=2)
        self.stats.add('limit', "SPD LIMIT", "—", unit="km/h")
        self.stats.add('steer', "STEER", "—", unit="°")
        self.stats.add('rtk',   "GPS RTK", "—")
        self.stats.add('std',   "GPS STD", "—", unit="cm")
        self.stats.add('link',  "LINK", "—")
        self.stats.add('lane',  "LANE", "—")
        v.addWidget(self.stats)

        v.addStretch(1)
        return panel

    def _build_stage(self):
        panel = QFrame()
        panel.setObjectName("stage")

        wrap = QVBoxLayout(panel)
        wrap.setContentsMargins(0, 0, 0, 0)
        wrap.setSpacing(0)
        self.vehicle_view = VehicleViewA1()
        self.vehicle_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        wrap.addWidget(self.vehicle_view, 1)

        return panel

    def _build_right(self):
        panel = QFrame()
        panel.setObjectName("rightPanel")
        panel.setFixedWidth(RIGHT_W)

        v = QVBoxLayout(panel)
        v.setContentsMargins(20, 16, 20, 16)
        v.setSpacing(12)

        # ─ sensor list
        self.sensor_list = SensorList()
        for key, label in [
            ('gps', '/diagnostic/cpt7_gps'),
            ('adcu', '/diagnostic/adcu'),
            ('lidar', '/diagnostic/lidar'),
            ('radar', '/diagnostic/radar'),
            ('v2x', '/diagnostic/v2x'),
            ('hmi', '/diagnostic/hmi'),
            ('vcu', '/diagnostic/vcu'),
            ('cam', '/diagnostic/cam'),
            ('ipc', '/diagnostic/ipc'),
        ]:
            self.sensor_list.add_row(key, label)
        v.addWidget(self.sensor_list)

        # ─ bag banner
        v.addWidget(self._section_label("RECORDING"))
        self.bag_banner = BagBanner()
        v.addWidget(self.bag_banner)

        v.addStretch(1)
        return panel

    def _section_label(self, text):
        l = QLabel(text)
        l.setProperty("role", "section")
        l.setFont(labeled_mono_font(10, 2))
        return l

    # ─── controller wiring ───────────────────────────────────────
    def _wire_controller(self):
        c = self._controller
        # speed / gear / mode / aeb
        c.speed_changed.connect(self.speed_gauge.set_value)
        c.gear_changed.connect(self.drive_mode.set_gear)
        c.mode_changed.connect(self._on_mode_changed)
        c.aeb_changed.connect(self.speed_gauge.set_aeb)

        # vehicle view
        c.steering_changed.connect(self.vehicle_view.set_steering_angle)
        c.ego_pose_changed.connect(self.vehicle_view.set_ego_pose)
        c.objects_changed.connect(self._on_objects)

        # stats
        c.speed_limit_changed.connect(lambda v: self.stats.set('limit', int(v)))
        c.steering_changed.connect(lambda v: self.stats.set('steer', f"{v:.1f}"))
        c.gps_changed.connect(self._on_gps)
        c.link_lane_changed.connect(self._on_link_lane)
        c.odd_changed.connect(self._on_odd)

        # diag LEDs + Hz
        c.diag_changed.connect(self._on_diag)
        c.topic_event.connect(self._on_topic_event)

        # traffic + popup + bag
        c.traffic_changed.connect(self.bottom.set_traffic)
        c.popup_changed.connect(self.bottom.set_popup)
        c.bag_state_changed.connect(self.bag_banner.set_recording)

        # bottom bar → controller
        self.bottom.mode_request.connect(c.request_mode)
        self.bag_banner.toggled.connect(lambda _on: c.toggle_bag())

    # ─── controller event handlers ───────────────────────────────
    def _on_mode_changed(self, autonomous_mode):
        self.speed_gauge.set_mode("AUTO" if autonomous_mode == 1 else "MANUAL")
        self.bottom.set_mode(autonomous_mode == 1)

    def _on_objects(self, objects):
        self.vehicle_view.set_objects(objects)
        self.vehicle_view.update()

    def _on_gps(self, rtk_code, lon_std, lat_std):
        rtk_label = {2: "FIXED", 1: "FLOAT", 0: "NO RTK"}.get(int(rtk_code), "—")
        self.stats.set('rtk', rtk_label)
        max_cm = max(lon_std, lat_std) * 100.0
        self.stats.set('std', f"{max_cm:5.1f}")

    def _on_link_lane(self, link_id, lane):
        self.stats.set('link', str(link_id) if link_id else "—")
        self.stats.set('lane', str(lane) if lane else "—")
        self.topbar.set_link(link_id, lane)

    def _on_odd(self, on_odd, road_state):
        self.topbar.set_odd(on_odd, road_state)

    def _on_diag(self, status_dict):
        for key, status in status_dict.items():
            self.sensor_list.set_status(key, status)

    def _on_topic_event(self, key):
        self.sensor_list.tick(key)

    # ─── close / cleanup ─────────────────────────────────────────
    def closeEvent(self, event):
        try:
            self._controller.shutdown()
        except Exception:  # noqa: BLE001
            pass
        event.accept()

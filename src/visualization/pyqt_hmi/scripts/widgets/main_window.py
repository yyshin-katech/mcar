#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import signal
import os
import subprocess
import datetime
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from std_msgs.msg import UInt8, Bool
from katech_diagnostic_msgs.msg import *
from katech_custom_msgs.msg import ioniq5_ad_can_msg, v_can_msg
from mmc_msgs.msg import chassis_msg, to_control_team_from_local_msg
from v2x_msgs.msg import intersection_array_msg
from perception_ros_msg.msg import object_array_msg

from widgets.vehicle_view import VehicleViewWidget
from widgets.status_indicator import StatusIndicator

class MainDisplayWindow(QMainWindow):
    """메인 디스플레이 윈도우"""
    
    update_sensors_signal = pyqtSignal()
    update_vehicle_signal = pyqtSignal()
    update_steering_signal = pyqtSignal(float)
    update_objects_signal = pyqtSignal(list)
    
    def __init__(self):
        super().__init__()
        
        rospy.init_node('pyqt_display', anonymous=True)
        
        # 데이터 저장
        self.gps_status = 2
        self.adcu_status = 2
        self.lidar_status = 2
        self.radar_status = 2
        self.v2x_status = 2
        self.hmi_status = 2
        self.vcu_status = 2
        self.cam_status = 2
        self.ipc_status = 2
        self.odd_status = 0

        # msg_received 플래그 + 미수신 카운터 (10회 = 1초 미수신 시 비정상)
        self.diag_flags = {
            'gps': {'received': False, 'miss_cnt': 0, 'status_attr': 'gps_status'},
            'adcu': {'received': False, 'miss_cnt': 0, 'status_attr': 'adcu_status'},
            'lidar': {'received': False, 'miss_cnt': 0, 'status_attr': 'lidar_status'},
            'radar': {'received': False, 'miss_cnt': 0, 'status_attr': 'radar_status'},
            'v2x': {'received': False, 'miss_cnt': 0, 'status_attr': 'v2x_status'},
            'hmi': {'received': False, 'miss_cnt': 0, 'status_attr': 'hmi_status'},
            'vcu': {'received': False, 'miss_cnt': 0, 'status_attr': 'vcu_status'},
            'cam': {'received': False, 'miss_cnt': 0, 'status_attr': 'cam_status'},
            'ipc': {'received': False, 'miss_cnt': 0, 'status_attr': 'ipc_status'},
        }
        self.DIAG_MISS_THRESHOLD = 10  # 10 ticks × 100ms = 1초

        # bag 녹화 상태
        self.bag_process = None
        self.bag_recording = False
        
        self.eps_status = 0
        self.traffic_light_color = 0
        self.traffic_light_time = 0
        self.speed_limit = 0
        self.current_speed = 0
        self.selected_mode = 0
        self.autonomous_mode = 0
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
        self.link_id = 0
        self.gear_status = 0
        self.road_state = 0
        self.on_odd = 0
        self.aeb_flag = 0
        self.look_at_intersection_id = 0
        self.look_at_signal_group_id = 0
        self.traffic_light_color = 0   # 0=unknown, 1=green, 2=orange, 3=red
        self.traffic_light_time = 0
        self.GPS_STD_WARN_M = 0.05  # 5cm 초과 시 정밀도 경고

        # UI 초기화
        self.init_ui()
        
        # 지도 로딩
        map_path = "/home/ads/mcar_v13/src/localization/gps_system_localizer/src/A2_LINK_epsg5179.shp"
        if os.path.exists(map_path):
            self.vehicle_view.load_map(map_path)
            rospy.loginfo(f"Map loaded: {map_path}")
        else:
            rospy.logwarn(f"Map file not found: {map_path}")

        # ROS 초기화
        self.init_ros_subscribers()
        self.mode_command_pub = rospy.Publisher('/vehicle/mode_command', UInt8, queue_size=1)

        # Signal 연결
        self.update_sensors_signal.connect(self.update_sensor_display)
        self.update_vehicle_signal.connect(self.update_vehicle_view)
        self.update_steering_signal.connect(self.vehicle_view.set_steering_angle)
        self.update_objects_signal.connect(self.vehicle_view.set_objects)
        
        # Ctrl+C 처리
        signal.signal(signal.SIGINT, self.signal_handler)
        self.interrupt_timer = QTimer()
        self.interrupt_timer.timeout.connect(lambda: None)
        self.interrupt_timer.start(100)

        # 주기적 업데이트
        self.timer = QTimer()
        self.timer.timeout.connect(self.periodic_update)
        self.timer.start(100)
        
    def init_ui(self):
        """UI 초기화"""
        self.setWindowTitle("Vehicle Display System")
        self.setGeometry(100, 100, 1600, 900)
        
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        main_layout = QHBoxLayout()
        
        left_panel = self.create_left_panel()
        right_panel = self.create_vehicle_view()
        
        main_layout.addWidget(left_panel, 30)
        main_layout.addWidget(right_panel, 70)
        
        main_widget.setLayout(main_layout)
        
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
            }
            QLabel {
                color: white;
            }
            QGroupBox {
                color: white;
                border: 2px solid #555;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        
    def create_left_panel(self):
        """왼쪽 패널 생성"""
        panel = QWidget()
        layout = QVBoxLayout()
        
        # 시스템 상태
        status_group = QGroupBox("System Status")
        status_layout = QGridLayout()
        
        self.gps_indicator = StatusIndicator("GPS RTK")
        self.adcu_indicator = StatusIndicator("K-ADCU")
        self.lidar_indicator = StatusIndicator("LIDAR")
        self.radar_indicator = StatusIndicator("Radar")
        self.v2x_indicator = StatusIndicator("V2X")
        self.hmi_indicator = StatusIndicator("HMI")
        self.vcu_indicator = StatusIndicator("VCU")
        self.cam_indicator = StatusIndicator("CAM")
        self.ipc_indicator = StatusIndicator("IPC")
        self.odd_indicator = StatusIndicator("ODD")
        
        indicators = [
            self.adcu_indicator, self.gps_indicator,
            self.lidar_indicator, self.radar_indicator,
            self.v2x_indicator, self.hmi_indicator,
            self.vcu_indicator, self.cam_indicator,
            self.ipc_indicator, self.odd_indicator
        ]
        
        for i, indicator in enumerate(indicators):
            row = i // 2
            col = i % 2
            status_layout.addWidget(indicator, row, col)
            
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # 주행 모드
        mode_group = self.create_mode_group()
        layout.addWidget(mode_group)
        
        # 속도 정보
        speed_group = self.create_speed_group()
        layout.addWidget(speed_group)

        # GPS 정보
        gps_info_group = self.create_gps_info_group()
        layout.addWidget(gps_info_group)

        # 신호등 정보
        traffic_group = self.create_traffic_light_group()
        layout.addWidget(traffic_group)

        layout.addStretch()
        panel.setLayout(layout)
        return panel
    
    def create_mode_group(self):
        """모드 그룹 생성"""
        mode_group = QGroupBox("Current Driving Mode")
        mode_layout = QVBoxLayout()
        
        self.mode_display_label = QLabel("Manual")
        self.mode_display_label.setAlignment(Qt.AlignCenter)
        self.mode_display_label.setStyleSheet("""
            QLabel {
                background-color: #6c757d;
                font-size: 22px;
                font-weight: bold;
                padding: 20px;
                border-radius: 5px;
                color: white;
            }
        """)
        mode_layout.addWidget(self.mode_display_label)
        
        button_container = QWidget()
        button_layout = QHBoxLayout()
        button_layout.setContentsMargins(0, 5, 0, 0)
        button_layout.setSpacing(0)

        self.auto_button = QPushButton("Autonomous")
        self.auto_button.setCheckable(True)
        self.auto_button.setChecked(False)
        self.auto_button.setFixedHeight(60)
        self.auto_button.clicked.connect(self.on_auto_button_clicked)
        self.auto_button.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: black;
                font-size: 18px;
                font-weight: normal;
                border: 2px solid #6c757d;
                border-right: 1px solid #6c757d;
                border-radius: 0px;
                padding: 10px;
            }
            QPushButton:checked {
                background-color: #28a745;
                color: white;
                font-weight: bold;
                border: 2px solid #28a745;
            }
            QPushButton:hover {
                background-color: #e9ecef;
            }
            QPushButton:checked:hover {
                background-color: #218838;
            }
        """)

        self.manual_button = QPushButton("Manual")
        self.manual_button.setCheckable(True)
        self.manual_button.setChecked(True)
        self.manual_button.setFixedHeight(60)
        self.manual_button.clicked.connect(self.on_manual_button_clicked)
        self.manual_button.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: black;
                font-size: 18px;
                font-weight: normal;
                border: 2px solid #6c757d;
                border-left: 1px solid #6c757d;
                border-radius: 0px;
                padding: 10px;
            }
            QPushButton:checked {
                background-color: #28a745;
                color: white;
                font-weight: bold;
                border: 2px solid #28a745;
            }
            QPushButton:hover {
                background-color: #e9ecef;
            }
            QPushButton:checked:hover {
                background-color: #218838;
            }
        """)

        button_layout.addWidget(self.auto_button)
        button_layout.addWidget(self.manual_button)
        button_container.setLayout(button_layout)
        mode_layout.addWidget(button_container)
        mode_group.setLayout(mode_layout)
        
        return mode_group
    
    def create_speed_group(self):
        """속도 그룹 생성"""
        speed_group = QGroupBox("Speed Information")
        speed_main_layout = QVBoxLayout()
        speed_h_layout = QHBoxLayout()
        
        # 속도 제한
        speed_limit_container = QWidget()
        speed_limit_layout = QVBoxLayout()
        
        speed_limit_title = QLabel("Limit")
        speed_limit_title.setAlignment(Qt.AlignCenter)
        speed_limit_title.setStyleSheet("""
            QLabel {
                font-size: 14px;
                font-weight: bold;
                color: white;
                background-color: transparent;
                padding: 5px;
            }
        """)
        
        self.speed_label = QLabel("0")
        self.speed_label.setAlignment(Qt.AlignCenter)
        self.speed_label.setStyleSheet("""
            QLabel {
                background-color: #dc3545;
                font-size: 32px;
                font-weight: bold;
                padding: 20px;
                border-radius: 10px;
                color: white;
                border: 2px solid white;
            }
        """)
        
        speed_limit_unit = QLabel("km/h")
        speed_limit_unit.setAlignment(Qt.AlignCenter)
        speed_limit_unit.setStyleSheet("""
            QLabel {
                font-size: 12px;
                color: #aaa;
                background-color: transparent;
                padding: 2px;
            }
        """)
        
        speed_limit_layout.addWidget(speed_limit_title)
        speed_limit_layout.addWidget(self.speed_label)
        speed_limit_layout.addWidget(speed_limit_unit)
        speed_limit_container.setLayout(speed_limit_layout)
        
        # 현재 속도
        current_speed_container = QWidget()
        current_speed_layout = QVBoxLayout()
        
        current_speed_title = QLabel("Current")
        current_speed_title.setAlignment(Qt.AlignCenter)
        current_speed_title.setStyleSheet("""
            QLabel {
                font-size: 14px;
                font-weight: bold;
                color: white;
                background-color: transparent;
                padding: 5px;
            }
        """)
        
        self.current_speed_label = QLabel("0")
        self.current_speed_label.setAlignment(Qt.AlignCenter)
        self.current_speed_label.setStyleSheet("""
            QLabel {
                background-color: #007bff;
                font-size: 32px;
                font-weight: bold;
                padding: 20px;
                border-radius: 10px;
                color: white;
                border: 2px solid white;
            }
        """)
        
        current_speed_unit = QLabel("km/h")
        current_speed_unit.setAlignment(Qt.AlignCenter)
        current_speed_unit.setStyleSheet("""
            QLabel {
                font-size: 12px;
                color: #aaa;
                background-color: transparent;
                padding: 2px;
            }
        """)
        
        current_speed_layout.addWidget(current_speed_title)
        current_speed_layout.addWidget(self.current_speed_label)
        current_speed_layout.addWidget(current_speed_unit)
        current_speed_container.setLayout(current_speed_layout)
        
        # 기어 상태
        gear_container = QWidget()
        gear_layout = QVBoxLayout()

        gear_title = QLabel("Gear")
        gear_title.setAlignment(Qt.AlignCenter)
        gear_title.setStyleSheet("""
            QLabel {
                font-size: 14px;
                font-weight: bold;
                color: white;
                background-color: transparent;
                padding: 5px;
            }
        """)

        self.gear_label = QLabel("P")
        self.gear_label.setAlignment(Qt.AlignCenter)
        self.gear_label.setStyleSheet("""
            QLabel {
                background-color: #444;
                font-size: 32px;
                font-weight: bold;
                padding: 20px;
                border-radius: 10px;
                color: white;
                border: 2px solid white;
            }
        """)

        gear_layout.addWidget(gear_title)
        gear_layout.addWidget(self.gear_label)
        gear_container.setLayout(gear_layout)

        speed_h_layout.addWidget(speed_limit_container)
        speed_h_layout.addWidget(current_speed_container)
        speed_h_layout.addWidget(gear_container)

        speed_main_layout.addLayout(speed_h_layout)
        speed_group.setLayout(speed_main_layout)
        
        return speed_group
        
    def create_gps_info_group(self):
        """GPS 정보 그룹 생성"""
        gps_group = QGroupBox("GPS Information")
        gps_layout = QVBoxLayout()

        self.lane_label = QLabel("Curr LANE: 0")
        self.lane_label.setStyleSheet("""
            QLabel {
                font-size: 16px;
                font-weight: bold;
                color: white;
                background-color: transparent;
                padding: 5px;
            }
        """)

        self.gpsrtk_label = QLabel("GPSRTK: N/A")
        self.gpsrtk_label.setStyleSheet("""
            QLabel {
                font-size: 16px;
                font-weight: bold;
                color: white;
                background-color: transparent;
                padding: 5px;
            }
        """)

        self.gps_std_label = QLabel("GPS std\nH: --.-- cm\nV: --.-- cm")
        self.gps_std_label.setStyleSheet("""
            QLabel {
                font-size: 14px;
                font-weight: bold;
                color: #1afff0;
                background-color: transparent;
                font-family: 'DejaVu Sans Mono', monospace;
                padding: 5px;
            }
        """)

        gps_layout.addWidget(self.lane_label)
        gps_layout.addWidget(self.gpsrtk_label)
        gps_layout.addWidget(self.gps_std_label)
        gps_group.setLayout(gps_layout)

        return gps_group

    def create_traffic_light_group(self):
        """신호등 정보 그룹 생성"""
        group = QGroupBox("Traffic Light")
        layout = QHBoxLayout()

        # 신호등 원형 표시
        self.traffic_light_indicator = QLabel()
        self.traffic_light_indicator.setFixedSize(60, 60)
        self.traffic_light_indicator.setAlignment(Qt.AlignCenter)
        self.traffic_light_indicator.setStyleSheet("""
            QLabel {
                background-color: #333;
                border-radius: 30px;
                border: 3px solid #555;
            }
        """)

        # 남은 시간 표시
        self.traffic_time_label = QLabel("N/A")
        self.traffic_time_label.setAlignment(Qt.AlignCenter)
        self.traffic_time_label.setStyleSheet("""
            QLabel {
                font-size: 36px;
                font-weight: bold;
                color: white;
                background-color: transparent;
                padding: 5px;
            }
        """)

        layout.addWidget(self.traffic_light_indicator)
        layout.addWidget(self.traffic_time_label)
        group.setLayout(layout)
        return group

    def create_vehicle_view(self):
        """차량 뷰 패널 생성"""
        panel = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)

        self.vehicle_view = VehicleViewWidget()
        layout.addWidget(self.vehicle_view)

        # 팝업 오버레이 (차량 뷰 위에 표시)
        self.popup_label = QLabel(self.vehicle_view)
        self.popup_label.setAlignment(Qt.AlignCenter)
        self.popup_label.setWordWrap(True)
        self.popup_label.hide()
        self.popup_label.setStyleSheet("""
            QLabel {
                background-color: rgba(200, 30, 30, 220);
                color: white;
                font-size: 28px;
                font-weight: bold;
                padding: 20px 40px;
                border-radius: 12px;
                border: 3px solid rgba(255, 255, 255, 180);
            }
        """)

        # bag 녹화 UI (차량 뷰 오른쪽 위)
        bag_container = QWidget(self.vehicle_view)
        bag_container.setFixedSize(320, 90)
        bag_container.setStyleSheet("background-color: rgba(0, 0, 0, 160); border-radius: 8px;")
        bag_layout = QVBoxLayout(bag_container)
        bag_layout.setContentsMargins(8, 6, 8, 6)
        bag_layout.setSpacing(4)

        # 경로 입력
        path_layout = QHBoxLayout()
        path_label = QLabel("Path:")
        path_label.setStyleSheet("color: #aaa; font-size: 11px; background: transparent;")
        self.bag_path_edit = QLineEdit(os.path.expanduser("~/bag_data"))
        self.bag_path_edit.setStyleSheet("""
            QLineEdit {
                background-color: #333; color: white; font-size: 11px;
                border: 1px solid #555; border-radius: 3px; padding: 2px 4px;
            }
        """)
        path_layout.addWidget(path_label)
        path_layout.addWidget(self.bag_path_edit)
        bag_layout.addLayout(path_layout)

        # 버튼 + 상태
        btn_layout = QHBoxLayout()
        self.bag_record_btn = QPushButton("REC")
        self.bag_record_btn.setFixedSize(60, 28)
        self.bag_record_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc3545; color: white; font-size: 12px;
                font-weight: bold; border-radius: 4px; border: none;
            }
            QPushButton:hover { background-color: #c82333; }
        """)
        self.bag_record_btn.clicked.connect(self.toggle_bag_recording)

        self.bag_status_label = QLabel("Stopped")
        self.bag_status_label.setStyleSheet("color: #888; font-size: 11px; background: transparent;")

        btn_layout.addWidget(self.bag_record_btn)
        btn_layout.addWidget(self.bag_status_label)
        btn_layout.addStretch()
        bag_layout.addLayout(btn_layout)

        self.bag_container = bag_container

        panel.setLayout(layout)
        return panel

    def init_ros_subscribers(self):
        """ROS Subscriber 초기화"""
        rospy.Subscriber("/diagnostic/cpt7_gps", cpt7_gps_diagnostic_msg, self.gps_callback)
        rospy.Subscriber("/diagnostic/adcu", k_adcu_diagnostic_msg, self.adcu_callback)
        rospy.Subscriber("/diagnostic/lidar", lidar_diagnostic_msg, self.lidar_callback)
        rospy.Subscriber("/diagnostic/radar", radar_diagnostic_msg, self.radar_callback)
        rospy.Subscriber("/diagnostic/v2x", v2x_diagnostic_msg, self.v2x_callback)
        rospy.Subscriber("/diagnostic/hmi", hmi_diagnostic_msg, self.hmi_callback)
        rospy.Subscriber("/diagnostic/vcu", vcu_diagnostic_msg, self.vcu_callback)
        rospy.Subscriber("/diagnostic/cam", cam_diagnostic_msg, self.cam_callback)
        rospy.Subscriber("/diagnostic/ipc", ipc_diagnostic_msg, self.ipc_callback)
        rospy.Subscriber("/sensors/chassis", chassis_msg, self.chassis_callback)
        rospy.Subscriber("/sensors/ioniq5_ad_can", ioniq5_ad_can_msg, self.ioniq5_ad_can_callback)
        rospy.Subscriber("/sensors/v_can", v_can_msg, self.v_can_callback)
        rospy.Subscriber("/localization/to_control_team", to_control_team_from_local_msg, self.local_callback)
        rospy.Subscriber("/katri_v2x_node/katri_spat", intersection_array_msg, self.traffic_light_callback)
        rospy.Subscriber("/track_Multi_RS", object_array_msg, self.track_objects_callback)
        
    def gps_callback(self, msg):
        self.diag_flags['gps']['received'] = True
        self.gps_rtk_code = msg.GPSRTK_StatCode
        self.gps_lon_std = msg.lon_std
        self.gps_lat_std = msg.lat_std

    def adcu_callback(self, msg):
        self.diag_flags['adcu']['received'] = True
        self.adcu_swc_code = msg.ADCU_SWC_StatCode

    def lidar_callback(self, msg):
        self.diag_flags['lidar']['received'] = True
        self.lidar_center_code = msg.LIDAR_Center_StatCode
        self.lidar_right_code = msg.LIDAR_Right_StatCode
        self.lidar_left_code = msg.LIDAR_Left_StatCode

    def radar_callback(self, msg):
        self.diag_flags['radar']['received'] = True

    def v2x_callback(self, msg):
        self.diag_flags['v2x']['received'] = True
        self.v2x_stat_code = msg.V2X_StatCode

    def hmi_callback(self, msg):
        self.diag_flags['hmi']['received'] = True

    def vcu_callback(self, msg):
        self.diag_flags['vcu']['received'] = True
        self.vcu_stat_code = msg.VCU_StatCode

    def cam_callback(self, msg):
        self.diag_flags['cam']['received'] = True

    def ipc_callback(self, msg):
        self.diag_flags['ipc']['received'] = True
        self.ipc_swc_code = msg.IPC_SWC_StatCode
        
    def ioniq5_ad_can_callback(self, msg):
        self.autonomous_mode = msg.autonomous_mode

    def v_can_callback(self, msg):
        self.update_steering_signal.emit(msg.steering_angle)
        avg_spd = (msg.wheel_speed_fl + msg.wheel_speed_fr +
                   msg.wheel_speed_rl + msg.wheel_speed_rr) / 4.0
        self.current_speed = avg_spd * 3.6  # m/s → km/h
        self.gear_status = msg.gear_status

    def chassis_callback(self, msg):
        self.current_speed = getattr(msg, 'vehicle_speed', 0)
        self.aeb_flag = getattr(msg, 'AEB_flag', 0)
        
    def local_callback(self, msg):
        self.speed_limit = msg.Speed_Limit
        self.odd_status = msg.Road_State
        self.road_state = msg.Road_State
        self.on_odd = msg.On_ODD
        self.link_id = msg.LINK_ID
        self.look_at_intersection_id = msg.look_at_IntersectionID
        self.look_at_signal_group_id = msg.look_at_signalGroupID
        self.update_sensors_signal.emit()

        ego_x = msg.host_east
        ego_y = msg.host_north
        ego_heading = msg.host_yaw
        
        self.vehicle_view.set_ego_pose(ego_x, ego_y, ego_heading)
        
    def track_objects_callback(self, msg):
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
        self.update_objects_signal.emit(objects)

    def traffic_light_callback(self, msg):
        if self.look_at_intersection_id == 0:
            self.traffic_light_time = 0
            self.traffic_light_color = 0
            return
        for intersection in msg.data:
            if intersection.IntersectionID != self.look_at_intersection_id:
                continue
            movement = intersection.Movements
            if self.look_at_signal_group_id != 0 and \
               movement.SignalGroupID != self.look_at_signal_group_id:
                continue
            self.traffic_light_time = movement.TimeChangeDetails
            phase = movement.MovementPhaseStatus
            if phase == 3:
                self.traffic_light_color = 3    # 초록 (stat_display: color 3)
            elif phase == 8:
                self.traffic_light_color = 2    # 주황 (stat_display: color 2)
            elif phase == 6:
                self.traffic_light_color = 1    # 빨강 (stat_display: color 1)
            else:
                self.traffic_light_color = 0
            return
        
    def update_sensor_display(self):
        """센서 상태 업데이트"""
        self.gps_indicator.set_status(self.gps_status)
        self.adcu_indicator.set_status(self.adcu_status)
        self.lidar_indicator.set_status(self.lidar_status)
        self.radar_indicator.set_status(self.radar_status)
        self.v2x_indicator.set_status(self.v2x_status)
        self.hmi_indicator.set_status(self.hmi_status)
        self.vcu_indicator.set_status(self.vcu_status)
        self.cam_indicator.set_status(self.cam_status)
        self.ipc_indicator.set_status(self.ipc_status)
        self.odd_indicator.set_status(self.odd_status)
        
    def update_vehicle_view(self):
        pass
        
    def _evaluate_diag_status(self, name, received):
        """0=정상, 1=warning(StatCode 기반), 2=error(메시지 단절)"""
        d = self.diag_flags[name]
        if received:
            d['miss_cnt'] = 0
        else:
            d['miss_cnt'] += 1
            if d['miss_cnt'] > self.DIAG_MISS_THRESHOLD:
                return 2

        # 메시지 수신 중 → StatCode 도메인 조건으로 warning 판정
        if name == 'gps':
            if self.gps_rtk_code < 2:
                return 1  # No RTK or Float
            if self.gps_lon_std > self.GPS_STD_WARN_M or self.gps_lat_std > self.GPS_STD_WARN_M:
                return 1  # 정밀도 5cm 초과
        elif name == 'lidar':
            if 1 in (self.lidar_center_code, self.lidar_right_code, self.lidar_left_code):
                return 1
        elif name == 'v2x':
            if self.v2x_stat_code == 1:
                return 1
        elif name == 'vcu':
            if self.vcu_stat_code == 1:
                return 2  # VCU life_count 결손 = 장치 고장 → 에러
        elif name == 'ipc':
            if self.ipc_swc_code == 1:
                return 1
        elif name == 'adcu':
            if self.adcu_swc_code == 1:
                return 1
        return 0

    def periodic_update(self):
        """주기적 업데이트"""
        # diagnostic 상태 결정 (수신 여부 + StatCode 도메인 조건)
        for name, d in self.diag_flags.items():
            received = d['received']
            d['received'] = False
            setattr(self, d['status_attr'], self._evaluate_diag_status(name, received))

        mode_msg = UInt8()
        mode_msg.data = self.selected_mode
        self.mode_command_pub.publish(mode_msg)
        
        if self.autonomous_mode == 1:
            self.mode_display_label.setText("Autonomous")
            self.mode_display_label.setStyleSheet("""
                QLabel {
                    background-color: #28a745;
                    font-size: 22px;
                    font-weight: bold;
                    padding: 20px;
                    border-radius: 5px;
                    color: white;
                }
            """)
            if not self.auto_button.isChecked():
                self.auto_button.setChecked(True)
                self.manual_button.setChecked(False)
            # Auto 모드 진입 후엔 재요청 막기
            self.auto_button.setEnabled(False)
        else:
            self.mode_display_label.setText("Manual")
            self.mode_display_label.setStyleSheet("""
                QLabel {
                    background-color: #6c757d;
                    font-size: 22px;
                    font-weight: bold;
                    padding: 20px;
                    border-radius: 5px;
                    color: white;
                }
            """)
            if not self.manual_button.isChecked():
                self.manual_button.setChecked(True)
                self.auto_button.setChecked(False)
            # Manual일 때만 Auto 요청 가능
            self.auto_button.setEnabled(True)
            
        self.speed_label.setText(str(self.speed_limit))
        self.current_speed_label.setText(str(int(self.current_speed)))

        # 기어 상태 업데이트
        gear_map = {0: "-", 1: "P", 2: "R", 3: "N", 4: "D"}
        gear_str = gear_map.get(self.gear_status, "-")
        gear_color = "#dc3545" if self.gear_status == 2 else "#28a745" if self.gear_status == 4 else "#444"
        self.gear_label.setText(gear_str)
        self.gear_label.setStyleSheet("""
            QLabel {
                background-color: %s;
                font-size: 32px;
                font-weight: bold;
                padding: 20px;
                border-radius: 10px;
                color: white;
                border: 2px solid white;
            }
        """ % gear_color)

        # GPS 정보 업데이트
        self.lane_label.setText("Curr LANE: " + str(self.link_id))
        rtk_map = {2: "Fixed", 1: "Float", 0: "No RTK"}
        rtk_str = rtk_map.get(self.gps_rtk_code, "N/A")
        self.gpsrtk_label.setText("GPSRTK: " + rtk_str)

        # GPS std (H/V cm + 5cm 임계 색상)
        h_cm = self.gps_lon_std * 100.0
        v_cm = self.gps_lat_std * 100.0
        self.gps_std_label.setText("GPS std\nH: {:6.2f} cm\nV: {:6.2f} cm".format(h_cm, v_cm))
        if self.gps_lon_std > self.GPS_STD_WARN_M or self.gps_lat_std > self.GPS_STD_WARN_M:
            std_color = "#ff8800"  # 주황: 정밀도 나쁨
        else:
            std_color = "#1afff0"  # 청록: 정밀도 양호
        self.gps_std_label.setStyleSheet("""
            QLabel {{
                font-size: 14px;
                font-weight: bold;
                color: {color};
                background-color: transparent;
                font-family: 'DejaVu Sans Mono', monospace;
                padding: 5px;
            }}
        """.format(color=std_color))

        # 센서 인디케이터 업데이트
        self.update_sensor_display()

        # 신호등 업데이트
        self.update_traffic_light()

        # 팝업 판단 (stat_display와 동일 로직)
        self.update_popup()

        # vehicle view 갱신 (타이머 기반 단일 repaint)
        self.vehicle_view.update()

    def update_traffic_light(self):
        """신호등 색상 및 남은 시간 업데이트"""
        if self.look_at_intersection_id == 0 or self.traffic_light_color == 0:
            self.traffic_light_indicator.setStyleSheet("""
                QLabel { background-color: #333; border-radius: 30px; border: 3px solid #555; }
            """)
            self.traffic_time_label.setText("N/A")
            self.traffic_time_label.setStyleSheet("""
                QLabel { font-size: 36px; font-weight: bold; color: #888; background-color: transparent; }
            """)
            return

        seconds = self.traffic_light_time // 10

        if self.traffic_light_color == 1:    # 초록 (stat_display: color 1 → green)
            color = "#00cc00"
            border = "#00ff00"
        elif self.traffic_light_color == 2:  # 주황 (stat_display: color 2 → orange)
            color = "#cc8800"
            border = "#ffaa00"
        elif self.traffic_light_color == 3:  # 빨강 (stat_display: color 3 → red)
            color = "#cc0000"
            border = "#ff0000"
        else:
            color = "#333"
            border = "#555"

        self.traffic_light_indicator.setStyleSheet("""
            QLabel {{ background-color: {c}; border-radius: 30px; border: 3px solid {b}; }}
        """.format(c=color, b=border))
        self.traffic_time_label.setText(str(seconds) + "s")
        self.traffic_time_label.setStyleSheet("""
            QLabel {{ font-size: 36px; font-weight: bold; color: {b}; background-color: transparent; }}
        """.format(b=border))

    def update_popup(self):
        """시스템 고장/경고 팝업 표시 (stat_display 로직과 동일)"""
        statuses = {
            "GPS": self.gps_status, "ADCU": self.adcu_status,
            "LiDAR": self.lidar_status, "Radar": self.radar_status,
            "V2X": self.v2x_status, "HMI": self.hmi_status,
            "CAM": self.cam_status, "VCU": self.vcu_status,
            "IPC": self.ipc_status,
        }
        abnormal = [name for name, st in statuses.items() if st != 0]
        abnormal_count = len(abnormal)

        popup_text = ""
        popup_color = "rgba(200, 30, 30, 220)"

        if abnormal_count == 1:
            popup_text = "  " + abnormal[0] + " 센서 고장"
            popup_color = "rgba(200, 140, 0, 220)"
        elif abnormal_count >= 2:
            popup_text = " 시스템 고장 (" + str(abnormal_count) + "개 시스템 오류)"
            popup_color = "rgba(200, 30, 30, 220)"
        elif self.road_state == 1:
            popup_text = "전방 ODD 이탈 경고"
            popup_color = "rgba(200, 140, 0, 220)"
        elif self.aeb_flag == 1:
            popup_text = " 전방 추돌 경고"
            popup_color = "rgba(200, 30, 30, 220)"
        elif self.on_odd == 1:
            popup_text = "ODD 이탈 !!!!"
            popup_color = "rgba(200, 30, 30, 220)"

        if popup_text:
            self.popup_label.setText(popup_text)
            self.popup_label.setStyleSheet("""
                QLabel {{
                    background-color: {color};
                    color: white;
                    font-size: 28px;
                    font-weight: bold;
                    padding: 20px 40px;
                    border-radius: 12px;
                    border: 3px solid rgba(255, 255, 255, 180);
                }}
            """.format(color=popup_color))
            self.popup_label.adjustSize()
            # 차량 뷰 상단 중앙에 배치
            vw = self.vehicle_view.width()
            pw = self.popup_label.width()
            self.popup_label.move((vw - pw) // 2, 20)
            self.popup_label.show()
            self.popup_label.raise_()
        else:
            self.popup_label.hide()

    def on_auto_button_clicked(self):
        if self.auto_button.isChecked():
            self.manual_button.setChecked(False)
            self.selected_mode = 1
            rospy.loginfo("모드 변경 요청: Autonomous (1초 펄스)")
            # MD_AD_Req는 mode change request 신호이므로 1초간 1을 유지한 뒤 0으로 복귀
            QTimer.singleShot(1000, self._reset_mode_request)
        else:
            self.auto_button.setChecked(True)

    def _reset_mode_request(self):
        self.selected_mode = 0

    def on_manual_button_clicked(self):
        if self.manual_button.isChecked():
            self.auto_button.setChecked(False)
            self.selected_mode = 0
            rospy.loginfo("모드 변경: Manual")
        else:
            self.manual_button.setChecked(True)

    def toggle_bag_recording(self):
        if self.bag_recording:
            self.stop_bag_recording()
        else:
            self.start_bag_recording()

    def start_bag_recording(self):
        bag_dir = self.bag_path_edit.text().strip()
        if not bag_dir:
            bag_dir = os.path.expanduser("~/bag_data")
        os.makedirs(bag_dir, exist_ok=True)

        timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        prefix = os.path.join(bag_dir, timestamp)

        self.bag_process = subprocess.Popen(
            ["rosbag", "record", "-a", "--split", "--size=10240", "-o", prefix],
            preexec_fn=os.setsid
        )
        self.bag_recording = True
        self.bag_path_edit.setEnabled(False)
        self.bag_record_btn.setText("STOP")
        self.bag_record_btn.setStyleSheet("""
            QPushButton {
                background-color: #28a745; color: white; font-size: 12px;
                font-weight: bold; border-radius: 4px; border: none;
            }
            QPushButton:hover { background-color: #218838; }
        """)
        self.bag_status_label.setText("Recording...")
        self.bag_status_label.setStyleSheet("color: #ff4444; font-size: 11px; background: transparent;")
        rospy.loginfo("Bag recording started: %s", prefix)

    def stop_bag_recording(self):
        if self.bag_process:
            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
            self.bag_process.wait()
            self.bag_process = None
        self.bag_recording = False
        self.bag_path_edit.setEnabled(True)
        self.bag_record_btn.setText("REC")
        self.bag_record_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc3545; color: white; font-size: 12px;
                font-weight: bold; border-radius: 4px; border: none;
            }
            QPushButton:hover { background-color: #c82333; }
        """)
        self.bag_status_label.setText("Stopped")
        self.bag_status_label.setStyleSheet("color: #888; font-size: 11px; background: transparent;")
        rospy.loginfo("Bag recording stopped")

    def _update_bag_container_pos(self):
        if hasattr(self, 'bag_container') and hasattr(self, 'vehicle_view'):
            vw = self.vehicle_view.width()
            bw = self.bag_container.width()
            self.bag_container.move(vw - bw - 2, 2)
            self.bag_container.raise_()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_bag_container_pos()

    def showEvent(self, event):
        super().showEvent(event)
        QTimer.singleShot(0, self._update_bag_container_pos)

    def signal_handler(self, sig, frame):
        print("\n시그널을 받았습니다. 프로그램을 종료합니다...")
        self.cleanup()
        QApplication.quit()

    def cleanup(self):
        print("리소스 정리 중...")
        self.stop_bag_recording()
        rospy.signal_shutdown("User interrupted")

    def closeEvent(self, event):
        self.cleanup()
        event.accept()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import signal
import os
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
        
        self.eps_status = 0
        self.traffic_light_color = 0
        self.traffic_light_time = 0
        self.speed_limit = 0
        self.current_speed = 0
        self.selected_mode = 0
        self.autonomous_mode = 0
        self.gps_rtk_code = 0
        self.link_id = 0
        self.road_state = 0
        self.on_odd = 0
        self.aeb_flag = 0
        self.look_at_intersection_id = 0
        self.look_at_signal_group_id = 0
        self.traffic_light_color = 0   # 0=unknown, 1=green, 2=orange, 3=red
        self.traffic_light_time = 0

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
        
        speed_h_layout.addWidget(speed_limit_container)
        speed_h_layout.addWidget(current_speed_container)
        
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

        gps_layout.addWidget(self.lane_label)
        gps_layout.addWidget(self.gpsrtk_label)
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
        self.gps_status = 0
        self.gps_rtk_code = msg.GPSRTK_StatCode
        self.update_sensors_signal.emit()
        
    def adcu_callback(self, msg):
        self.adcu_status = 0
        self.update_sensors_signal.emit()
        
    def lidar_callback(self, msg):
        self.lidar_status = 0
        self.update_sensors_signal.emit()
        
    def radar_callback(self, msg):
        self.radar_status = 0
        self.update_sensors_signal.emit()
        
    def v2x_callback(self, msg):
        self.v2x_status = 0
        self.update_sensors_signal.emit()
        
    def hmi_callback(self, msg):
        self.hmi_status = 0
        self.update_sensors_signal.emit()
        
    def vcu_callback(self, msg):
        self.vcu_status = 0
        self.update_sensors_signal.emit()
        
    def cam_callback(self, msg):
        self.cam_status = 0
        self.update_sensors_signal.emit()
        
    def ipc_callback(self, msg):
        self.ipc_status = 0
        self.update_sensors_signal.emit()
        
    def ioniq5_ad_can_callback(self, msg):
        self.autonomous_mode = msg.autonomous_mode

    def v_can_callback(self, msg):
        self.update_steering_signal.emit(msg.steering_angle)

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
        
    def periodic_update(self):
        """주기적 업데이트"""
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
            
        self.speed_label.setText(str(self.speed_limit))
        self.current_speed_label.setText(str(int(self.current_speed)))

        # GPS 정보 업데이트
        self.lane_label.setText("Curr LANE: " + str(self.link_id))
        rtk_map = {2: "Fixed", 1: "Float", 0: "No RTK"}
        rtk_str = rtk_map.get(self.gps_rtk_code, "N/A")
        self.gpsrtk_label.setText("GPSRTK: " + rtk_str)

        # 신호등 업데이트
        self.update_traffic_light()

        # 팝업 판단 (stat_display와 동일 로직)
        self.update_popup()

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
            rospy.loginfo("모드 변경: Autonomous")
        else:
            self.auto_button.setChecked(True)

    def on_manual_button_clicked(self):
        if self.manual_button.isChecked():
            self.auto_button.setChecked(False)
            self.selected_mode = 0
            rospy.loginfo("모드 변경: Manual")
        else:
            self.manual_button.setChecked(True)

    def signal_handler(self, sig, frame):
        print("\n시그널을 받았습니다. 프로그램을 종료합니다...")
        self.cleanup()
        QApplication.quit()
        
    def cleanup(self):
        print("리소스 정리 중...")
        rospy.signal_shutdown("User interrupted")
        
    def closeEvent(self, event):
        self.cleanup()
        event.accept()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import math
import signal
import shapefile  # 추가
import os
import glob

# ROS 메시지
from katech_diagnostic_msgs.msg import *
from mmc_msgs.msg import chassis_msg, to_control_team_from_local_msg
from v2x_msgs.msg import intersection_array_msg
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import UInt8, Bool

# from derived_object_msgs.msg import ObjectArray

def load_shapefile(shp_file):
    """Load shapefile and return line features"""
    try:
        sf = shapefile.Reader(shp_file)
        print(f"Loading shapefile: {os.path.basename(shp_file)}")
        
        shapes = sf.shapes()
        if not shapes:
            print(f"No shapes found in {shp_file}")
            return []
        
        features = []
        
        for j, shape in enumerate(shapes):
            if len(shape.points) == 0:
                continue
            
            feature = {
                'id': j,
                'points': shape.points  # [(x, y), (x, y), ...]
            }
            features.append(feature)
        
        print(f"Loaded {len(features)} features from {os.path.basename(shp_file)}")
        return features
        
    except Exception as e:
        print(f"Error loading {shp_file}: {str(e)}")
        return []

class VehicleViewWidget(QWidget):
    """자차 및 오브젝트를 2D 탑뷰로 표시하는 위젯"""
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(800, 600)
        
        # 자차 위치 (항상 중앙)
        self.ego_x = 0
        self.ego_y = 0
        self.ego_heading = 0
        
        # 오브젝트 리스트 (상대 좌표)
        self.objects = []
        
        # 줌 레벨 (미터/픽셀)
        self.scale = 10.0  # 1픽셀 = 0.1미터
        # 지도 데이터
        self.map_features = []
        
        # 배경색
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(self.backgroundRole(), QColor(30, 30, 30))
        self.setPalette(palette)

    def load_map(self, shp_file):
        """Load map from shapefile"""
        self.map_features = load_shapefile(shp_file)
        self.update()

    def set_ego_pose(self, x, y, heading):
        self.ego_x = x
        self.ego_y = y
        self.ego_heading = heading
        self.update()
        
    def rotate_point(self, x, y, angle):
        """Rotate point by angle around origin"""
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        
        x_rot = x * cos_angle - y * sin_angle
        y_rot = x * sin_angle + y * cos_angle
        
        return x_rot, y_rot

    def world_to_screen(self, world_x, world_y):
        """Convert world coordinates to screen coordinates"""
        # 1. 상대 좌표 계산
        rel_x = world_x - self.ego_x
        rel_y = world_y - self.ego_y
        
        # 2. 회전 적용 (자차 heading 기준)
        rot_x, rot_y = self.rotate_point(rel_x, rel_y, -self.ego_heading + math.pi/2)
        
        # 3. 화면 좌표로 변환
        center_x = self.width() / 2
        center_y = self.height() * 0.75  # 75% 아래 위치 (0.67~0.8 사이로 조정 가능)
        
        screen_x = center_x + rot_x * self.scale
        screen_y = center_y - rot_y * self.scale  # y축 반전
        
        return screen_x, screen_y

    def set_objects(self, objects):
        self.objects = objects
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 배경
        painter.fillRect(self.rect(), QColor(30, 30, 30))
        
        # 중심점 계산
        center_x = self.width() / 2
        center_y = self.height() * 0.75
        
        # 그리드 그리기
        self.draw_grid(painter, center_x, center_y)
        
        # 지도 그리기 (먼저 그려서 아래 레이어)
        self.draw_map(painter)

        # 자차 그리기 (항상 중앙, 위쪽이 전방)
        self.draw_ego_vehicle(painter, center_x, center_y)
        
        # 오브젝트 그리기
        self.draw_objects(painter, center_x, center_y)
        
        # 범위 표시
        painter.setPen(QPen(QColor(100, 100, 100), 1))
        painter.drawText(10, 20, f"Scale: {self.scale:.1f}m/div")
        painter.drawText(10, 40, f"Ego: ({self.ego_x:.1f}, {self.ego_y:.1f})")
        
        # 디버깅 정보
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.drawText(10, 20, f"Scale: {self.scale:.1f}m/div")
        painter.drawText(10, 40, f"Ego: ({self.ego_x:.1f}, {self.ego_y:.1f})")
        painter.drawText(10, 60, f"Heading: {math.degrees(self.ego_heading):.1f}°")
        painter.drawText(10, 80, f"Features: {len(self.map_features)}")
    def draw_grid(self, painter, cx, cy):
        """그리드 그리기"""
        painter.setPen(QPen(QColor(60, 60, 60), 1, Qt.DashLine))
        
        grid_size = 100  # 픽셀 단위
        
        # 세로선
        for x in range(0, self.width(), grid_size):
            painter.drawLine(x, 0, x, self.height())
            
        # 가로선
        for y in range(0, self.height(), grid_size):
            painter.drawLine(0, y, self.width(), y)
            
        # 중앙선 강조
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.drawLine(cx, 0, cx, self.height())
        painter.drawLine(0, cy, self.width(), cy)
        
    def draw_map(self, painter):
        """지도 그리기"""
        if not self.map_features:
            return
        
        # 지도 선 스타일
        painter.setPen(QPen(QColor(242, 217, 132, 100), 2))  # 반투명 노란색
        
        for feature in self.map_features:
            points = feature['points']
            
            if len(points) < 2:
                continue
            
            # QPainterPath 사용해서 부드럽게 그리기
            path = QPainterPath()
            
            # 첫 번째 점
            first_point = points[0]
            screen_x, screen_y = self.world_to_screen(first_point[0], first_point[1])
            
            # 화면 밖이면 건너뛰기 (성능 향상)
            if self.is_point_out_of_view(screen_x, screen_y, margin=100):
                continue
                
            path.moveTo(screen_x, screen_y)
            
            # 나머지 점들
            valid_points = 1
            for i in range(1, len(points)):
                point = points[i]
                screen_x, screen_y = self.world_to_screen(point[0], point[1])
                
                # 화면 범위 체크
                if abs(screen_x) < 10000 and abs(screen_y) < 10000:
                    path.lineTo(screen_x, screen_y)
                    valid_points += 1
            
            # 유효한 선만 그리기
            if valid_points >= 2:
                painter.drawPath(path)

    def is_point_out_of_view(self, x, y, margin=0):
        """화면 밖 여부 확인"""
        return (x < -margin or x > self.width() + margin or 
                y < -margin or y > self.height() + margin)

    def draw_ego_vehicle(self, painter, cx, cy):
        """자차 그리기"""
        # 자차를 화면 아래쪽에 그리기
        ego_cx = self.width() / 2
        ego_cy = self.height() * 0.75  # 화면 아래쪽 75% 위치

        # 차량 크기 (미터 단위 → 픽셀)
        vehicle_length = 4.47 * self.scale
        vehicle_width = 1.82 * self.scale
        
        # 차량 사각형
        painter.setBrush(QBrush(QColor(0, 200, 0, 150)))
        painter.setPen(QPen(QColor(0, 255, 0), 2))
        
        rect = QRectF(
            cx - vehicle_width/2,
            cy - vehicle_length/2,
            vehicle_width,
            vehicle_length
        )
        painter.drawRect(rect)
        
        # 전방 표시 (삼각형)
        painter.setBrush(QBrush(QColor(255, 255, 0)))
        triangle = QPolygonF([
            QPointF(cx, cy - vehicle_length/2 - 10),
            QPointF(cx - 10, cy - vehicle_length/2),
            QPointF(cx + 10, cy - vehicle_length/2)
        ])
        painter.drawPolygon(triangle)
        
        # "EGO" 텍스트
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.drawText(int(cx - 15), int(cy + 5), "EGO")
        
    def draw_objects(self, painter, cx, cy):
        """오브젝트 그리기"""
        for obj in self.objects:
            # 상대 좌표 → 화면 좌표 변환
            # 객체가 자차 앞쪽(+y)에 있으면 화면 위쪽에 표시
            screen_x = cx + obj['x'] * self.scale
            screen_y = cy - obj['y'] * self.scale  # y축 반전
            
            obj_width = obj.get('width', 2.0) * self.scale
            obj_length = obj.get('length', 4.0) * self.scale
            
            # 오브젝트 타입에 따라 색상 변경
            if obj['type'] == 'car':
                color = QColor(255, 100, 100)
            elif obj['type'] == 'pedestrian':
                color = QColor(100, 100, 255)
            else:
                color = QColor(200, 200, 200)
                
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(color.darker(), 2))
            
            rect = QRectF(
                screen_x - obj_width/2,
                screen_y - obj_length/2,
                obj_width,
                obj_length
            )
            painter.drawRect(rect)
            
            # 거리 표시
            distance = math.sqrt(obj['x']**2 + obj['y']**2)
            painter.setPen(QPen(QColor(255, 255, 255)))
            painter.drawText(
                int(screen_x - 20), 
                int(screen_y - obj_length/2 - 5), 
                f"{distance:.1f}m"
            )
    def wheelEvent(self, event):
        """마우스 휠로 줌 조정"""
        delta = event.angleDelta().y()
        # 줌 팩터
        zoom_factor = 1.2
        
        if delta > 0:
            # 휠 위로 → 줌 아웃 → scale 감소
            self.scale /= zoom_factor
        else:
            # 휠 아래로 → 줌 인 → scale 증가
            self.scale *= zoom_factor
        
        # 줌 레벨 제한
        self.scale = max(1.0, min(50.0, self.scale))
        self.update()

class StatusIndicator(QLabel):
    """센서 상태 표시 위젯"""
    
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.status = 2  # 0: 정상, 1: 경고, 2: 에러
        self.setFixedSize(120, 40)
        self.update_display()
        
    def set_status(self, status):
        self.status = status
        self.update_display()
        
    def update_display(self):
        if self.status == 0:  # 정상
            bg_color = "#28a745"
            text_color = "white"
        elif self.status == 1:  # 경고
            bg_color = "#ff8c00"
            text_color = "white"
        else:  # 에러
            bg_color = "#dc3545"
            text_color = "white"
            
        self.setStyleSheet(f"""
            QLabel {{
                background-color: {bg_color};
                color: {text_color};
                border: 2px solid #333;
                border-radius: 5px;
                font-size: 14px;
                font-weight: bold;
                padding: 5px;
            }}
        """)
        self.setText(self.name)
        self.setAlignment(Qt.AlignCenter)

class MainDisplayWindow(QMainWindow):
    """메인 디스플레이 윈도우"""
    
    # Qt Signal 정의
    update_sensors_signal = pyqtSignal()
    update_vehicle_signal = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        
        # ROS 초기화
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
        self.odd_status = 2
        
        self.eps_status = 0
        self.traffic_light_color = 0
        self.traffic_light_time = 0
        self.speed_limit = 0
        self.current_speed = 0  # 추가

        # 모드 제어 변수: 기본값 Manual = 0
        self.selected_mode = 0  # 0: Manual, 1: Auto

        # UI 초기화
        self.init_ui()
        
        # 지도 로딩 (경로 수정 필요)
        map_path = "/home/yuyeong/mcar/src/localization/gps_system_localizer/src/A2_LINK_epsg5179.shp"
        if os.path.exists(map_path):
            self.vehicle_view.load_map(map_path)
            rospy.loginfo(f"Map loaded: {map_path}")
        else:
            rospy.logwarn(f"Map file not found: {map_path}")

        # ROS Subscribers
        self.init_ros_subscribers()
        
        # ROS Publisher 추가 (모드 명령)
        self.mode_command_pub = rospy.Publisher('/vehicle/mode_command', UInt8, queue_size=1)

        # Signal 연결
        self.update_sensors_signal.connect(self.update_sensor_display)
        self.update_vehicle_signal.connect(self.update_vehicle_view)
        
        # Ctrl+C 핸들러 등록
        signal.signal(signal.SIGINT, self.signal_handler)

        # Python 인터럽트 처리를 위한 타이머
        self.interrupt_timer = QTimer()
        self.interrupt_timer.timeout.connect(lambda: None)
        self.interrupt_timer.start(100)

        # 타이머 (주기적 업데이트)
        self.timer = QTimer()
        self.timer.timeout.connect(self.periodic_update)
        self.timer.start(100)  # 100ms
        
    def init_ui(self):
        """UI 초기화"""
        self.setWindowTitle("Vehicle Display System")
        self.setGeometry(100, 100, 1600, 900)
        
        # 메인 위젯
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # 전체 레이아웃
        main_layout = QHBoxLayout()
        
        # === 왼쪽 패널 (센서 상태 + 정보) ===
        left_panel = self.create_left_panel()
        
        # === 오른쪽 패널 (차량 뷰) ===
        right_panel = self.create_vehicle_view()
        
        # 레이아웃에 추가
        main_layout.addWidget(left_panel, 30)
        main_layout.addWidget(right_panel, 70)
        
        main_widget.setLayout(main_layout)
        
        # 다크 테마 적용
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
        """왼쪽 패널 생성 (센서 상태)"""
        panel = QWidget()
        layout = QVBoxLayout()
        
        # === 시스템 상태 그룹 ===
        status_group = QGroupBox("System Status")
        status_layout = QGridLayout()
        
        # 상태 인디케이터 생성
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
        
        # 그리드에 배치 (2열)
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
        
        # === 주행 모드 ===
        mode_group = QGroupBox("Current Driving Mode")
        mode_layout = QVBoxLayout()
        
        # 상단: 현재 모드 표시 (큰 박스)
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
        
        # 하단: 선택 버튼 (Autonomous / Manual)
        button_container = QWidget()
        button_layout = QHBoxLayout()
        button_layout.setContentsMargins(0, 5, 0, 0)
        button_layout.setSpacing(0)

        # Autonomous 버튼
        self.auto_button = QPushButton("Autonomous")
        self.auto_button.setCheckable(True)
        self.auto_button.setChecked(False)  # 기본값: 선택 안 됨
        self.auto_button.setFixedHeight(40)
        self.auto_button.clicked.connect(self.on_auto_button_clicked)
        self.auto_button.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: black;
                font-size: 14px;
                font-weight: normal;
                border: 2px solid #6c757d;
                border-right: 1px solid #6c757d;
                border-radius: 0px;
                padding: 5px;
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

        # Manual 버튼
        self.manual_button = QPushButton("Manual")
        self.manual_button.setCheckable(True)
        self.manual_button.setChecked(True)  # 기본값: Manual
        self.manual_button.setFixedHeight(40)
        self.manual_button.clicked.connect(self.on_manual_button_clicked)
        self.manual_button.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: black;
                font-size: 14px;
                font-weight: normal;
                border: 2px solid #6c757d;
                border-left: 1px solid #6c757d;
                border-radius: 0px;
                padding: 5px;
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
        layout.addWidget(mode_group)

        # === 속도 정보 (수정된 부분) ===
        speed_group = QGroupBox("Speed Information")
        speed_main_layout = QVBoxLayout()
        
        # 좌우 레이아웃
        speed_h_layout = QHBoxLayout()
        
        # 왼쪽: 속도 제한
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
        
        # 오른쪽: 현재 속도
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
        
        # 좌우 배치
        speed_h_layout.addWidget(speed_limit_container)
        speed_h_layout.addWidget(current_speed_container)
        
        speed_main_layout.addLayout(speed_h_layout)
        speed_group.setLayout(speed_main_layout)
        layout.addWidget(speed_group)
        
        layout.addStretch()
        panel.setLayout(layout)
        return panel
        
    def create_vehicle_view(self):
        """차량 뷰 패널 생성"""
        panel = QWidget()
        layout = QVBoxLayout()
        
        title = QLabel("Vehicle Top View")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: white;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        self.vehicle_view = VehicleViewWidget()
        layout.addWidget(self.vehicle_view)
        
        panel.setLayout(layout)
        return panel
        
    def init_ros_subscribers(self):
        """ROS Subscriber 초기화"""
        rospy.Subscriber("/diagnostic/cpt7_gps", cpt7_gps_diagnostic_msg, 
                        self.gps_callback)
        rospy.Subscriber("/diagnostic/adcu", k_adcu_diagnostic_msg, 
                        self.adcu_callback)
        rospy.Subscriber("/diagnostic/lidar", lidar_diagnostic_msg, 
                        self.lidar_callback)
        rospy.Subscriber("/diagnostic/radar", radar_diagnostic_msg, 
                        self.radar_callback)
        rospy.Subscriber("/diagnostic/v2x", v2x_diagnostic_msg, 
                        self.v2x_callback)
        rospy.Subscriber("/diagnostic/hmi", hmi_diagnostic_msg, 
                        self.hmi_callback)
        rospy.Subscriber("/diagnostic/vcu", vcu_diagnostic_msg, 
                        self.vcu_callback)
        rospy.Subscriber("/diagnostic/cam", cam_diagnostic_msg, 
                        self.cam_callback)
        rospy.Subscriber("/diagnostic/ipc", ipc_diagnostic_msg, 
                        self.ipc_callback)
        rospy.Subscriber("/sensors/chassis", chassis_msg, 
                        self.chassis_callback)
        rospy.Subscriber("/localization/to_control_team", 
                        to_control_team_from_local_msg, 
                        self.local_callback)
        rospy.Subscriber("/katri_v2x_node/katri_spat", 
                        intersection_array_msg, 
                        self.traffic_light_callback)
        
        # 오브젝트 정보 (예시 토픽)
        # rospy.Subscriber("/perception/objects", ObjectArray, 
        #                 self.objects_callback)
        
    def gps_callback(self, msg):
        # AliveCount 체크 로직 필요
        self.gps_status = 0  # 임시
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
        
    def chassis_callback(self, msg):
        self.eps_status = msg.vcu_EPS_Status
        self.current_speed = getattr(msg, 'vehicle_speed', 0)  # 필드명에 맞게 수정
        
    def local_callback(self, msg):
        self.speed_limit = msg.Speed_Limit
        self.odd_status = msg.Road_State

        # 자차 위치 및 heading 업데이트
        ego_x = msg.host_east  # UTM East
        ego_y = msg.host_north  # UTM North
        ego_heading = msg.host_yaw  # 라디안
        
        self.vehicle_view.set_ego_pose(ego_x, ego_y, ego_heading)
        
    def traffic_light_callback(self, msg):
        # 신호등 처리 로직 (C++ 코드 참조)
        pass
        
    def objects_callback(self, msg):
        """오브젝트 정보 수신"""
        objects = []
        for obj in msg.objects:
            objects.append({
                'x': obj.pose.position.x,
                'y': obj.pose.position.y,
                'width': 2.0,
                'length': 4.0,
                'type': 'car'
            })
        self.vehicle_view.set_objects(objects)
        
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
        """차량 뷰 업데이트"""
        pass
        
    def periodic_update(self):
        """주기적 업데이트"""
        # 모드 명령 발행 (100ms마다)
        mode_msg = UInt8()
        mode_msg.data = self.selected_mode  # 0: Manual, 1: Auto
        self.mode_command_pub.publish(mode_msg)
        
        # 현재 모드 표시 업데이트 (실제 차량 상태 반영)
        if self.eps_status == 2:
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
            # 실제 상태와 버튼 동기화
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
            # 실제 상태와 버튼 동기화
            if not self.manual_button.isChecked():
                self.manual_button.setChecked(True)
                self.auto_button.setChecked(False)
            
        # 속도 제한 업데이트
        self.speed_label.setText(str(self.speed_limit))
        # 현재 속도 업데이트 (추가)
        self.current_speed_label.setText(str(int(self.current_speed)))
        
        # 신호등 업데이트
        if self.traffic_light_time > 0:
            seconds = self.traffic_light_time // 10
            self.traffic_label.setText(f"{seconds}s")
            
            if self.traffic_light_color == 1:  # 초록
                color = "#28a745"
            elif self.traffic_light_color == 2:  # 주황
                color = "#ff8c00"
            elif self.traffic_light_color == 3:  # 빨강
                color = "#dc3545"
            else:
                color = "#666"
                
            self.traffic_label.setStyleSheet(f"""
                QLabel {{
                    background-color: {color};
                    font-size: 36px;
                    font-weight: bold;
                    padding: 20px;
                    border-radius: 10px;
                    color: white;
                }}
            """)

    def on_auto_button_clicked(self):
        """Autonomous 버튼 클릭"""
        if self.auto_button.isChecked():
            self.manual_button.setChecked(False)
            self.selected_mode = 1
            rospy.loginfo("모드 변경: Autonomous")
        else:
            # 체크 해제 방지 (하나는 항상 선택되어야 함)
            self.auto_button.setChecked(True)

    def on_manual_button_clicked(self):
        """Manual 버튼 클릭"""
        if self.manual_button.isChecked():
            self.auto_button.setChecked(False)
            self.selected_mode = 0
            rospy.loginfo("모드 변경: Manual")
        else:
            # 체크 해제 방지
            self.manual_button.setChecked(True)

    def signal_handler(self, sig, frame):
        """Ctrl+C 처리"""
        print("\n시그널을 받았습니다. 프로그램을 종료합니다...")
        self.cleanup()
        QApplication.quit()
        
    def cleanup(self):
        """종료 시 정리 작업"""
        print("리소스 정리 중...")
        rospy.signal_shutdown("User interrupted")
        
    def closeEvent(self, event):
        """윈도우 닫기 이벤트"""
        self.cleanup()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = MainDisplayWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
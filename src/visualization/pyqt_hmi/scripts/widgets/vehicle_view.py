#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPainterPath, QPolygonF
from PyQt5.QtCore import QPointF, QRectF
import math

import rospy
from visualization_msgs.msg import MarkerArray

from utils.shapefile_loader import load_shapefile

class VehicleViewWidget(QWidget):
    """자차 및 오브젝트를 2D 탑뷰로 표시하는 위젯"""
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(800, 600)
        
        # 자차 위치
        self.ego_x = 0
        self.ego_y = 0
        self.ego_heading = 0
        
        # 오브젝트 리스트
        self.objects = []
        
        # 줌 레벨
        self.scale = 10.0
        
        # 지도 데이터
        self.map_features = []
        
        # 배경색
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(self.backgroundRole(), QColor(30, 30, 30))
        self.setPalette(palette)
        
        # ROS 초기화
        self.init_ros()
        
        # UI 업데이트 타이머
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update)
        self.update_timer.start(50)  # 20Hz
    
    def init_ros(self):
        """ROS 노드 초기화"""
        try:
            # rospy.init_node('pyqt_hmi_vehicle_view', anonymous=True)
            # rviz_filter 노드가 NodeHandle("~")를 사용하므로 토픽 이름이 /rviz_filter/visualization_marker_array
            self.marker_sub = rospy.Subscriber('/rviz_filter_node/visualization_marker_array', MarkerArray, self.marker_callback, queue_size=10)
            # rospy.loginfo("VehicleViewWidget ROS node initialized")
            rospy.loginfo("Subscribed to: /rviz_filter/visualization_marker_array")
        except rospy.exceptions.ROSException as e:
            rospy.logwarn(f"ROS already initialized: {e}")
            try:
                self.marker_sub = rospy.Subscriber('/rviz_filter_node/visualization_marker_array', MarkerArray, self.marker_callback, queue_size=10)
                rospy.loginfo("Subscribed to: /rviz_filter/visualization_marker_array")
            except Exception as sub_error:
                rospy.logerr(f"Failed to subscribe: {sub_error}")
    
    def marker_callback(self, msg):
        """RVIZ MarkerArray 메시지 수신"""
        rospy.loginfo(f"Received MarkerArray with {len(msg.markers)} markers")
        
        try:
            self.objects = []
            self.refpos = None
            sdsm_count = 0
            
            # MarkerArray에서 'sdsm_objects' namespace의 마커만 추출
            for marker in msg.markers:
                rospy.loginfo(f"Marker namespace: {marker.ns}, id: {marker.id}")
                
                if marker.ns == 'sdsm_refpos':
                    self.refpos = {
                        'x': marker.pose.position.x,
                        'y': marker.pose.position.y,
                        'z': marker.pose.position.z
                    }
                    rospy.loginfo(f"RefPos found: ({self.refpos['x']:.2f}, {self.refpos['y']:.2f})")
                
                if marker.ns == 'sdsm_objects':
                    sdsm_count += 1
                    # 마커 정보를 객체 딕셔너리로 변환
                    obj_x = marker.pose.position.x
                    obj_y = marker.pose.position.y
                    
                    # 크기 정보
                    width = marker.scale.y  # y축이 width
                    length = marker.scale.x  # x축이 length
                    
                    # 색상으로 타입 판별
                    if marker.color.r > 0.9 and marker.color.g < 0.1 and marker.color.b < 0.1:
                        obj_type = 'car'  # 빨간색 = vehicle
                    elif marker.color.r > 0.9 and marker.color.g > 0.4 and marker.color.b < 0.1:
                        obj_type = 'pedestrian'  # 주황색 = vru
                    else:
                        obj_type = 'unknown'  # 회색 = unknown
                    
                    # Quaternion을 yaw(heading)으로 변환
                    qx = marker.pose.orientation.x
                    qy = marker.pose.orientation.y
                    qz = marker.pose.orientation.z
                    qw = marker.pose.orientation.w
                    
                    # Quaternion to Euler (yaw만 필요)
                    siny_cosp = 2 * (qw * qz + qx * qy)
                    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                    heading = math.atan2(siny_cosp, cosy_cosp)
                    
                    obj_data = {
                        'id': marker.id - 200,  # ID 오프셋 제거
                        'type': obj_type,
                        'x': obj_x,
                        'y': obj_y,
                        'heading': heading,
                        'width': width,
                        'length': length
                    }
                    
                    self.objects.append(obj_data)
                    rospy.loginfo(f"Added SDSM object: ID={obj_data['id']}, type={obj_type}, pos=({obj_x:.2f}, {obj_y:.2f})")
            
            rospy.loginfo(f"Total SDSM objects extracted: {sdsm_count}")
            
        except Exception as e:
            rospy.logerr(f"Error in marker callback: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def load_map(self, shp_file):
        """Load map from shapefile"""
        self.map_features = load_shapefile(shp_file)
        self.update()

    def set_ego_pose(self, x, y, heading):
        self.ego_x = x
        self.ego_y = y
        self.ego_heading = heading
        self.update()
        
    def set_objects(self, objects):
        self.objects = objects
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
        
        # 2. 회전 적용
        rot_x, rot_y = self.rotate_point(rel_x, rel_y, -self.ego_heading + math.pi/2)
        
        # 3. 화면 좌표로 변환
        center_x = self.width() / 2
        center_y = self.height() * 0.75
        
        screen_x = center_x + rot_x * self.scale
        screen_y = center_y - rot_y * self.scale
        
        return screen_x, screen_y
        
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
        
        # 지도 그리기
        self.draw_map(painter)

        # RefPos 그리기 (객체보다 먼저 그려서 아래에 표시)
        self.draw_refpos(painter, center_x, center_y)

        # 자차 그리기
        self.draw_ego_vehicle(painter, center_x, center_y)
        
        # 오브젝트 그리기
        self.draw_objects(painter, center_x, center_y)
        
        # 디버깅 정보
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.drawText(10, 20, f"Scale: {self.scale:.1f}m/div")
        painter.drawText(10, 40, f"Ego: ({self.ego_x:.1f}, {self.ego_y:.1f})")
        painter.drawText(10, 60, f"Heading: {math.degrees(self.ego_heading):.1f}°")
        painter.drawText(10, 80, f"Objects: {len(self.objects)}")
        painter.drawText(10, 100, f"Features: {len(self.map_features)}")
        
    def draw_grid(self, painter, cx, cy):
        """그리드 그리기"""
        painter.setPen(QPen(QColor(60, 60, 60), 1, Qt.DashLine))
        
        grid_size = 100
        
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
        
        painter.setPen(QPen(QColor(242, 217, 132, 100), 2))
        
        for feature in self.map_features:
            points = feature['points']
            
            if len(points) < 2:
                continue
            
            path = QPainterPath()
            
            first_point = points[0]
            screen_x, screen_y = self.world_to_screen(first_point[0], first_point[1])
            
            if self.is_point_out_of_view(screen_x, screen_y, margin=100):
                continue
                
            path.moveTo(screen_x, screen_y)
            
            valid_points = 1
            for i in range(1, len(points)):
                point = points[i]
                screen_x, screen_y = self.world_to_screen(point[0], point[1])
                
                if abs(screen_x) < 10000 and abs(screen_y) < 10000:
                    path.lineTo(screen_x, screen_y)
                    valid_points += 1
            
            if valid_points >= 2:
                painter.drawPath(path)

    def is_point_out_of_view(self, x, y, margin=0):
        """화면 밖 여부 확인"""
        return (x < -margin or x > self.width() + margin or 
                y < -margin or y > self.height() + margin)

    def draw_refpos(self, painter, cx, cy):
        """RefPos 마커 그리기"""
        if not self.refpos:
            return
        
        # RefPos의 화면 좌표 계산
        refpos_x = self.refpos['x']
        refpos_y = self.refpos['y']
        
        screen_x = cx + refpos_x * self.scale
        screen_y = cy - refpos_y * self.scale
        
        # 화면 밖이면 스킵
        if self.is_point_out_of_view(screen_x, screen_y, margin=50):
            return
        
        # 파란색 원으로 표시
        radius = 15
        painter.setBrush(QBrush(QColor(0, 0, 255, 180)))
        painter.setPen(QPen(QColor(0, 100, 255), 3))
        painter.drawEllipse(QPointF(screen_x, screen_y), radius, radius)
        
        # 중심점 표시
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawLine(int(screen_x - 5), int(screen_y), int(screen_x + 5), int(screen_y))
        painter.drawLine(int(screen_x), int(screen_y - 5), int(screen_x), int(screen_y + 5))
        
        # RefPos 텍스트 표시
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.drawText(
            int(screen_x - 30), 
            int(screen_y - radius - 5), 
            "RefPos"
        )
        
        # 거리 표시
        distance = math.sqrt(refpos_x**2 + refpos_y**2)
        painter.drawText(
            int(screen_x - 30), 
            int(screen_y + radius + 15), 
            f"{distance:.1f}m"
        )

    def draw_ego_vehicle(self, painter, cx, cy):
        """자차 그리기"""
        ego_cx = self.width() / 2
        ego_cy = self.height() * 0.75

        vehicle_length = 4.47 * self.scale
        vehicle_width = 1.82 * self.scale
        
        painter.setBrush(QBrush(QColor(0, 200, 0, 150)))
        painter.setPen(QPen(QColor(0, 255, 0), 2))
        
        rect = QRectF(
            cx - vehicle_width/2,
            cy - vehicle_length/2,
            vehicle_width,
            vehicle_length
        )
        painter.drawRect(rect)
        
        painter.setBrush(QBrush(QColor(255, 255, 0)))
        triangle = QPolygonF([
            QPointF(cx, cy - vehicle_length/2 - 10),
            QPointF(cx - 10, cy - vehicle_length/2),
            QPointF(cx + 10, cy - vehicle_length/2)
        ])
        painter.drawPolygon(triangle)
        
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.drawText(int(cx - 15), int(cy + 5), "EGO")
        
    def draw_objects(self, painter, cx, cy):
        """오브젝트 그리기 (SDSM 객체, heading 적용)"""
        for obj in self.objects:
            # 객체의 상대 좌표
            obj_x = obj['x']
            obj_y = obj['y']
            
            # 화면 좌표로 변환
            screen_x = cx + obj_x * self.scale
            screen_y = cy - obj_y * self.scale
            
            # 화면 밖이면 스킵
            if self.is_point_out_of_view(screen_x, screen_y, margin=50):
                continue
            
            obj_width = obj.get('width', 2.0) * self.scale
            obj_length = obj.get('length', 4.0) * self.scale
            obj_heading = obj.get('heading', 0.0)
            
            # 객체 타입별 색상
            if obj['type'] == 'car':
                color = QColor(255, 100, 100)
            elif obj['type'] == 'pedestrian':
                color = QColor(255, 128, 0)
            else:
                color = QColor(128, 128, 128)
            
            # 회전 변환을 위한 painter save
            painter.save()

            try:
                painter.translate(screen_x, screen_y)
                
                # 객체의 heading 회전 적용 (화면 좌표계 고려)
                painter.rotate(math.degrees(-obj_heading))
                
                # 객체 박스 그리기
                painter.setBrush(QBrush(color))
                painter.setPen(QPen(color.darker(), 2))
                
                rect = QRectF(
                    -obj_width/2,
                    -obj_length/2,
                    obj_width,
                    obj_length
                )
                painter.drawRect(rect)
                
                # 방향 표시 (삼각형)
                painter.setBrush(QBrush(QColor(255, 255, 0)))
                triangle = QPolygonF([
                    QPointF(0, -obj_length/2 - 5),
                    QPointF(-5, -obj_length/2),
                    QPointF(5, -obj_length/2)
                ])
                painter.drawPolygon(triangle)
            
            finally:
                # 반드시 restore 호출
                painter.restore()
            # painter.translate(screen_x, screen_y)
            
            # # 객체의 heading 회전 적용 (화면 좌표계 고려)
            # painter.rotate(math.degrees(-obj_heading))
            
            # # 객체 박스 그리기
            # painter.setBrush(QBrush(color))
            # painter.setPen(QPen(color.darker(), 2))
            
            # rect = QRectF(
            #     -obj_width/2,
            #     -obj_length/2,
            #     obj_width,
            #     obj_length
            # )
            # painter.drawRect(rect)
            
            # # 방향 표시 (삼각형)
            # painter.setBrush(QBrush(QColor(255, 255, 0)))
            # triangle = QPolygonF([
            #     QPointF(0, -obj_length/2 - 5),
            #     QPointF(-5, -obj_length/2),
            #     QPointF(5, -obj_length/2)
            # ])
            # painter.drawPolygon(triangle)
            
            # painter.restore()
            
            # 거리 및 ID 표시
            distance = math.sqrt(obj_x**2 + obj_y**2)
            obj_id = obj.get('id', '?')
            painter.setPen(QPen(QColor(255, 255, 255)))
            painter.drawText(
                int(screen_x - 30), 
                int(screen_y - obj_length/2 - 10), 
                f"ID:{obj_id} {distance:.1f}m"
            )
            
    def wheelEvent(self, event):
        """마우스 휠로 줌 조정"""
        delta = event.angleDelta().y()
        zoom_factor = 1.2
        
        if delta > 0:
            self.scale /= zoom_factor
        else:
            self.scale *= zoom_factor
        
        self.scale = max(1.0, min(50.0, self.scale))
        self.update()
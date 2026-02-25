#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPainterPath, QPolygonF
from PyQt5.QtCore import QPointF, QRectF
import math

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
        
        # 스티어링 각도 (deg)
        self.steering_angle = 0.0

        # 줌 레벨
        self.scale = 10.0
        
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
        
    def set_steering_angle(self, angle):
        self.steering_angle = angle
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
        center_y = self.height() * 0.5
        
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
        center_y = self.height() * 0.5
        
        # 그리드 그리기
        self.draw_grid(painter, center_x, center_y)
        
        # 지도 그리기
        self.draw_map(painter)

        # 자차 그리기
        self.draw_ego_vehicle(painter, center_x, center_y)
        
        # 오브젝트 그리기
        self.draw_objects(painter, center_x, center_y)
        
        # 디버깅 정보
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.drawText(10, 20, f"Scale: {self.scale:.1f}m/div")
        painter.drawText(10, 40, f"Ego: ({self.ego_x:.1f}, {self.ego_y:.1f})")
        painter.drawText(10, 60, f"Heading: {math.degrees(self.ego_heading):.1f}°")
        painter.drawText(10, 80, f"Features: {len(self.map_features)}")

        # 스티어링 아이콘
        self.draw_steering_icon(painter, 10, 95)
        
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
        """오브젝트 그리기"""
        for obj in self.objects:
            screen_x = cx + obj['x'] * self.scale
            screen_y = cy - obj['y'] * self.scale
            
            obj_width = obj.get('width', 2.0) * self.scale
            obj_length = obj.get('length', 4.0) * self.scale
            
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
            
            distance = math.sqrt(obj['x']**2 + obj['y']**2)
            painter.setPen(QPen(QColor(255, 255, 255)))
            painter.drawText(
                int(screen_x - 20), 
                int(screen_y - obj_length/2 - 5), 
                f"{distance:.1f}m"
            )
            
    def draw_steering_icon(self, painter, x, y):
        """스티어링 휠 아이콘 그리기"""
        cx = x + 55
        cy = y + 55
        r_outer = 48
        r_inner = 36
        hub_r = 14

        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-self.steering_angle)

        # 림 (두꺼운 도넛 형태)
        rim_path = QPainterPath()
        rim_path.addEllipse(QPointF(0, 0), r_outer, r_outer)
        inner_path = QPainterPath()
        inner_path.addEllipse(QPointF(0, 0), r_inner, r_inner)
        rim_only = rim_path - inner_path
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(180, 140, 100)))
        painter.drawPath(rim_only)
        # 림 외곽선
        painter.setPen(QPen(QColor(120, 90, 60), 1.5))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(0, 0), r_outer, r_outer)
        painter.drawEllipse(QPointF(0, 0), r_inner, r_inner)

        # 스포크 3개 (둥근 끝)
        spoke_pen = QPen(QColor(160, 160, 170), 7, Qt.SolidLine, Qt.RoundCap)
        painter.setPen(spoke_pen)
        # 위쪽
        painter.drawLine(QPointF(0, -hub_r), QPointF(0, -r_inner))
        # 왼쪽 아래
        lx = -math.sin(math.radians(60))
        ly = math.cos(math.radians(60))
        painter.drawLine(QPointF(lx * hub_r, ly * hub_r), QPointF(lx * r_inner, ly * r_inner))
        # 오른쪽 아래
        rx = math.sin(math.radians(60))
        ry = math.cos(math.radians(60))
        painter.drawLine(QPointF(rx * hub_r, ry * hub_r), QPointF(rx * r_inner, ry * r_inner))

        # 중심 허브
        painter.setPen(QPen(QColor(100, 100, 110), 1.5))
        painter.setBrush(QBrush(QColor(70, 70, 80)))
        painter.drawEllipse(QPointF(0, 0), hub_r, hub_r)
        # 허브 내부 작은 원
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(90, 90, 100)))
        painter.drawEllipse(QPointF(0, 0), 6, 6)

        # 12시 방향 마커
        painter.setBrush(QBrush(QColor(255, 80, 80)))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QPointF(0, -r_outer + 6), 4, 4)

        painter.restore()

        # 각도 텍스트
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.drawText(x - 5, cy + r_outer + 18, f"Steer: {self.steering_angle:.1f}")

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
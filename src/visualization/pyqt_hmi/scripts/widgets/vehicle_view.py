#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPainterPath, QPolygonF, QFont, QLinearGradient
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

        # 계획 경로(arc): from_Control arc_len/arc_kappa/arc_ds
        self.planned_arc = None  # (arc_len, arc_kappa, arc_ds)
        self.arc_visible = True   # 주행경로(arc) 표출 On/Off

        # 스티어링 각도 (deg)
        self.steering_angle = 0.0

        # 줌 레벨
        self.scale = 10.0
        
        # 지도 데이터
        self.map_features = []
        
        # 배경색
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(self.backgroundRole(), QColor(22, 25, 30))
        self.setPalette(palette)

    def load_map(self, shp_file):
        """Load map from shapefile"""
        self.map_features = load_shapefile(shp_file)

    def set_ego_pose(self, x, y, heading):
        self.ego_x = x
        self.ego_y = y
        self.ego_heading = heading

    def set_steering_angle(self, angle):
        self.steering_angle = angle

    def set_objects(self, objects):
        self.objects = objects

    def set_planned_arc(self, arc_len, arc_kappa, arc_ds):
        self.planned_arc = (arc_len, arc_kappa, arc_ds)

    def set_arc_visible(self, on):
        self.arc_visible = bool(on)

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
        painter.fillRect(self.rect(), QColor(22, 25, 30))
        
        # 중심점 계산
        center_x = self.width() / 2
        center_y = self.height() * 0.5
        
        # 그리드 그리기
        self.draw_grid(painter, center_x, center_y)
        
        # 지도 그리기
        self.draw_map(painter)

        # 계획 경로(arc) 그리기
        self.draw_planned_arc(painter, center_x, center_y)

        # 자차 그리기
        self.draw_ego_vehicle(painter, center_x, center_y)
        
        # 오브젝트 그리기
        self.draw_objects(painter, center_x, center_y)
        
        # 정보 오버레이 (좌상단)
        painter.setFont(QFont("Monospace", 9))
        painter.setPen(QPen(QColor(160, 170, 180), 1))
        info_x, info_y = 12, 20
        painter.drawText(info_x, info_y, f"Heading  {math.degrees(self.ego_heading):.1f}\u00b0")
        painter.drawText(info_x, info_y + 18, f"E {self.ego_x:.1f}  N {self.ego_y:.1f}")

        # 스티어링 아이콘
        self.draw_steering_icon(painter, 10, info_y + 28)
        
    def draw_grid(self, painter, cx, cy):
        """그리드 그리기 - 동심원 + 십자선"""
        # 동심원 (거리 표시)
        painter.setPen(QPen(QColor(50, 55, 60), 1, Qt.DotLine))
        font = QFont("Monospace", 8)
        painter.setFont(font)
        for dist_m in range(10, 101, 10):
            r = dist_m * self.scale
            painter.drawEllipse(QPointF(cx, cy), r, r)
            if dist_m % 20 == 0:
                painter.setPen(QPen(QColor(90, 95, 100), 1))
                painter.drawText(int(cx + r + 3), int(cy - 3), f"{dist_m}m")
                painter.setPen(QPen(QColor(50, 55, 60), 1, Qt.DotLine))

        # 십자선
        painter.setPen(QPen(QColor(70, 75, 80), 1))
        painter.drawLine(int(cx), 0, int(cx), self.height())
        painter.drawLine(0, int(cy), self.width(), int(cy))
        
    def draw_map(self, painter):
        """지도 그리기 - 자차 주변 200m 이내 feature만 렌더링"""
        if not self.map_features:
            return

        painter.setPen(QPen(QColor(242, 217, 132, 100), 2))

        view_range = 200.0  # meters

        for feature in self.map_features:
            # bbox 기반 거리 필터 (자차에서 200m 이내만)
            bbox = feature.get('bbox')
            if bbox:
                bx_min, by_min, bx_max, by_max = bbox
                if (bx_max < self.ego_x - view_range or bx_min > self.ego_x + view_range or
                    by_max < self.ego_y - view_range or by_min > self.ego_y + view_range):
                    continue

            points = feature['points']
            if len(points) < 2:
                continue

            path = QPainterPath()
            first_point = points[0]
            screen_x, screen_y = self.world_to_screen(first_point[0], first_point[1])
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

    def draw_planned_arc(self, painter, cx, cy):
        """계획 경로(arc) 그리기 - body frame(전방 x=위, 좌측 y+=왼쪽)"""
        if not self.arc_visible or self.planned_arc is None:
            return
        arc_len, arc_kappa, arc_ds = self.planned_arc
        if arc_len <= 0:
            return

        if arc_ds <= 0:
            arc_ds = 0.5
        n = int(round(arc_len / arc_ds)) + 1
        n = max(2, min(400, n))

        k = arc_kappa
        path = QPainterPath()
        for i in range(n):
            s = arc_len * i / (n - 1)
            if abs(k) < 1e-4:
                x = s
                y = 0.0
            else:
                x = math.sin(k * s) / k
                y = (1.0 - math.cos(k * s)) / k
            # body → 화면: 전방 x=위쪽, 좌측 y(+)=왼쪽 (draw_objects와 동일 규약)
            screen_x = cx - y * self.scale
            screen_y = cy - x * self.scale
            if i == 0:
                path.moveTo(screen_x, screen_y)
            else:
                path.lineTo(screen_x, screen_y)

        painter.setPen(QPen(QColor(0, 230, 180), 3))
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(path)

    def is_point_out_of_view(self, x, y, margin=0):
        """화면 밖 여부 확인"""
        return (x < -margin or x > self.width() + margin or 
                y < -margin or y > self.height() + margin)

    def draw_ego_vehicle(self, painter, cx, cy):
        """자차 그리기 - IONIQ 5 스타일"""
        vehicle_length = 4.47 * self.scale
        vehicle_width = 1.82 * self.scale
        wheel_w = max(3, vehicle_width * 0.12)
        wheel_h = max(6, vehicle_length * 0.13)
        wheelbase_front = vehicle_length * 0.32
        wheelbase_rear = vehicle_length * 0.28
        track = vehicle_width * 0.42

        # --- 차체 본체 (둥근 사각형) ---
        body_rect = QRectF(cx - vehicle_width/2, cy - vehicle_length/2,
                           vehicle_width, vehicle_length)
        body_radius = min(vehicle_width, vehicle_length) * 0.12

        # 그림자
        shadow_offset = max(2, self.scale * 0.3)
        shadow_rect = body_rect.adjusted(shadow_offset, shadow_offset,
                                         shadow_offset, shadow_offset)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(0, 0, 0, 60)))
        painter.drawRoundedRect(shadow_rect, body_radius, body_radius)

        # 차체
        body_gradient = QLinearGradient(body_rect.topLeft(), body_rect.bottomLeft())
        body_gradient.setColorAt(0.0, QColor(55, 65, 80))
        body_gradient.setColorAt(0.5, QColor(45, 55, 70))
        body_gradient.setColorAt(1.0, QColor(35, 45, 60))
        painter.setBrush(QBrush(body_gradient))
        painter.setPen(QPen(QColor(80, 180, 220, 180), max(1, self.scale * 0.12)))
        painter.drawRoundedRect(body_rect, body_radius, body_radius)

        # --- 앞 유리 ---
        ws_w = vehicle_width * 0.7
        ws_h = vehicle_length * 0.18
        ws_y = cy - vehicle_length/2 + vehicle_length * 0.15
        ws_rect = QRectF(cx - ws_w/2, ws_y, ws_w, ws_h)
        ws_gradient = QLinearGradient(ws_rect.topLeft(), ws_rect.bottomLeft())
        ws_gradient.setColorAt(0.0, QColor(120, 180, 220, 160))
        ws_gradient.setColorAt(1.0, QColor(80, 140, 180, 120))
        painter.setBrush(QBrush(ws_gradient))
        painter.setPen(QPen(QColor(100, 160, 200, 140), max(1, self.scale * 0.06)))
        ws_radius = min(ws_w, ws_h) * 0.2
        painter.drawRoundedRect(ws_rect, ws_radius, ws_radius)

        # --- 뒤 유리 ---
        rw_w = vehicle_width * 0.6
        rw_h = vehicle_length * 0.10
        rw_y = cy + vehicle_length/2 - vehicle_length * 0.18
        rw_rect = QRectF(cx - rw_w/2, rw_y, rw_w, rw_h)
        rw_gradient = QLinearGradient(rw_rect.topLeft(), rw_rect.bottomLeft())
        rw_gradient.setColorAt(0.0, QColor(80, 140, 180, 120))
        rw_gradient.setColorAt(1.0, QColor(100, 160, 200, 140))
        painter.setBrush(QBrush(rw_gradient))
        painter.setPen(QPen(QColor(100, 160, 200, 140), max(1, self.scale * 0.06)))
        rw_radius = min(rw_w, rw_h) * 0.2
        painter.drawRoundedRect(rw_rect, rw_radius, rw_radius)

        # --- 헤드라이트 ---
        hl_w = vehicle_width * 0.18
        hl_h = max(3, vehicle_length * 0.04)
        hl_y = cy - vehicle_length/2 + max(2, vehicle_length * 0.03)
        for side in [-1, 1]:
            hl_x = cx + side * vehicle_width * 0.28 - hl_w/2
            hl_rect = QRectF(hl_x, hl_y, hl_w, hl_h)
            painter.setBrush(QBrush(QColor(255, 255, 240, 220)))
            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(hl_rect, hl_h * 0.4, hl_h * 0.4)

        # --- 테일라이트 ---
        tl_w = vehicle_width * 0.22
        tl_h = max(3, vehicle_length * 0.03)
        tl_y = cy + vehicle_length/2 - max(2, vehicle_length * 0.03) - tl_h
        for side in [-1, 1]:
            tl_x = cx + side * vehicle_width * 0.26 - tl_w/2
            tl_rect = QRectF(tl_x, tl_y, tl_w, tl_h)
            painter.setBrush(QBrush(QColor(255, 60, 60, 220)))
            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(tl_rect, tl_h * 0.4, tl_h * 0.4)

        # --- 바퀴 4개 ---
        wheel_positions = [
            (cx - track - wheel_w/2, cy - wheelbase_front - wheel_h/2),
            (cx + track + wheel_w/2 - wheel_w, cy - wheelbase_front - wheel_h/2),
            (cx - track - wheel_w/2, cy + wheelbase_rear - wheel_h/2),
            (cx + track + wheel_w/2 - wheel_w, cy + wheelbase_rear - wheel_h/2),
        ]
        painter.setBrush(QBrush(QColor(25, 25, 25)))
        painter.setPen(QPen(QColor(60, 60, 60), max(1, self.scale * 0.06)))
        for wx, wy in wheel_positions:
            painter.drawRoundedRect(QRectF(wx, wy, wheel_w, wheel_h), 2, 2)

        # --- 진행 방향 화살표 ---
        arrow_y = cy - vehicle_length/2 - max(8, self.scale * 1.2)
        arrow_size = max(6, self.scale * 0.6)
        arrow_path = QPainterPath()
        arrow_path.moveTo(cx, arrow_y - arrow_size)
        arrow_path.lineTo(cx - arrow_size * 0.7, arrow_y + arrow_size * 0.3)
        arrow_path.lineTo(cx + arrow_size * 0.7, arrow_y + arrow_size * 0.3)
        arrow_path.closeSubpath()
        painter.setBrush(QBrush(QColor(80, 200, 255, 200)))
        painter.setPen(Qt.NoPen)
        painter.drawPath(arrow_path)
        
    def draw_objects(self, painter, cx, cy):
        """오브젝트 그리기 - orientation 회전 적용"""
        for obj in self.objects:
            screen_x = cx - obj['y'] * self.scale
            screen_y = cy - obj['x'] * self.scale

            obj_width = obj.get('width', 2.0) * self.scale
            obj_length = obj.get('length', 4.0) * self.scale
            orientation = obj.get('orientation', 0.0)

            if obj['type'] == 'car':
                fill_color = QColor(220, 80, 80, 160)
                border_color = QColor(255, 120, 120, 200)
            elif obj['type'] == 'pedestrian':
                fill_color = QColor(80, 120, 255, 160)
                border_color = QColor(120, 160, 255, 200)
            else:
                fill_color = QColor(180, 180, 180, 140)
                border_color = QColor(210, 210, 210, 180)

            painter.save()
            painter.translate(screen_x, screen_y)
            # orientation: 센서 좌표계(x=앞, y=왼) 기준 라디안 → 화면 회전각
            painter.rotate(-math.degrees(orientation))

            rect = QRectF(-obj_width/2, -obj_length/2, obj_width, obj_length)
            r = min(obj_width, obj_length) * 0.15
            painter.setBrush(QBrush(fill_color))
            painter.setPen(QPen(border_color, max(1, self.scale * 0.08)))
            painter.drawRoundedRect(rect, r, r)

            # 진행 방향 표시 (앞쪽 삼각형)
            arrow_size = max(4, obj_width * 0.3)
            painter.setBrush(QBrush(QColor(255, 255, 255, 150)))
            painter.setPen(Qt.NoPen)
            arrow = QPolygonF([
                QPointF(0, -obj_length/2 - arrow_size * 0.6),
                QPointF(-arrow_size * 0.5, -obj_length/2 + 1),
                QPointF(arrow_size * 0.5, -obj_length/2 + 1),
            ])
            painter.drawPolygon(arrow)

            painter.restore()

            # 텍스트 (회전 없이)
            distance = math.sqrt(obj['x']**2 + obj['y']**2)
            painter.setFont(QFont("Monospace", 8))
            painter.setPen(QPen(QColor(255, 255, 255, 200)))
            obj_id = obj.get('id', '')
            painter.drawText(int(screen_x - 22), int(screen_y - obj_length/2 - 4),
                             f"#{obj_id} {distance:.1f}m")
            
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
        """마우스 휠로 줌 조정 (창이 활성 + 커서가 위젯 위일 때만)"""
        # WSLg/XWayland 는 wheel 을 키보드 포커스가 아닌 커서 위치 기준으로
        # 전달해, 다른 창을 선택(Ctrl+Tab)해도 커서가 이 위에 있으면 줌이 먹는다.
        # pyqt 창이 활성일 때만 줌 처리하고 그 외에는 이벤트를 넘긴다.
        if not self.isActiveWindow():
            event.ignore()
            return

        delta = event.angleDelta().y()
        zoom_factor = 1.2

        if delta > 0:
            self.scale /= zoom_factor
        else:
            self.scale *= zoom_factor

        self.scale = max(1.0, min(50.0, self.scale))
        self.update()
        event.accept()
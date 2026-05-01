# -*- coding: utf-8 -*-
"""A-1 vehicle view: range rings + detection cones + Tesla top-down ego.

Inherits VehicleViewWidget so map / object / steering / world_to_screen logic
is reused unchanged. paintEvent is replaced; draw_grid / draw_ego_vehicle
are overridden with A-1 styling.
"""
import math

from PyQt5.QtCore import Qt, QPointF, QRectF
from PyQt5.QtGui import (QBrush, QColor, QFont, QLinearGradient, QPainter,
                         QPainterPath, QPen, QTransform)

from utils.theme import (AMBER_0, BG_0, TEXT_3, mono_font)
from widgets.vehicle_view import VehicleViewWidget


_RING_RADII = [(10, "10m", False), (25, "25m", True),
               (50, "50m", False), (100, "100m", False)]

# Pseudo-3D camera: vertical compression simulates a forward-tilted view.
# 1.00 = pure top-down, 0.50 ≈ 60° tilt. 0.62 ≈ isometric-ish.
_TILT_Y_SCALE = 0.62


class VehicleViewA1(VehicleViewWidget):
    def __init__(self):
        super().__init__()
        # darker, design-token background
        palette = self.palette()
        palette.setColor(self.backgroundRole(), BG_0)
        self.setPalette(palette)

    # ─── paintEvent: world tilted, HUD upright ───
    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)
        p.fillRect(self.rect(), BG_0)

        cx = self.width() / 2
        cy = self.height() / 2

        # World layer — isometric tilt around the ego (cx, cy)
        p.save()
        tilt = QTransform()
        tilt.translate(cx, cy)
        tilt.scale(1.0, _TILT_Y_SCALE)
        tilt.translate(-cx, -cy)
        p.setWorldTransform(tilt)

        self._draw_rings(p, cx, cy)
        self._draw_detection_cones(p, cx, cy)
        self.draw_map(p)
        self.draw_objects(p, cx, cy)
        self._draw_ego_a1(p, cx, cy)
        p.restore()

        # HUD layer — untilted overlays
        self._draw_corners(p)
        p.setFont(mono_font(9))
        p.setPen(QPen(QColor(160, 170, 180), 1))
        info_y = 22
        p.drawText(12, info_y,
                   f"HEADING  {math.degrees(self.ego_heading):.1f}°")
        p.drawText(12, info_y + 16,
                   f"E {self.ego_x:.1f}  N {self.ego_y:.1f}")

        # steering icon bottom-left
        self.draw_steering_icon(p, 10, self.height() - 140)

    # ─── A-1 range rings (replaces draw_grid) ────────────────────
    def _draw_rings(self, p, cx, cy):
        p.save()
        for dist_m, label, solid in _RING_RADII:
            r = dist_m * self.scale
            if solid:
                pen = QPen(QColor(255, 181, 71, 90), 1, Qt.SolidLine)
            else:
                pen = QPen(QColor(200, 204, 210, 30), 1, Qt.DashLine)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(cx, cy), r, r)
            # label chip on the right edge
            p.setPen(TEXT_3)
            p.setFont(mono_font(8))
            tw = 44
            th = 14
            tx = cx + r - tw / 2
            ty = cy - th / 2
            p.fillRect(QRectF(tx, ty, tw, th), BG_0)
            p.drawText(QRectF(tx, ty, tw, th), Qt.AlignCenter, label)
        p.restore()

    # ─── forward amber + rear blue cones ─────────────────────────
    def _draw_detection_cones(self, p, cx, cy):
        p.save()
        p.translate(cx, cy)

        # Forward cone — pointing up (-y in screen). Arc spans 60°→120° (centered at 90°).
        # Rotate so it always points along the ego heading? In top-down with
        # heading already applied to map/objects, the cone is in screen space:
        # forward = up, rear = down.
        forward = QPainterPath()
        forward.moveTo(0, 0)
        forward.arcTo(QRectF(-330, -330, 660, 660), 60, 60)
        forward.closeSubpath()
        p.setBrush(QColor(255, 181, 71, 26))
        p.setPen(QPen(QColor(255, 181, 71, 90), 1))
        p.drawPath(forward)

        rear = QPainterPath()
        rear.moveTo(0, 0)
        rear.arcTo(QRectF(-240, -240, 480, 480), 240, 60)
        rear.closeSubpath()
        p.setBrush(QColor(96, 165, 250, 16))
        p.setPen(QPen(QColor(96, 165, 250, 64), 1))
        p.drawPath(rear)
        p.restore()

    # ─── Tesla-style top-down ego body around (cx, cy) ───────────
    def _draw_ego_a1(self, p, cx, cy):
        p.save()
        p.translate(cx, cy)
        # scale body to match self.scale (nominal car: 4.6m × 1.85m).
        body_h = 4.6 * self.scale * 0.5
        body_w = 1.85 * self.scale * 0.5

        body = QLinearGradient(0, -body_h, 0, body_h)
        body.setColorAt(0.00, QColor("#E8EAED"))
        body.setColorAt(0.35, QColor("#F4F5F7"))
        body.setColorAt(0.65, QColor("#D5D8DC"))
        body.setColorAt(1.00, QColor("#A8ADB3"))

        path = QPainterPath()
        path.moveTo(-body_w * 0.85, -body_h + 6)
        path.quadTo(-body_w * 0.85, -body_h, -body_w * 0.6, -body_h)
        path.lineTo(body_w * 0.6, -body_h)
        path.quadTo(body_w * 0.85, -body_h, body_w * 0.85, -body_h + 6)
        path.lineTo(body_w, -body_h * 0.5)
        path.lineTo(body_w, body_h * 0.5)
        path.lineTo(body_w * 0.85, body_h - 6)
        path.quadTo(body_w * 0.85, body_h, body_w * 0.6, body_h)
        path.lineTo(-body_w * 0.6, body_h)
        path.quadTo(-body_w * 0.85, body_h, -body_w * 0.85, body_h - 6)
        path.lineTo(-body_w, body_h * 0.5)
        path.lineTo(-body_w, -body_h * 0.5)
        path.closeSubpath()
        p.setBrush(QBrush(body))
        p.setPen(QPen(QColor(0, 0, 0, 64), 0.5))
        p.drawPath(path)

        # specular highlight strip
        p.setBrush(QColor(255, 255, 255, 38))
        p.setPen(Qt.NoPen)
        p.drawRect(QRectF(-body_w * 0.4, -body_h + 4, body_w * 0.8, body_h * 2 - 8))

        # glass — front windshield trapezoid, roof, rear windshield
        glass = QLinearGradient(0, -body_h * 0.6, 0, body_h * 0.6)
        glass.setColorAt(0, QColor("#1F2329"))
        glass.setColorAt(1, QColor("#0E1115"))
        p.setBrush(QBrush(glass))
        p.setPen(QPen(QColor(0, 0, 0, 100), 0.5))

        ws = QPainterPath()
        ws.moveTo(-body_w * 0.7, -body_h * 0.55)
        ws.lineTo(body_w * 0.7, -body_h * 0.55)
        ws.lineTo(body_w * 0.55, -body_h * 0.25)
        ws.lineTo(-body_w * 0.55, -body_h * 0.25)
        ws.closeSubpath()
        p.drawPath(ws)

        p.drawRoundedRect(QRectF(-body_w * 0.55, -body_h * 0.23,
                                 body_w * 1.1, body_h * 0.55),
                          2, 2)

        rs = QPainterPath()
        rs.moveTo(-body_w * 0.55, body_h * 0.33)
        rs.lineTo(body_w * 0.55, body_h * 0.33)
        rs.lineTo(body_w * 0.7, body_h * 0.65)
        rs.lineTo(-body_w * 0.7, body_h * 0.65)
        rs.closeSubpath()
        p.drawPath(rs)

        # mirrors
        p.setBrush(QBrush(body))
        p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(-body_w * 0.95, -body_h * 0.4), 4, 3)
        p.drawEllipse(QPointF( body_w * 0.95, -body_h * 0.4), 4, 3)

        # door cut lines
        p.setPen(QPen(QColor(0, 0, 0, 38), 0.4))
        p.drawLine(QPointF(-body_w * 0.85, -body_h * 0.15),
                   QPointF( body_w * 0.85, -body_h * 0.15))
        p.drawLine(QPointF(-body_w * 0.85,  body_h * 0.22),
                   QPointF( body_w * 0.85,  body_h * 0.22))

        # signature lights
        p.setBrush(AMBER_0); p.setPen(Qt.NoPen)
        p.drawRect(QRectF(-body_w * 0.7, -body_h - 1.5, body_w * 1.4, 1.5))
        p.setBrush(QColor("#F87171"))
        p.drawRect(QRectF(-body_w * 0.7,  body_h, body_w * 1.4, 1.5))

        # heading chevron above car
        p.setPen(QPen(AMBER_0, 1.4, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        p.setBrush(Qt.NoBrush)
        chev = QPainterPath()
        chev_y = -body_h - 14
        chev.moveTo(-6, chev_y); chev.lineTo(0, chev_y - 6); chev.lineTo(6, chev_y)
        p.drawPath(chev)
        p.restore()

    # ─── corner labels ───────────────────────────────────────────
    def _draw_corners(self, p):
        p.setPen(TEXT_3)
        p.setFont(mono_font(9))
        p.drawText(QRectF(self.width() - 240, 16, 220, 16),
                   Qt.AlignRight, "FRONT · LIDAR + CAM")
        p.drawText(QRectF(self.width() - 240, self.height() - 28, 220, 16),
                   Qt.AlignRight, "PROJ · UTM 52N")
        p.drawText(QRectF(self.width() - 240, self.height() - 44, 220, 16),
                   Qt.AlignRight, f"SCALE · 1px = {1.0/self.scale:.2f} m"
                   if self.scale > 0 else "SCALE · —")

    # ─── disable parent's draw_grid (replaced by _draw_rings) ────
    def draw_grid(self, painter, cx, cy):
        return  # no concentric dotted grid in A-1

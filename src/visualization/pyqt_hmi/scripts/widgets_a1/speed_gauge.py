# -*- coding: utf-8 -*-
"""270° amber arc speed gauge with center number + mode label."""
import math

from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QPainter, QPen, QLinearGradient
from PyQt5.QtWidgets import QWidget

from utils.theme import (AMBER_0, AMBER_2, BG_3, RED, TEXT_0, TEXT_2, TEXT_3,
                         display_font, mono_font)


class SpeedGauge(QWidget):
    def __init__(self, parent=None, vmax=120):
        super().__init__(parent)
        self._v = 0.0
        self._vmax = vmax
        self._mode = "MANUAL"
        self._aeb = False
        self.setMinimumSize(272, 272)

    def set_value(self, kmh):
        self._v = max(0.0, min(self._vmax, float(kmh)))
        self.update()

    def set_mode(self, mode):
        self._mode = (mode or "").upper()
        self.update()

    def set_aeb(self, on):
        new_state = bool(on)
        if new_state != self._aeb:
            self._aeb = new_state
            self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)

        w = min(self.width(), self.height())
        cx, cy = self.width() / 2, self.height() / 2
        r = w / 2 - 18

        start = 225 * 16
        span_total = -270 * 16
        progress = self._v / self._vmax if self._vmax else 0

        p.setPen(QPen(BG_3, 2))
        p.drawArc(QRectF(cx - r, cy - r, 2 * r, 2 * r), start, span_total)

        if self._aeb:
            pen = QPen(RED, 4, Qt.SolidLine, Qt.RoundCap)
        else:
            grad = QLinearGradient(cx - r, cy, cx + r, cy)
            grad.setColorAt(0.0, AMBER_2)
            grad.setColorAt(1.0, AMBER_0)
            pen = QPen(grad, 3, Qt.SolidLine, Qt.RoundCap)
        p.setPen(pen)
        p.drawArc(QRectF(cx - r, cy - r, 2 * r, 2 * r),
                  start, int(span_total * progress))

        active_color = RED if self._aeb else AMBER_0
        for i in range(0, self._vmax + 1, 10):
            a = math.radians(225 - 270 * (i / self._vmax))
            major = (i % 20 == 0)
            inner = r - (14 if major else 8)
            past = (i / self._vmax) <= progress
            x0 = cx + inner * math.cos(a); y0 = cy - inner * math.sin(a)
            x1 = cx + r * math.cos(a);     y1 = cy - r * math.sin(a)
            p.setPen(QPen(active_color if past else BG_3, 1.5 if major else 1))
            p.drawLine(int(x0), int(y0), int(x1), int(y1))

        p.setPen(TEXT_0)
        p.setFont(display_font(40, weight=300))
        text = str(int(round(self._v)))
        p.drawText(self.rect().adjusted(0, -8, 0, -8), Qt.AlignCenter, text)

        p.setPen(TEXT_3)
        p.setFont(mono_font(9))
        p.drawText(self.rect().adjusted(0, 56, 0, 56),
                   Qt.AlignHCenter | Qt.AlignVCenter, "KM / H")

        p.setPen(RED if self._aeb else TEXT_2)
        sub = "AEB ACTIVE" if self._aeb else f"{self._mode} • DRIVE"
        p.drawText(self.rect().adjusted(0, 78, 0, 78),
                   Qt.AlignHCenter | Qt.AlignVCenter, sub)

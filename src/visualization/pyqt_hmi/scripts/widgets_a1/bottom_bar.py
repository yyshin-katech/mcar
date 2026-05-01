# -*- coding: utf-8 -*-
"""Bottom bar: traffic-light mini + popup banner + action buttons."""
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QPainter, QPen
from PyQt5.QtWidgets import (QFrame, QHBoxLayout, QLabel, QPushButton,
                             QSizePolicy, QVBoxLayout, QWidget)

from utils.theme import (AMBER_0, BG_2, BOTTOM_H, GREEN, LINE, RED, TEXT_2,
                         display_font, labeled_mono_font, mono_font)


_LIGHT_COLOR = {
    1: ("#34D399", "GREEN"),    # green
    2: ("#FFB547", "AMBER"),    # orange / amber
    3: ("#F87171", "RED"),      # red
}


class _TrafficChip(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._color = 0
        self._time_s = 0
        self.setFixedHeight(48)
        self.setMinimumWidth(140)

    def set_state(self, color, time_decisec):
        self._color = int(color) if color is not None else 0
        self._time_s = (int(time_decisec) // 10) if time_decisec else 0
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)

        rect = self.rect().adjusted(0, 4, -2, -4)
        p.setPen(QPen(LINE, 1))
        p.setBrush(QBrush(BG_2))
        p.drawRoundedRect(rect, 8, 8)

        # circle
        cy = rect.center().y()
        cx = rect.left() + 24
        p.setPen(Qt.NoPen)
        if self._color in _LIGHT_COLOR:
            hex_, _ = _LIGHT_COLOR[self._color]
            p.setBrush(QBrush(QColor(hex_)))
        else:
            p.setBrush(QBrush(QColor("#3A3F45")))
        p.drawEllipse(cx - 10, cy - 10, 20, 20)

        # text
        p.setPen(TEXT_2)
        p.setFont(labeled_mono_font(9, 2))
        label = _LIGHT_COLOR.get(self._color, ("", "—"))[1] if self._color else "—"
        p.drawText(cx + 18, cy - 3, label)
        if self._color and self._time_s > 0:
            p.setPen(AMBER_0)
            p.setFont(mono_font(11, weight=500))
            p.drawText(cx + 18, cy + 14, f"{self._time_s:>3} s")


class BottomBar(QFrame):
    mode_request = pyqtSignal(int)  # 0=manual, 1=auto

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("bottom")
        self.setFixedHeight(BOTTOM_H)

        h = QHBoxLayout(self)
        h.setContentsMargins(20, 14, 20, 14)
        h.setSpacing(16)

        self._traffic = _TrafficChip()
        h.addWidget(self._traffic)

        # popup / message banner (center, takes stretch)
        self._popup = QLabel("")
        self._popup.setFont(display_font(15, weight=500))
        self._popup.setAlignment(Qt.AlignCenter)
        self._popup.setStyleSheet("color: #F4F5F7; padding: 6px 14px; border-radius: 8px;")
        self._popup.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        h.addWidget(self._popup, 1)

        # action buttons
        self._btn_manual = QPushButton("MANUAL")
        self._btn_manual.setMinimumWidth(120)
        self._btn_manual.clicked.connect(lambda: self.mode_request.emit(0))

        self._btn_auto = QPushButton("AUTONOMOUS")
        self._btn_auto.setProperty("variant", "primary")
        self._btn_auto.setMinimumWidth(140)
        self._btn_auto.clicked.connect(lambda: self.mode_request.emit(1))

        h.addWidget(self._btn_manual)
        h.addWidget(self._btn_auto)

    def set_traffic(self, color, time_decisec):
        self._traffic.set_state(color, time_decisec)

    def set_popup(self, text, severity="info"):
        if not text:
            self._popup.setText("")
            self._popup.setStyleSheet("color: #5A616B; padding: 6px 14px; border-radius: 8px;")
            return
        self._popup.setText(text)
        if severity == "error":
            self._popup.setStyleSheet(
                "color: #F4F5F7; background: #2A1418; border: 1px solid #3A2024;"
                " padding: 6px 14px; border-radius: 8px;")
        elif severity == "warn":
            self._popup.setStyleSheet(
                "color: #1A1206; background: #FFB547;"
                " padding: 6px 14px; border-radius: 8px;")
        else:
            self._popup.setStyleSheet(
                "color: #F4F5F7; background: #1C1F23; border: 1px solid #232830;"
                " padding: 6px 14px; border-radius: 8px;")

    def set_mode(self, autonomous):
        if autonomous:
            self._btn_auto.setEnabled(False)
            self._btn_manual.setEnabled(True)
        else:
            self._btn_auto.setEnabled(True)
            self._btn_manual.setEnabled(False)
